#include <iostream>
#include <thread>
#include <vector>
#include <memory>
#include <chrono>
#include <cstring>
#include <cmath> // Required for Trigonometry
#include <algorithm>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fstream>
#include <iomanip>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/failure/failure.h>
#include <mavsdk/plugins/param/param.h>

using namespace mavsdk;
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::this_thread::sleep_for;

// INDIVIDUAL DRONE CLASS
class DroneNode {
    public:
        int id;
        std::shared_ptr<System> system;
        std::unique_ptr<Action> action;
        std::unique_ptr<Telemetry> telemetry;
        std::unique_ptr<Offboard> offboard;
        std::unique_ptr<Mission> mission;

        DroneNode(int drone_id, std::shared_ptr<System> sys) : id(drone_id), system(sys) {
            action = std::make_unique<Action>(system);
            telemetry = std::make_unique<Telemetry>(system);
            offboard = std::make_unique<Offboard>(system);
            mission = std::make_unique<Mission>(system);
        }

        void performTakeoff() {
            std::cout << "[Drone " << id << "] takeoff sequence initiated..." << std::endl;
            action->arm();
            action->takeoff();
        }
};

// SWARM MANAGER CLASS
class SwarmManager {
    private:
        Mavsdk mavsdk;
        std::vector<std::shared_ptr<DroneNode>> fleet;
        std::vector<std::shared_ptr<mavsdk::Mission>> mission_plugins;
        // Added for collision avoidance
        std::vector<std::shared_ptr<mavsdk::Telemetry>> telemetry_plugins;
        std::vector<std::shared_ptr<mavsdk::Action>> action_plugins;
        bool is_formation_active = false;

    public:
        SwarmManager() : mavsdk(Mavsdk::Configuration{ComponentType::GroundStation}) {}

        void listenForPorts(){
            std::cout << "[MANAGER] Waiting for drones..." << std::endl;
            mavsdk.add_any_connection("udpin://0.0.0.0:14541");
            mavsdk.add_any_connection("udpin://0.0.0.0:14542");
            mavsdk.add_any_connection("udpin://0.0.0.0:14543");
        }

        void assembleFleet(){
            while(mavsdk.systems().size() < 3){
                sleep_for(seconds(1));
            }
            int id_counter = 1;
            for (auto sys : mavsdk.systems()){
                fleet.push_back(std::make_shared<DroneNode>(id_counter++, sys));
            }
            std::cout << "[MANAGER] " << fleet.size() << " drones successfully joined the fleet" << std::endl;
        }

        void synchronizedTakeoff() {
            std::cout << "[MANAGER] Staggered takeoff initiated..." << std::endl;
            for (auto& drone : fleet){
                drone->performTakeoff();
                sleep_for(seconds(4)); 
            }
            std::cout << "[MANAGER] All drones are airborne. Awaiting mission" << std::endl;
        }

        // LINE ABREAST FORMATION (TRANSIT)
        void mode_LineAbreastFormation(int duration_seconds){
            std::cout << "[MANAGER] MODE: Line Abreast Formation started. Transiting to target area..." << std::endl;
            is_formation_active = true;

            auto leader = fleet[0];
            auto left_wing = fleet[1];
            auto right_wing = fleet[2];

            // Start followers as Threads (Left: -1, Right: +1)
            std::thread left_thread(&SwarmManager::followerOffboardLoop, this, left_wing, leader, -1);
            std::thread right_thread(&SwarmManager::followerOffboardLoop, this, right_wing, leader, 1);
            
            // Initiate leader movement
            leader->offboard->set_position_ned(Offboard::PositionNedYaw{});
            leader->offboard->start();

            float leader_north_target = 0.0f;

            // Fly in formation for the specified duration
            for(int i=0; i<(duration_seconds * 20); ++i) { // 20Hz loop
                leader_north_target += 0.1f;
                Offboard::PositionNedYaw leader_target{};
                leader_target.north_m = leader_north_target;
                leader_target.east_m = 0.0f;
                leader_target.down_m = -20.0f;
                leader_target.yaw_deg = 0.0f;

                leader->offboard->set_position_ned(leader_target);
                sleep_for(milliseconds(50));
            }

            // End formation and stop leader
            is_formation_active = false;
            leader->offboard->stop();
            left_thread.join();
            right_thread.join();
            std::cout << "[MANAGER] Transit complete. Breaking formation." << std::endl;
        }

        // Inner loop for followers
        void followerOffboardLoop(std::shared_ptr<DroneNode> follower, std::shared_ptr<DroneNode> leader, int direction_multiplier) {
            follower->offboard->set_position_ned(Offboard::PositionNedYaw{});
            follower->offboard->start();

            while (is_formation_active) {
                auto leader_pos = leader->telemetry->position_velocity_ned().position;
                Offboard::PositionNedYaw follower_target{};
                follower_target.north_m = leader_pos.north_m;
                follower_target.east_m = leader_pos.east_m + (5.0f * direction_multiplier);
                follower_target.down_m = leader_pos.down_m;
                follower_target.yaw_deg = 0.0f;

                follower->offboard->set_position_ned(follower_target);
                sleep_for(milliseconds(50));
            }
            follower->offboard->stop();
        }

        // COMB PATTERN SCANNING
        void mode_UnifiedCombSweep() {
            std::cout << "[MANAGER] MODE: Unified Swarm Sweep (Comb Pattern Lawnmower) started..." << std::endl;
            
            // Gazebo Standard Origin Point (Zurich/Switzerland) 
            const float origin_lat = 47.397742f;
            const float origin_lon = 8.545594;
            
            const float altitude = 25.0f; // Scanning altitude
            const float speed = 5.0f; // Flying speed

            // Forest Borders
            const float x_min = -80.0f;
            const float x_max = 80.0f;
            const float y_min = 10.0f;
            const float y_max = 150.0f;

            // Distance between drones
            const float strip_spacing = 6.0f;

            int num_drones = fleet.size();
            if (num_drones == 0) return;

            // Total width cowered by the swarm in a single pass (3 drones * 6 meters = 18 meters)
            float swarm_step = num_drones * strip_spacing;

            for (int i = 0; i < num_drones; ++i) {
                auto mission_plugin = std::make_shared<mavsdk::Mission>(fleet[i]->system);
                mission_plugins.push_back(mission_plugin);

                auto telemetry_plugin = std::make_shared<mavsdk::Telemetry>(fleet[i]->system);
                telemetry_plugins.push_back(telemetry_plugin);

                auto action_plugin = std::make_shared<mavsdk::Action>(fleet[i]->system);
                action_plugins.push_back(action_plugin);

                // Waiting 500 miliseconds for communication channel ready
                std::this_thread::sleep_for(std::chrono::milliseconds(500));

                // Clear old missions
                mission_plugin->clear_mission();

                mavsdk::Mission::MissionPlan mission_plan;

                // Starting from left
                float current_swarm_base_x = x_min;
                bool flying_up = true; // Zigzag direction

                // The swarm continues to the right side of the forest
                while (current_swarm_base_x <= x_max) {
                    // The drone's own X position within the swarm (0, 6, or 12 meters to the right)
                    float drone_x = current_swarm_base_x + (i * strip_spacing);

                    // To the top or to the bottom
                    float current_y_start = flying_up ? y_min : y_max;
                    float current_y_end = flying_up ? y_max : y_min;

                    // Start of the strip
                    mavsdk::Mission::MissionItem item_start;
                    item_start.latitude_deg = origin_lat + (current_y_start / 111320.0f);  // 1° latitude = 111320 meters
                    item_start.longitude_deg = origin_lon + (drone_x / 75440.0f); // 1° longitude at Zurich = 111320 * cos (47.397742) = 75440 meters
                    item_start.relative_altitude_m = altitude;
                    item_start.speed_m_s = speed;
                    item_start.is_fly_through = true;
                    mission_plan.mission_items.push_back(item_start);

                    // End of the strip
                    mavsdk::Mission::MissionItem item_end;
                    item_end.latitude_deg = origin_lat + (current_y_end / 111320.0f);
                    item_end.longitude_deg = origin_lon + (drone_x / 75440.0f);
                    item_end.relative_altitude_m = altitude;
                    item_end.speed_m_s = speed;
                    item_end.is_fly_through = true;
                    mission_plan.mission_items.push_back(item_end);

                    // Move the swarm 18 meters to the right
                    flying_up = !flying_up;
                    current_swarm_base_x += swarm_step;

                }

                // Upload the mission and check the result
                mavsdk::Mission::Result upload_result = mission_plugin->upload_mission(mission_plan);

                if (upload_result == mavsdk::Mission::Result::Success) {
                    std::cout << "[MANAGER] Mission uploaded to Drone " << i << " successfully. Starting..." << std::endl;
                    mission_plugin->start_mission();
                } else {
                    std::cout << "[ERROR] Mission upload failed for Drone " << i << ": " << upload_result << std::endl;
                }
            }
        }

        // COLLISION AVOIDANCE
        void check_swarm_collision() {
            const float safety_radius = 5.0f; // 5 meters safety border
            const float evasion_alt = 35.0f; // A safe upper altitude to escape to in case of collision
            int num_drones = fleet.size();

            // Loop that compares each drone to all other drones 
            for (int i = 0; i < num_drones; ++i) {
                for (int j = i + 1; j < num_drones; ++j) {

                    // Real-time GPS and altitude data
                    auto pos1 = telemetry_plugins[i]->position();
                    auto pos2 = telemetry_plugins[j]->position();

                    // Converting GPS degrees to meters
                    float dx = (pos2.longitude_deg - pos1.longitude_deg) * 75440.0f;
                    float dy = (pos2.latitude_deg - pos1.latitude_deg) *111320.0f;
                    float dz = pos2.relative_altitude_m - pos1.relative_altitude_m;

                    // 3-Dimensional Euclidean distance
                    float distance = std::sqrt(dx*dx + dy*dy + dz*dz);

                    // If the drones get closer than 5 meters to each other
                    if (distance < safety_radius) {
                        std::cout << "\n[CRITICAL WARNING] COLLISION RISK DETECTED!" << std::endl;
                        std::cout << "->Drone " << i << " and Drone " << j << " are too close: "
                                  << std::fixed << std::setprecision(2) << distance << "m apart." << std::endl;

                        std::cout << "[ATC] Executing Evasion Maneuver: Drone " << j
                                  << " climbing to " << evasion_alt << "m!" << std::endl;
                        
                        action_plugins[j]->goto_location(pos2.latitude_deg, pos2.longitude_deg, evasion_alt, 0);
                    }

                }
            }
        }


        // TRIANGLE (V-SHAPE) FORMATION
        void mode_TriangleFormation(int duration_seconds) {
            std::cout << "[MANAGER] MODE: Triangle (V-Shape) Formation initiated..." << std::endl;
            is_formation_active = true;

            auto leader = fleet[0];
            auto left_follower = fleet[1];
            auto right_follower = fleet[2];

            // Offsets: 5m behind, 5m to the sides
            float back_offset = -5.0f;
            float side_offset = 5.0f;

            // Use Lambda for Threads (left: -5m east, right: +5m east)
            std::thread left_thread([this, left_follower, leader, back_offset, side_offset]() {
                this->followerTriangleLoop(left_follower, leader, back_offset, -side_offset);
            });
            std::thread right_thread([this, right_follower, leader, back_offset, side_offset](){
                this->followerTriangleLoop(right_follower, leader, back_offset, side_offset);
            });

            // Leader movement logic
            leader->offboard->start();
            float leader_n = 0.0f;
            for(int i = 0; i < (duration_seconds * 20); ++i) {
                leader_n += 0.1f;
                Offboard::PositionNedYaw target{};
                target.north_m = leader_n;
                target.down_m = -20.0f;
                leader->offboard->set_position_ned(target);
                sleep_for(milliseconds(20));
            }

            is_formation_active = false;
            left_thread.join();
            right_thread.join();
        }

        void followerTriangleLoop(std::shared_ptr<DroneNode> follower, std::shared_ptr<DroneNode> leader, float n_offset, float e_offset) {
            follower->offboard->start();

            while (is_formation_active) {
                auto leader_pos = leader->telemetry->position_velocity_ned().position;
                Offboard::PositionNedYaw target{};
                target.north_m = leader_pos.north_m + n_offset;
                target.east_m = leader_pos.east_m + e_offset;
                target.down_m = leader_pos.down_m;
                target.yaw_deg = 0.0f;
                follower->offboard->set_position_ned(target);
                sleep_for(milliseconds(50));
            }
            follower->offboard->stop();
        }

        // CYBER ATTACK
        void execute_cyber_attack(int choice) {
            for(auto& drone : fleet) {
                auto param = mavsdk::Param{drone->system};
                param.set_param_int("SYS_FAILURE_EN", 1);
                auto failure = mavsdk::Failure{drone->system};

                if (choice == 1) failure.inject(mavsdk::Failure::FailureUnit::SensorGps, mavsdk::Failure::FailureType::Off, 0);
                else if (choice == 2) failure.inject(mavsdk::Failure::FailureUnit::SensorGps, mavsdk::Failure::FailureType::Garbage, 0);
                else if (choice == 3) failure.inject(mavsdk::Failure::FailureUnit::SensorGps, mavsdk::Failure::FailureType::Ok, 0);
            }
            if (choice == 1) std::cout << "\n[CYBER ATTACK] GPS SIGNAL LOST!" << std::endl;
            else if (choice == 2) std::cout << "\n[CYBER ATTACK] GPS NOISE INJECTED!" << std::endl;
            else if (choice == 3) std::cout << "\n[RECOVERY] GPS RESTORED." << std::endl;
        }

        // BACKGROUND UDP LISTENER
        void run_udp_listener() {
            int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
            struct sockaddr_in servaddr;
            memset(&servaddr, 0, sizeof(servaddr));
            servaddr.sin_family = AF_INET;
            servaddr.sin_addr.s_addr = INADDR_ANY;
            servaddr.sin_port = htons(9090);
            
            bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr));
            char buffer[1024];
            
            std::cout << "[SYSTEM] External Cyber Attack Listener active on UDP Port 9090" <<std::endl;

            while(true) {
                int n = recvfrom(sockfd, (char *)buffer, 1024, MSG_WAITALL, NULL, NULL);
                buffer[n] = '\0';
                std::string msg(buffer);

                try{
                    // "1" -> Deny, "2" -> Spoof, "3" -> Recover
                    if (msg == "1") {
                        std::cout << "\n[CYBER THREAT] WARNING: GPS Denial detected across the swarm!" << std::endl;
                        std::cout << "[DEFENSE] Autonomous Defense Activated: All UAVs climbing to +30m safe altitude..." << std::endl;

                        for (auto& drone : fleet) {
                            // Initialize MAVSDK Plugins
                            auto telemetry = mavsdk::Telemetry{drone->system};
                            auto action = mavsdk::Action{drone->system};
                            auto param = mavsdk::Param{drone->system};

                            // Get the current position
                            auto current_pos = telemetry.position();

                            // Calculate the safe altitude (+30m)
                            float safe_alt_absolute = current_pos.absolute_altitude_m + 30.0f;

                            // Send the autonomous recovery command
                            action.goto_location(current_pos.latitude_deg, current_pos.longitude_deg, safe_alt_absolute, 0.0f);

                            // Block the GPS sensor inside PX4
                            param.set_param_int("SIM_GPS_BLOCK", 1);

                        }
                    }
                    else if (msg == "2") {
                        std::cout << "\n[CYBER THREAT] Spoofing attack signal received." << std::endl;
                        std::cout << "[DEFENSE] Autonomous Defense: Expanding formation to prevent mid-air collision (Dispersion)..." << std::endl;

                        // Increasing the safe distance between drones to 15 meters
                        float dispersion_distance = 15.0f;

                        // Taking the leader drone's position as reference center
                        auto telemetry_leader = mavsdk::Telemetry{fleet[0]->system};
                        auto leader_pos = telemetry_leader.position();

                        // Keep the leader holding its current position
                        auto action_leader = mavsdk::Action{fleet[0]->system};
                        action_leader.goto_location(leader_pos.latitude_deg, leader_pos.longitude_deg, leader_pos.absolute_altitude_m, 0.0f);

                        // Dispersing the other drones to safe distances
                        for (size_t i = 1; i < fleet.size(); ++i) {
                            auto action = mavsdk::Action{fleet[i]->system};
                            auto telemetry = mavsdk::Telemetry{fleet[i]->system};
                            auto current_pos = telemetry.position();

                            // Simple mathematical dispersion
                            // Expanding on the Longitude by multiplying the drone index with dispersion_distance
                            // 1 degree of latitude/longitude is approxiamtely 111,320 meters at the equador
                            float offset_deg = (i * dispersion_distance) / 111320.0f;

                            float safe_lat = leader_pos.latitude_deg;
                            // Expand one drone to the east(even indices), the other to the west(odd indices)
                            float safe_lon = (i % 2 == 0) ? (leader_pos.longitude_deg + offset_deg) : (leader_pos.longitude_deg - offset_deg);

                            // Fly to the new, expanded safe coordinates
                            action.goto_location(safe_lat, safe_lon, current_pos.absolute_altitude_m, 0.0f);
                            
                            // Physically injection GPS noise
                            auto param = mavsdk::Param{fleet[i]->system};
                            param.set_param_int("SIM_GPS_NOISE", 1);


                        }

                    }
                    else if (msg == "3") {
                        std::cout << "\n[SYSTEM] Recovery signal received. Restoring GPS..." << std::endl;

                        // Returning to line-abreast formation again
                        float normal_spacing = 5.0f;

                        // Leader's current position as the anchor point
                        auto telemetry_leader = mavsdk::Telemetry{fleet[0]->system};
                        auto leader_pos = telemetry_leader.position();

                        for (size_t i = 0; i < fleet.size(); ++i) {
                            auto param = mavsdk::Param{fleet[i]->system};
                            auto action = mavsdk::Action{fleet[i]->system};

                            // Restoring GPS sensors to Normal
                            param.set_param_int("SIM_GPS_BLOCK", 0);
                            param.set_param_int("SIM_GPS_NOISE", 0);

                            // Calculate normal line-abreast formation coordinates
                            // Placing them side-by-side
                            float offset_deg = (i * normal_spacing) / 111320.0f;
                            float target_lat = leader_pos.latitude_deg;
                            float target_lon = leader_pos.longitude_deg + offset_deg;

                            // Command the UAVs to regroup
                            action.goto_location(target_lat, target_lon, leader_pos.absolute_altitude_m, 0.0f);

                        }

                        std::cout << "[SYSTEM] All sensors online. Swarm reggrouping is in progress..." << std::endl;
                    }
                } catch (...) {
                    std::cout << "[WARNING] Invalid network command received." << std::endl;
                }
            }
        }

        // TELEMETRY BROADCASTER
        void run_telemetry_broadcaster() {
        int telemetry_fd = socket(AF_INET, SOCK_DGRAM, 0);
        struct sockaddr_in gui_addr;
        memset(&gui_addr, 0, sizeof(gui_addr));
        gui_addr.sin_family = AF_INET;
        gui_addr.sin_port = htons(9091);
        gui_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        
        // Reading the ground truth file
        float target_fire_x = 0.0f;
        float target_fire_y = 0.0f;
        std::string home_dir = getenv("HOME");
        std::ifstream fire_file(home_dir + "/swarm-uav-simulation-env/worlds/fire_ground_truth.txt");
        if (fire_file.is_open()) {
            fire_file >> target_fire_x >> target_fire_y;
            fire_file.close();
            std::cout << "[SYSTEM] Ground Trurh loaded. Hidden fire location confirmed." << std::endl;
        } else {
            std::cout << "[WARNING] Could not read fire_ground_truth.txt. Check paths." << std::endl;
        }

        
        while (fleet.empty()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        
        std::vector<std::shared_ptr<mavsdk::Telemetry>> telemetry_plugins;
        for (auto& drone : fleet) {
            auto tel = std::make_shared<mavsdk::Telemetry>(drone->system);
            
            
            tel->set_rate_position(5.0);
            
            telemetry_plugins.push_back(tel);
        }

        std::cout << "[SYSTEM] Telemetry Broadcaster active. Waiting for GPS locks..." << std::endl;

        const float origin_lat = 47.397742f;
        const float origin_lon = 8.545594f;

        while (true) {
            for (size_t i = 0; i < fleet.size(); ++i) {
                
                auto pos = telemetry_plugins[i]->position();

                if (std::isnan(pos.latitude_deg) || std::isnan(pos.longitude_deg)) {
                    
                    if (i == 0) std::cout << "[DEBUG] Drone 0 is waiting for GPS lock (nan)..." << std::endl;
                    continue; 
                }

                
                float gazebo_y = (pos.latitude_deg - origin_lat) * 111320.0f; 
                float gazebo_x = (pos.longitude_deg - origin_lon) * 75440.0f;

                float fov_radius = pos.relative_altitude_m * 0.15f; // Scanning radius = 3.75 meters (It is calculated by using the altitude and field of view)
                if (fov_radius < 1.0f) fov_radius = 1.0f;

                std::string sx = std::to_string(gazebo_x);
                std::string sy = std::to_string(gazebo_y);
                std::string sfov = std::to_string(fov_radius);
                
                std::replace(sx.begin(), sx.end(), ',', '.');
                std::replace(sy.begin(), sy.end(), ',', '.');
                std::replace(sfov.begin(), sfov.end(), ',', '.');

                // The distance between two points Euclidean
                float distance_to_fire = std::hypot(gazebo_x - target_fire_x, gazebo_y - target_fire_y);

                if (distance_to_fire <= fov_radius) {

                    std::string fire_msg = "F:" +std::to_string(i) + ":" + sx + ":" + sy;
                    sendto(telemetry_fd, fire_msg.c_str(), fire_msg.length(), MSG_CONFIRM, (const struct sockaddr *) &gui_addr, sizeof(gui_addr));

                    std::cout << "\n[MISSION CRITICAL] DRONE " << i << " HAS DETECTED THE FIRE AT X: "
                              << gazebo_x << " Y: " << gazebo_y << "!" << std::endl;
                }

                std::string msg = "T:" + std::to_string(i) + ":" + sx + ":" + sy + ":" + sfov;

                sendto(telemetry_fd, msg.c_str(), msg.length(), MSG_CONFIRM, (const struct sockaddr *) &gui_addr, sizeof(gui_addr));
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }
    };

int main() {
    std::cout << "--- [SYSTEM] SWARM UAV SIMULATION IS STARTING ---" << std::endl;

    SwarmManager manager;

    // Starting the udp listener
    std::thread udp_thread(&SwarmManager::run_udp_listener, &manager);
    udp_thread.detach();

    // Starting the telemetry broadcaster
    std::thread telemetry_thread(&SwarmManager::run_telemetry_broadcaster, &manager);
    telemetry_thread.detach();    

    // PREPARATION
    manager.listenForPorts();
    manager.assembleFleet();

    // TAKEOFF
    manager.synchronizedTakeoff();
    sleep_for(seconds(5)); // Wait for stabilization

    // TRANSIT (FORMATION MODE)
    // Drones align side-by-side and fly to the fire zone for 20 seconds
    // Choosing one of the formations is optional
    //manager.mode_LineAbreastFormation(20);

    // TRIANGLE (V-SHAPE) FORMATION
    manager.mode_TriangleFormation(15);

    // MISSION
    // Upon reaching the area, they form a coordinated comb pattern to scan the forest as a unified swarm.
    manager.mode_UnifiedCombSweep();

    // Keep the program running
    std::cout << "\n[SYSTEM] Operation manager active. Press Ctrl+C to exit." << std::endl;
    while (true) {

        manager.check_swarm_collision();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    }

    return 0;
}