#include <iostream>
#include <thread>
#include <vector>
#include <memory>
#include <chrono>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fstream>
#include <iomanip>
#include <atomic>
#include <mutex>

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

// Calculates how many meters 1 degree of longitude in meters based on latitude
float get_longitude_scale(float lat_deg) {
    const float earth_radius_m = 6378137.0f;
    return earth_radius_m * std::cos(lat_deg * M_PI / 180.0f) * (M_PI / 180.0f);
}

// INDIVIDUAL DRONE CLASS
class DroneNode {
    public:
        int id;
        std::shared_ptr<System> system;
        std::unique_ptr<Action> action;
        std::unique_ptr<Telemetry> telemetry;
        std::unique_ptr<Offboard> offboard;
        std::unique_ptr<Mission> mission;
        std::unique_ptr<Param> param;

        DroneNode(int drone_id, std::shared_ptr<System> sys) : id(drone_id), system(sys) {
            action = std::make_unique<Action>(system);
            telemetry = std::make_unique<Telemetry>(system);
            offboard = std::make_unique<Offboard>(system);
            mission = std::make_unique<Mission>(system);
        }

        void performTakeoff() {
            std::cout << "[Drone " << id << "] takeoff sequence initiated..." << std::endl;
            
            mavsdk::Action::Result arm_result = action->arm();
            if (arm_result != mavsdk::Action::Result::Success) {
                std::cerr << "[ERROR] Drone " << id << " failed to arm: " << arm_result << std::endl;
                return;
            }

            mavsdk::Action::Result takeoff_result = action->takeoff();
            if (takeoff_result != mavsdk::Action::Result::Success) {
                std::cerr << "[ERROR] Drone " << id << " failed to takeoff: " << takeoff_result << std::endl;
            }
        }
};

// SWARM MANAGER CLASS
class SwarmManager {
    private:
        Mavsdk mavsdk;
        std::vector<std::shared_ptr<DroneNode>> fleet;
        size_t expected_drones; // For expected number of drones
        
        // THREAD SAFETY: Atomics and Mutex for shared resources
        std::mutex swarm_mutex;
        std::atomic<bool> is_formation_active{false};
        std::atomic<bool> is_mission_phase{false};
        std::atomic<bool> fire_alarm_triggered{false};

        // METRIC VARIABLES
        std::chrono::time_point<std::chrono::steady_clock> mission_start_time;
        std::chrono::time_point<std::chrono::steady_clock> gps_recovery_start_time;

        // FORMATION ERROR VARIABLES (RMSE)
        double sum_squared_error = 0.0f;
        int formation_error_samples = 0;
        double gps_recovery_duration = 0.0; // Time to GPS recovery

    public:
        // Constructor retrieves the number of drones
        SwarmManager(size_t drone_count) : mavsdk(Mavsdk::Configuration{ComponentType::GroundStation}), expected_drones(drone_count) {}

        void listenForPorts(){
            std::cout << "[MANAGER] Waiting for " << expected_drones << " drones..." << std::endl;
            
            for (size_t i = 1; i <= expected_drones; ++i) {
                mavsdk.add_any_connection("udpin://0.0.0.0:" + std::to_string(14540 + i));
            }
        }

        void assembleFleet(){
            while(mavsdk.systems().size() < expected_drones){
                sleep_for(seconds(1));
            }

            std::lock_guard<std::mutex> lock(swarm_mutex);
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
            // If there aren't enough drones, skip forming the formation
            if (fleet.size() < 3) {
                std::cout << "[MANAGER] Skipping Line Abreast Formation (requires at least 3 drones)." << std::endl;
                return;
            }
            std::cout << "[MANAGER] MODE: Line Abreast Formation started. Transiting to target area..." << std::endl;
            is_formation_active = true;

            auto leader = fleet[0];
            auto left_wing = fleet[1];
            auto right_wing = fleet[2];

            // Start followers as Threads (Left: -1, Right: +1)
            std::thread left_thread(&SwarmManager::followerOffboardLoop, this, left_wing, leader, -1);
            std::thread right_thread(&SwarmManager::followerOffboardLoop, this, right_wing, leader, 1);

            auto leader_start_pos = leader->telemetry->position_velocity_ned().position;

            Offboard::PositionNedYaw initial_target{};
            initial_target.north_m = leader_start_pos.north_m;
            initial_target.east_m = leader_start_pos.east_m;
            initial_target.down_m = -25.0f;
            
            for (int i = 0; i < 20; ++i) {
                leader->offboard->set_position_ned(initial_target);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            
            mavsdk::Offboard::Result offboard_result = leader->offboard->start();
            if (offboard_result != mavsdk::Offboard::Result::Success) {
                std::cerr << "[ERROR] Leader rejected Offboard mode: " << offboard_result << std::endl;
            } 

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
            auto my_start_pos = follower->telemetry->position_velocity_ned().position;

            Offboard::PositionNedYaw initial_target{};
            initial_target.north_m = my_start_pos.north_m;
            initial_target.east_m = my_start_pos.east_m;
            initial_target.down_m = -25.0f;

            for (int i = 0; i < 20; ++i) {
                follower->offboard->set_position_ned(initial_target);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            follower->offboard->start();

            while (is_formation_active) {
                auto leader_pos = leader->telemetry->position_velocity_ned().position;

                Offboard::PositionNedYaw follower_target{};
                follower_target.north_m = leader_pos.north_m;
                follower_target.east_m = leader_pos.east_m + (5.0f * direction_multiplier);
                follower_target.down_m = -25.0f;
                follower_target.yaw_deg = 0.0f;

                follower->offboard->set_position_ned(follower_target);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            follower->offboard->stop();
        }

        // COMB PATTERN SCANNING
        // FINAL SOLUTION: MANAGER-CONTROLLED VELOCITY AND SYNCHRONIZATION BARRIER SCAN
        void mode_UnifiedCombSweep() {
            std::cout << "[MANAGER] MODE: Synchronized Unified Swarm Sweep (Velocity Control) started..." << std::endl;
            
            // Gazebo Standard Origin Point
            const float origin_lat = 47.397742f;
            const float origin_lon = 8.545594;
            const float speed = 5.0f; // Flying speed (5 m/s)
            
            // Forest Borders
            const float x_min = -80.0f;
            const float x_max = 80.0f;
            const float y_min = 10.0f;
            const float y_max = 150.0f;
            const float strip_spacing = 6.0f;
            float lon_scale = get_longitude_scale(origin_lat);

            int num_drones = fleet.size();
            if (num_drones == 0) return;
            float swarm_step = num_drones * strip_spacing;

            // Phase 1: Stabilize
            std::cout << "[MANAGER] Stabilizing swarm..." << std::endl;
            for (auto& drone : fleet) drone->action->hold();
            std::this_thread::sleep_for(std::chrono::seconds(2)); 

            // Phase 2: Staggered Rendezvous (Entry to Assembly Point) 
            std::cout << "[MANAGER] Phase 2: Transit to entry points at staggered altitudes..." << std::endl;
            std::vector<float> transit_altitudes = {25.0f, 30.0f, 20.0f};
            std::vector<std::pair<float, float>> target_coords(num_drones);
            for (int i = 0; i < num_drones; ++i) {
                int lane_index = (num_drones == 1) ? 0 : ((i == 1) ? 0 : ((i == 0) ? 1 : 2));
                float drone_x = x_min + (lane_index * strip_spacing);
                target_coords[i] = {origin_lat + (y_min / 111320.0f), origin_lon + (drone_x / lon_scale)};
                auto pos = fleet[i]->telemetry->position();
                float ground_alt_amsl = pos.absolute_altitude_m - pos.relative_altitude_m;
                fleet[i]->action->goto_location(target_coords[i].first, target_coords[i].second, ground_alt_amsl + transit_altitudes[i], 0.0f);
            }
            // Wait loop with NaN protection
            std::cout << "[MANAGER] Actively verifying swarm arrival..." << std::endl;
            bool all_arrived = false;
            while (!all_arrived) {
                all_arrived = true;
                for (int i = 0; i < num_drones; ++i) {
                    auto pos = fleet[i]->telemetry->position();
                    float dy = (pos.latitude_deg - target_coords[i].first) * 111320.0f;
                    float dx = (pos.longitude_deg - target_coords[i].second) * lon_scale;
                    float dist = std::sqrt(dx*dx + dy*dy);
                    if (std::isnan(dist) || dist > 2.0f) { all_arrived = false; break; }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

            // Phase 3: Descend and Prepare
            std::cout << "[MANAGER] Descending to unified 25m and switching to Offboard..." << std::endl;
            for (int i = 0; i < num_drones; ++i) {
                auto pos = fleet[i]->telemetry->position();
                float ground_alt_amsl = pos.absolute_altitude_m - pos.relative_altitude_m;
                fleet[i]->action->goto_location(pos.latitude_deg, pos.longitude_deg, ground_alt_amsl + 25.0f, 0.0f);
            }
            std::this_thread::sleep_for(std::chrono::seconds(7));

            // MANAGER-CONTROLLED VELOCITY SCAN
            std::cout << "[MANAGER] Phase 4: Starting Manager-Controlled Velocity Scan..." << std::endl;
            is_formation_active = true; // Inform ATC that it should perform an Offboard recovery
            is_mission_phase =true; // Start the error calculator
            
            // Put drones into Offboard velocity mode
            for (auto& drone : fleet) {
                Offboard::VelocityNedYaw target_velocity{};
                target_velocity.yaw_deg = 0.0f; // Pivot to face North (Y+)
                drone->offboard->set_velocity_ned(target_velocity);
                drone->offboard->start();
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));

            float current_base_x = x_min;
            bool flying_up = true; // Are we heading North (Y+)?

            // Lane Loop (Scanning on the X axis)
            while (current_base_x <= x_max) {
                // Timing Synchronization
                float current_n_velocity = flying_up ? speed : -speed;
                float turn_line_y = flying_up ? y_max : y_min;

                std::cout << "[MANAGER] Pass " << (flying_up ? "North" : "South") << " on X: " << current_base_x << "..." << std::endl;
                
                // STRAIGHT FLIGHT VELOCITY
                for (auto& drone : fleet) {
                    Offboard::VelocityNedYaw target_velocity{};
                    target_velocity.north_m_s = current_n_velocity; // Applying only North/South velocity
                    target_velocity.east_m_s = 0.0f; // Straight North-South line
                    target_velocity.down_m_s = 0.0f;
                    target_velocity.yaw_deg = 0.0f;
                    drone->offboard->set_velocity_ned(target_velocity);
                }

                // ACTIVE VERIFICATION BARRIER
                bool all_passed_line = false;
                // Stopping distance 
                float braking_distance = 2.0f; 

                while (!all_passed_line) {
                    all_passed_line = true;
                    for (int i = 0; i < num_drones; ++i) {
                        auto pos = fleet[i]->telemetry->position();
                        float gazebo_y = (pos.latitude_deg - origin_lat) * 111320.0f; 
                        
                        // Did it cross the Y boundary?
                        if (flying_up) {
                            // Heading North (up): Trigger stop if approaching the boundary (y_max) within braking distance
                            if (gazebo_y < (turn_line_y - braking_distance)) {
                                all_passed_line = false; // Wait if someone is still behind
                                break;
                            }
                        } else {
                            // Heading South (down): Trigger stop if approaching the boundary (y_min) within braking distance
                            if (gazebo_y > (turn_line_y + braking_distance)) {
                                all_passed_line = false;
                                break;
                            }
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10Hz Telemetry listening
                }

                // TURN AND SYNCHRONIZATION
                std::cout << "[MANAGER] Swarm reached virtual break zone " << turn_line_y << "m. Stopping for synchronization barrier..." << std::endl;
                for (auto& drone : fleet) {
                    Offboard::VelocityNedYaw hover_velocity{}; // North=0, East=0
                    drone->offboard->set_velocity_ned(hover_velocity);
                }
                
                // Point Synchronization Verification (Verify from telemetry that each stopped at its target point)
                // This prevents them from drifting in the wind and triggering ATC.
                std::this_thread::sleep_for(std::chrono::seconds(2)); // Short margin for PID controllers to settle

                if (current_base_x + swarm_step <= x_max) {
                    // Horizontal Lane Shift (Pivot and Slide)
                    std::cout << "[MANAGER] Shifting swarm 18 meters East..." << std::endl;

                    // Target X Center after 18 meters
                    float target_base_x = current_base_x + swarm_step;

                    // North=0, East=speed
                    for (auto& drone : fleet) {
                        Offboard::VelocityNedYaw shift_velocity{};
                        shift_velocity.east_m_s = 2.0f; // Shift only East
                        drone->offboard->set_velocity_ned(shift_velocity);
                    }

                    // Coordinate-based lane shift instead of Time-based
                    bool all_shifted = false;
                    auto shift_start_time = std::chrono::steady_clock::now(); // Get start time
                    while (!all_shifted) {
                        all_shifted = true;
                        for (int i = 0; i < num_drones; ++i) {
                            int lane_index = (num_drones == 1) ? 0 : ((i == 1) ? 0 : ((i == 0) ? 1 : 2));
                            float target_drone_x = target_base_x + (lane_index * strip_spacing);

                            auto pos = fleet[i]->telemetry->position();
                            float gazebo_x = (pos.longitude_deg - origin_lon) * lon_scale;

                            // Keep waiting if the drone hasn't reached the target X coordinate (with 0.5m tolerance)
                            if (gazebo_x < target_drone_x - 0.5f) {
                                all_shifted = false;
                                break;
                            }
                        }

                        auto current_time = std::chrono::steady_clock::now();
                        if (std::chrono::duration_cast<std::chrono::seconds>(current_time - shift_start_time).count() > 10) {
                            std::cout << "[WARNING] Wind interference detected! Shift timeout reached. Forcing next phase..." << std::endl;
                            break;
                        }

                        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10Hz control loop
                    }

                    // Reached exact horizontal target, stop now
                    for (auto& drone : fleet) {
                        Offboard::VelocityNedYaw hover_velocity{};
                        drone->offboard->set_velocity_ned(hover_velocity);
                    }
                    std::this_thread::sleep_for(std::chrono::seconds(1)); // Pivot stabilization
                }

                if (current_base_x + swarm_step > x_max + 5.0f) {
                    break;
                }

                current_base_x += swarm_step;
                flying_up = !flying_up; // Change direction
            }

            // Phase 5: Mission complete
            std::cout << "[MANAGER] Forest sweep complete. Transitioning to Hold." << std::endl;
            is_formation_active = false;
            for (auto& drone : fleet) {
                drone->offboard->stop();
                drone->action->hold();
            }
        }

        // COLLISION AVOIDANCE
        void check_swarm_collision() {
            const float safety_radius = 3.0f; // 3.0 meters safe distance
            const float evasion_alt = 45.0f; // A safe upper altitude to escape to in case of collision
            int num_drones = fleet.size();

            // Loop that compares each drone to all other drones 
            for (int i = 0; i < num_drones; ++i) {
                for (int j = i + 1; j < num_drones; ++j) {

                    // Real-time GPS and altitude data
                    auto pos1 = fleet[i]->telemetry->position();
                    auto pos2 = fleet[j]->telemetry->position();

                    // Converting GPS degrees to meters
                    float lon_scale = get_longitude_scale(pos1.latitude_deg);
                    float dx = (pos2.longitude_deg - pos1.longitude_deg) * lon_scale;
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
                        
                        // Temporarily suspend commands from the control center to prevent the autopilot from locking
                        fleet[i]->action->hold();
                        fleet[j]->action->hold();

                        std::this_thread::sleep_for(std::chrono::milliseconds(200));

                        // Align the endangered Drone J to a safe altitude
                        fleet[j]->action->goto_location(pos2.latitude_deg, pos2.longitude_deg, evasion_alt, 0);

                        // Wait for the drone to climb 45 meters and for the danger to pass (Breaks the infinite loop)
                        std::this_thread::sleep_for(std::chrono::seconds(5));

                        if (is_mission_phase) {
                            // If there was a risk of collision while scanning the forest, continue the mission
                            fleet[i]->mission->start_mission();
                            fleet[j]->mission->start_mission();
                        } else if (is_formation_active) {
                            // If there was a risk of collision in a V-formation while on the road, restart offboard mode
                            fleet[i]->offboard->start();
                            fleet[j]->offboard->start();
                        }
                    }
                }
            }
        }


        // TRIANGLE (V-SHAPE) FORMATION
        void mode_TriangleFormation(int duration_seconds) {

            if (fleet.size() < 3) {
                std::cout << "[MANAGER] Skipping Triangle Formation (requires at least 3 drones)." << std::endl;
                return;
            }
            std::cout << "[MANAGER] MODE: Triangle (V-Shape) Formation initiated..." << std::endl;
            // Start the chronometer
            mission_start_time = std::chrono::steady_clock::now();
            std::cout << "[METRIC] Mission timer started!" << std::endl;

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

            auto leader_start_pos = leader->telemetry->position_velocity_ned().position;

            Offboard::PositionNedYaw initial_target{};
            initial_target.north_m = leader_start_pos.north_m;
            initial_target.east_m = leader_start_pos.east_m;
            initial_target.down_m = -25.0f;

            for (int i = 0; i < 20; ++i) {
                leader->offboard->set_position_ned(initial_target);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            mavsdk::Offboard::Result offboard_result = leader->offboard->start();
            if (offboard_result != mavsdk::Offboard::Result::Success) {
                std::cout << "[ERROR] Leader rejected Offboard mode: " << offboard_result << std::endl;
            }

            float leader_n = 0.0f;
            for (int i = 0; i < (duration_seconds * 20); ++i) {
                leader_n += 0.2f;
                Offboard::PositionNedYaw target{};
                target.north_m = leader_n;
                target.down_m = -25.0f;
                leader->offboard->set_position_ned(target);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            std::cout << "[MANAGER] Transitioning to Line Abreast for forest entry..." << std::endl;

            float transition_n_offset = 0.0f;
            float transition_e_offset = 6.0f;

            for (int i = 0; i < 50; ++i) {
                Offboard::PositionNedYaw target_left{};
                target_left.north_m = leader_n + transition_n_offset;
                target_left.east_m = -transition_e_offset;
                target_left.down_m = -25.0f;
                left_follower->offboard->set_position_ned(target_left);

                Offboard::PositionNedYaw target_right{};
                target_right.north_m = leader_n + transition_n_offset;
                target_right.east_m = transition_e_offset;
                target_right.down_m = -25.0f;
                right_follower->offboard->set_position_ned(target_right);

                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            is_formation_active = false;
            leader->offboard->stop();

            left_thread.join();
            right_thread.join();
            std::cout << "[MANAGER] Triangle formation complete. Transitioning to Sweep." << std::endl;

        }

        void followerTriangleLoop(std::shared_ptr<DroneNode> follower, std::shared_ptr<DroneNode> leader, float n_offset, float e_offset) {

            auto my_start_pos = follower->telemetry->position_velocity_ned().position;

            Offboard::PositionNedYaw initial_target{};
            initial_target.north_m = my_start_pos.north_m;
            initial_target.east_m = my_start_pos.east_m;
            initial_target.down_m = -25.0f;

            for (int i = 0; i < 20; ++i) {
                follower->offboard->set_position_ned(initial_target);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            follower->offboard->start();

            while (is_formation_active) {
                auto leader_pos = leader->telemetry->position_velocity_ned().position;
                Offboard::PositionNedYaw target{};
                target.north_m = leader_pos.north_m + n_offset;
                target.east_m = leader_pos.east_m + e_offset;
                target.down_m = -25.0f;
                target.yaw_deg = 0.0f;
                follower->offboard->set_position_ned(target);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            follower->offboard->stop();
        }
        // BACKGROUND UDP LISTENER
        void run_udp_listener() {
            int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
            if (sockfd < 0) {
                std::cerr << "[ERROR] Sensor Fault UDP Listener socket creation failed!" << std::endl;
                return;
            }

            struct sockaddr_in servaddr;
            memset(&servaddr, 0, sizeof(servaddr));
            servaddr.sin_family = AF_INET;
            servaddr.sin_addr.s_addr = INADDR_ANY;
            servaddr.sin_port = htons(9090);
            
            if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
                std::cerr << "[ERROR] Sensor Fault UDP Listener bind failed!" << std::endl;
                close(sockfd);
                return;
            }

            char buffer[1024];
            std::cout << "[SYSTEM] External Sensor Fault Injection Listener active on UDP Port 9090" <<std::endl;

            while(true) {
                // Buffer overflow protection: limiting recvfrom size and checking return value
                int n = recvfrom(sockfd, (char *)buffer, sizeof(buffer) - 1, MSG_WAITALL, NULL, NULL);
                if (n >= 0) {
                    buffer[n] = '\0';
                    std::string msg(buffer);

                    try{
                        // "1" -> Block, "2" -> Spoof, "3" -> Recover
                        if (msg == "1") {
                            std::cout << "\n[SENSOR FAULT] WARNING: GPS Signal Loss (Denial) injected across the swarm!" << std::endl;
                            std::cout << "[DEFENSE] Autonomous Recovery Activated: All UAVs climbing to +30m safe altitude..." << std::endl;

                            // Start the chronometer
                            gps_recovery_start_time = std::chrono::steady_clock::now();

                            std::lock_guard<std::mutex> lock(swarm_mutex);
                            for (auto& drone : fleet) {
                                auto current_pos = drone->telemetry->position();
                                float safe_alt_absolute = current_pos.absolute_altitude_m + 30.0f;

                                // Send the autonomous recovery command
                                drone->action->goto_location(current_pos.latitude_deg, current_pos.longitude_deg, safe_alt_absolute, 0.0f);
                                // Block the GPS sensor inside PX4
                                drone->param->set_param_int("SIM_GPS_BLOCK", 1);
                                
                            }
                        }
                        else if (msg == "2") {
                            std::cout << "\n[SENSOR FAULT] GPS Noise (Spoofing) injection received." << std::endl;
                            std::cout << "[DEFENSE] Autonomous Recovery: Expanding formation to prevent mid-air collision (Dispersion)..." << std::endl;

                            std::lock_guard<std::mutex> lock(swarm_mutex);
                            float dispersion_distance = 15.0f;

                            // Taking the leader drone's position as reference center
                            auto leader_pos = fleet[0]->telemetry->position();

                            // Keep the leader holding its current position
                            fleet[0]->action->goto_location(leader_pos.latitude_deg, leader_pos.longitude_deg, leader_pos.absolute_altitude_m, 0.0f);

                            // Dispersing the other drones to safe distances
                            for (size_t i = 1; i < fleet.size(); ++i) {
                                auto current_pos = fleet[i]->telemetry->position();
                                
                                // 1 degree of latitude/longitude is approximately 111320 meters at the Equator
                                float offset_deg = (i * dispersion_distance) / 111320.0f;
                                float safe_lat = leader_pos.latitude_deg;
                                // Expand one drone to the east(even indices), the other to the west(odd indices)
                                float safe_lon = (i % 2 == 0) ? (leader_pos.longitude_deg + offset_deg) : (leader_pos.longitude_deg - offset_deg);

                                // Fly to the new, expanded safe coordinates
                                fleet[i]->action->goto_location(safe_lat, safe_lon, current_pos.absolute_altitude_m, 0.0f);
                                // Physically injection GPS noise
                                fleet[i]->param->set_param_int("SIM_GPS_NOISE", 1);
                                
                            }
                        }
                        else if (msg == "3") {
                            std::cout << "\n[SYSTEM] Recovery signal received. Restoring GPS..." << std::endl;

                            // Stop the chronometer
                            auto recovery_end = std::chrono::steady_clock::now();
                            gps_recovery_duration = std::chrono::duration<double>(recovery_end - gps_recovery_start_time).count();
                            
                            std::cout << "[METRIC] GPS Denial/Recovery Event Duration: "
                                      << std::fixed << std::setprecision(2) << gps_recovery_duration << " seconds." << std::endl;
                            
                            std::lock_guard<std::mutex> lock(swarm_mutex);
                            float normal_spacing = 5.0f;

                            auto leader_pos = fleet[0]->telemetry->position();

                            for (size_t i = 0; i < fleet.size(); ++i) {
                                //Restoring GPS Signals
                                fleet[i]->param->set_param_int("SIM_GPS_BLOCK", 0);
                                fleet[i]->param->set_param_int("SIM_GPS_NOISE", 0);

                                float offset_deg = (i * normal_spacing) / 111320.0f;
                                float target_lat = leader_pos.latitude_deg;
                                float target_lon = leader_pos.longitude_deg + offset_deg;

                                // Command the UAVs to regroup
                                fleet[i]->action->goto_location(target_lat, target_lon, leader_pos.absolute_altitude_m, 0.0f);
                            }
                            std::cout << "[SYSTEM] All sensors online. Swarm regrouping is in progress..." << std::endl;
                        }
                    } catch (...) {
                        std::cout << "[WARNING] Invalid network command received." << std::endl;
                    }
                }
            }
        }

        // TELEMETRY BROADCASTER
        void run_telemetry_broadcaster() {
            int telemetry_fd = socket(AF_INET, SOCK_DGRAM, 0);
            if (telemetry_fd < 0) {
                std::cerr << "[ERROR] Telemetry socket creation failed!" << std::endl;
                return;
            }

            struct sockaddr_in gui_addr;
            memset(&gui_addr, 0, sizeof(gui_addr));
            gui_addr.sin_family = AF_INET;
            gui_addr.sin_port = htons(9091);
            gui_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
            
            // Dynamic root path
            const char* root_env = getenv("SWARM_UAV_ROOT");
            std::string root_dir = root_env ? std::string(root_env) : (std::string(getenv("HOME")) + "/swarm-uav-simulation-env");
            std::ifstream fire_file(root_dir + "/worlds/fire_ground_truth.txt");
            
            // Reading the ground truth file
            float target_fire_x = 0.0f;
            float target_fire_y = 0.0f;

            if (fire_file.is_open()) {
                fire_file >> target_fire_x >> target_fire_y;
                fire_file.close();
                std::cout << "[SYSTEM] Ground Truth loaded. Hidden fire location confirmed." << std::endl;
            } else {
                std::cerr << "[WARNING] Could not read fire_ground_truth.txt at " << root_dir << ". Check paths." << std::endl;
            }

            // Thread-safe Polling
            bool is_fleet_ready = false;
            while (!is_fleet_ready) {
                {
                    std::lock_guard<std::mutex> lock(swarm_mutex); // Lock only in control
                    is_fleet_ready = !fleet.empty(); // EKSİK OLAN KRİTİK SATIR
                }

                if (!is_fleet_ready) {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
            {
                std::lock_guard<std::mutex> lock(swarm_mutex);
                for (auto& drone : fleet) {
                    drone->telemetry->set_rate_position(5.0);
                }
            }
            
            std::cout << "[SYSTEM] Telemetry Broadcaster active. Waiting for GPS locks..." << std::endl;

            const float origin_lat = 47.397742f;
            const float origin_lon = 8.545594f;
            float lon_scale = get_longitude_scale(origin_lat); // Origin based dynamic calculation

            while (true) {
                std::lock_guard<std::mutex> lock(swarm_mutex); // Protect fleet iteration

                // METRIC: INSTANTANEOUS FORMATION ERROR (RMSE) CALCULATOR
                if (is_mission_phase && fleet.size() >= 3) {
                    auto p0 = fleet[0]->telemetry->position();
                    auto p1 = fleet[1]->telemetry->position();
                    auto p2 = fleet[2]->telemetry->position();

                    // For valid GPS data
                    if (!std::isnan(p0.latitude_deg) && !std::isnan(p1.latitude_deg) && !std::isnan(p2.latitude_deg)) {
                        float scale = get_longitude_scale(p0.latitude_deg);
                        const float target_spacing = 6.0f; //strip_spacing

                        // Error between Drone 0 (center) and Drone 1 (left) 
                        float dx1 = (p1.longitude_deg - p0.longitude_deg) * scale;
                        float dy1 = (p1.latitude_deg - p0.latitude_deg) * 111320.0f;
                        float dist1 = std::sqrt(dx1*dx1 + dy1*dy1);
                        float error1 = dist1 - target_spacing;

                        // Error between Drone 0 (center) and Drone 2 (right)
                        float dx2 = (p2.longitude_deg - p0.longitude_deg) * scale;
                        float dy2 = (p2.latitude_deg - p0.latitude_deg) * 111320.0f;
                        float dist2 = std::sqrt(dx2*dx2 + dy2*dy2);
                        float error2 = dist2 - target_spacing;

                        // Square the errors and add them to the total value (RMSE formula)
                        sum_squared_error += (error1 * error1 + error2 * error2) / 2.0;
                        formation_error_samples++;
                    }
                }

                for (size_t i = 0; i < fleet.size(); ++i) {
                    
                    auto pos = fleet[i]->telemetry->position();

                    if (std::isnan(pos.latitude_deg) || std::isnan(pos.longitude_deg)) {
                        if (i == 0) std::cout << "[DEBUG] Drone 0 is waiting for GPS lock (nan)..." << std::endl;
                        continue; 
                    }

                    float gazebo_y = (pos.latitude_deg - origin_lat) * 111320.0f; 
                    float gazebo_x = (pos.longitude_deg - origin_lon) * lon_scale;

                    float fov_radius = pos.relative_altitude_m * 0.15f; 
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
                        // FIRE DETECTED: Trigger atomic flag and HALT the mission
                        if (!fire_alarm_triggered.exchange(true)) {
                            // Stop the chronometer and calculate
                            auto fire_detected_time = std::chrono::steady_clock::now();
                            double time_to_fire = std::chrono::duration<double>(fire_detected_time - mission_start_time).count();

                            std::string fire_msg = "F:" +std::to_string(i) + ":" + sx + ":" + sy;
                            sendto(telemetry_fd, fire_msg.c_str(), fire_msg.length(), MSG_CONFIRM, (const struct sockaddr *) &gui_addr, sizeof(gui_addr));

                            std::cout << "\n[MISSION CRITICAL] DRONE " << i << " HAS DETECTED THE FIRE AT X: "
                                      << gazebo_x << " Y: " << gazebo_y << "!" << std::endl;
                            double rmse = 0.0;
                            if (formation_error_samples > 0) {
                                rmse = std::sqrt(sum_squared_error / formation_error_samples);
                            }

                            // EXPERIMENTAL METRIC OUTPUT
                            std::cout << "\n================ EXPERIMENTAL METRICS ================" << std::endl;
                            std::cout << "[METRIC] Fire Detection Time: " << std::fixed << std::setprecision(2) << time_to_fire << " seconds." << std::endl;
                            std::cout << "[METRIC] Formation Error (RMSE): " << std::fixed << std::setprecision(3) << rmse << " meters." << std::endl;
                            std::cout << "======================================================\n" << std::endl;

                            // Root direction of the project
                            const char* root_env = getenv("SWARM_UAV_ROOT");
                            std::string root_dir = root_env ? std::string(root_env) : (std::string(getenv("HOME")) + "/swarm-uav-simulation-env");
                            std::string csv_file_path = root_dir + "/test_results.csv";

                            // Read the environmental variables coming from the Automation script
                            const char* env_run_id = getenv("TEST_RUN_ID");
                            const char* env_seed = getenv("TEST_SEED");
                            const char* env_wind = getenv("TEST_WIND");

                            std::string run_id = env_run_id ? std::string(env_run_id) : "Manual";
                            std::string seed_val = env_seed ? std::string(env_seed) : "N/A";
                            std::string wind_val = env_wind ? std::string(env_wind) : "0";

                            // Check if the file already exists (to only enter the headers the first time)
                            bool file_exists = false;
                            std::ifstream fcheck(csv_file_path);
                            if (fcheck.good()) {
                                file_exists = true;
                            }
                            fcheck.close();

                            // Open the file in "Append" mode
                            std::ofstream csv_file(csv_file_path, std::ios::app);
                            if (csv_file.is_open()) {
                                if (!file_exists) {
                                    // If the file is being created for the first time, enter the CSV Headers
                                    csv_file << "Run_ID,Seed,Wind_Percent,Drones,Time_to_Fire_sec,RMSE_meters,GPS_Recovery_sec\n";
                                }
                                csv_file << run_id << ","
                                         << seed_val << ","
                                         << wind_val << ","
                                         << fleet.size() << ","
                                         << std::fixed << std::setprecision(2) << time_to_fire << ","
                                         << std::fixed << std::setprecision(3) << rmse << ",";
                                         << std::fixed << std::setprecision(2) << gps_recovery_duration << "\n";
                                csv_file.close();
                                std::cout << "[SYSTEM] Metrics successfully saved to " << csv_file_path << std::endl;
                            } else {
                                std::cerr << "[ERROR] Could not open CSV file for writing at: " << csv_file_path << std::endl;
                            }

                            std::cout << "[SYSTEM] Halting swarm mission to investigate..." << std::endl;
                            
                            for (auto& d : fleet) {
                                d->mission->pause_mission();
                            }
                        }
                    }

                    std::string msg = "T:" + std::to_string(i) + ":" + sx + ":" + sy + ":" + sfov;
                    int send_res = sendto(telemetry_fd, msg.c_str(), msg.length(), MSG_CONFIRM, (const struct sockaddr *) &gui_addr, sizeof(gui_addr));
                    if (send_res < 0) {
                        std::cerr << "[WARNING] Failed to broadcast telemetry for Drone " << i << std::endl;
                    }
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        }
};

int main() {
    std::cout << "--- [SYSTEM] SWARM UAV SIMULATION IS STARTING ---" << std::endl;

    size_t TARGET_DRONES = 3;
    SwarmManager manager(TARGET_DRONES);

    // Starting the udp listener
    std::thread udp_thread(&SwarmManager::run_udp_listener, &manager);
    udp_thread.detach();

    // Starting the telemetry broadcaster
    std::thread telemetry_thread(&SwarmManager::run_telemetry_broadcaster, &manager);
    telemetry_thread.detach();  
    
    // Collision control thread
    std::thread atc_thread([&manager]() {
        while (true) {
            manager.check_swarm_collision();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });
    atc_thread.detach();

    // PREPARATION
    manager.listenForPorts();
    manager.assembleFleet();

    // WAIT FOR EKF HEADING ALIGNMENTS
    std::cout << "[SYSTEM] Waiting for EKF heading alignment and GPS locks..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(15));

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
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}