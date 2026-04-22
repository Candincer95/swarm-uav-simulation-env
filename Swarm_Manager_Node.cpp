#include <iostream>
#include <thread>
#include <vector>
#include <memory>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/mission/mission.h>

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
            std::cout << "[Drone " << id << "] takeoff sequence initiated... \n";
            action->arm();
            action->takeoff();
        }

};

// SWARM MANAGER CLASS

class SwarmManager {
    private:
        Mavsdk mavsdk;
        std::vector<std::shared_ptr<DroneNode>> fleet;
        bool is_formation_active = false;

    public:
        SwarmManager() : mavsdk(Mavsdk::Configuration{ComponentType::GroundStation}) {}

        void listenForPorts(){
            std::cout << "[MANAGER] Waiting for drones...\n";
            mavsdk.add_any_connection("udp://:14541");
            mavsdk.add_any_connection("udp://:14542");
            mavsdk.add_any_connection("udp://:14543");
        }

        void assembleFleet(){

            while(mavsdk.systems().size() < 3){
                sleep_for(seconds(1));
            }
            int id_counter = 1;
            for (auto sys : mavsdk.systems()){
                fleet.push_back(std::make_shared<DroneNode>(id_counter++, sys));

            }
            std::cout << "\n[MANAGER] " << fleet.size() << "drones successfully joined the fleet\n\n";

        }

        void synchronizedTakeoff() {
            std::cout << "[MANAGER] Staggered takeoff initiated..\n";
            for (auto& drone : fleet){
                drone->performTakeoff();
                sleep_for(seconds(4)); //4 seconds delay to protect the physics engine
            }
            std::cout << "[MANAGER] All drones are airborne. Awaiting mission\n\n";
        }

        // CAPABILITY 1: LINNE ABREAST FORMATION (TRANSIT)
        void mode_LineAbreastFormation(int duration_seconds){
            std::cout << "[MANAGER] MODE: Line Abreast Formation started. Transiting to target area...\n";
            is_formation_active = true;

            auto leader = fleet[0];
            auto left_wing = fleet[1];
            auto right_wing = fleet[2];

            //Start followers as Threads (Left: -1, Right: +1)
            std::thread left_thread(&SwarmManager::followerOffboardLoop, this, left_wing, leader, -1);
            std::thread right_thread(&SwarmManager::followerOffboardLoop, this, right_wing, leader, 1);

            //Initiate leader movement
            leader->offboard->set_position_ned(Offboard::PositionNedYaw{});
            leader->offboard->start();

            float leader_north_target = 0.0f;

            //Fly in formationfor the specified duration
            for(int i=0; i<(duration_seconds * 20); ++i) { //20Hz loop
                leader_north_target += 0.1f;

                Offboard::PositionNedYaw leader_target{};
                leader_target.north_m = leader_north_target;
                leader_target.east_m = 0.0f;
                leader_target.down_m = -20.0f;
                leader_target.yaw_deg = 0.0f;

                leader->offboard->set_position_ned(leader_target);
                sleep_for(milliseconds(50));
            }

            //End formation and stop leader
            is_formation_active = false;
            leader->offboard->stop();
            left_thread.join();
            right_thread.join();
            std::cout << "[MANAGER] Transit complete. Breaking formation.\n\n";

        }

        //Inner loop for followers
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

        //CAPABILITY 2: INDEPENDENT EXPLORATION (FOREST SCANNING)
        void mode_IndependentExploration() {
            std::cout << "[MANAGER] MODE: Independent Exploration (Lawnmover) started. Swarm is dispersing to the area...\n";

            // Example: Distribute strips using the leader's final position as the center
            auto center = fleet[0]->telemetry->position();

            for (size_t i=0; i < fleet.size(); ++i){
                auto mission_plan = Mission::MissionPlan{};
                double offset_lon = i * 0.0004; //Each drone moves to adjacent strip to the east

                //Simple rectangular scan route ( Lawnmover Pattern)
                std::vector<std::pair<double, double>> waypoints = {
                    {center.latitude_deg, center.longitude_deg + offset_lon},
                    {center.latitude_deg + 0.0010, center.longitude_deg + offset_lon},
                    {center.latitude_deg + 0.0010, center.longitude_deg + offset_lon + 0.0002},
                    {center.latitude_deg, center.longitude_deg + offset_lon + 0.0002}
                };

                for (auto const& [lat, lon] : waypoints) {
                    Mission::MissionItem item;
                    item.latitude_deg = lat;
                    item.longitude_deg = lon;
                    item.relative_altitude_m = 25.0f;
                    item.speed_m_s = 5.0f;
                    item.is_fly_through = true;
                    mission_plan.mission_items.push_back(item);
                }

                fleet[i]->mission->upload_mission(mission_plan);
                fleet[i]->mission->start_mission();
            }
            std::cout << "[MANAGER] Exploration routes uploaded, scanning in process...\n";
        }

        //CAPABILITY 3: TRIANGLE (V-SHAPE) FORMATION
        void mode_TriangleFormation(int duration_seconds) {
            std::cout << "[MANAGER] MODE: Triangle (V-Shape) Formation initiated...\n";
            is_formation_active = true;

            auto leader = fleet[0];
            auto left_follower = fleet[1];
            auto right_follower = fleet[2];

            //Offsets: 5m behind, 5m to the sides
            float back_offset = -5.0f;
            float side_offset = 5.0f;

            //Use Lambda for Threads (left: -5m East, Right: +5m East)
            std::thread left_thread([this, left_follower, leader, back_offset, side_offset]() {
                this->followerTriangleLoop(left_follower, leader, back_offset, -side_offset, 2);
        
            });

            std::thread right_thread([this, right_follower, leader, back_offset, side_offset](){
                this->followerTriangleLoop(right_follower, leader, back_offset, side_offset, 3);
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

        void followerTriangleLoop(std::shared_ptr<DroneNode> follower, std::shared_ptr<DroneNode> leader, float n_offset, float e_offset, int id) {
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
        
};

int main() {
    SwarmManager manager;

    // 1-) PREPARATION
    manager.listenForPorts();
    manager.assembleFleet();

    // 2-) TAKEOFF
    manager.synchronizedTakeoff();
    sleep_for(seconds(5)); // Wait for stabilization

    // 3-) TRANSIT (FORMATION MODE)
    //Drones align side-by-side and fly to the fire zone for 20 seconds
    manager.mode_LineAbreastFormation(20);

    // 4-) TRIANGLE (V-SHAPE) FORMATION
    manager.mode_TriangleFormation(15);

    // 5-) MISSION (EXPLORATION MODE)
    //Upon reaching the area, formation breaks and they disperse into strips to scan the forest
    manager.mode_IndependentExploration();

    // Keep the program running
    std::cout << "\nOperation manager active. Press Ctrl+C to exit";
    while (true) {
        sleep_for(seconds(10));
    }

    return 0;

}

