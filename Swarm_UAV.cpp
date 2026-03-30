#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

//Flight function for every drone

void flight(std::shared_ptr<System> system, int drone_id){
    auto telemetry = Telemetry{system};
    auto action = Action{system};

    std::cout << "[Drone " << drone_id << "] Sensor and GPS data are expected\n";
    while (!telemetry.health().is_global_position_ok){
        sleep_for(seconds(1));
    }

    std::cout << "[Drone " << drone_id << "] GPS is locked, Ready to flight\n";

    // To prevent the physics engine from crashing, launches are being queued
    std::cout << "[Drone " << drone_id << "] Waiting for flight order\n";
    sleep_for(seconds(drone_id * 1)); // Drone 1 -> 1 seconds, Drone 2 -> 2 seconds, Drone 3 -> 3 seconds

    //Arm
    if (action.arm() != Action::Result::Success){
        std::cerr << "[Drone " << drone_id << "] Engines cannot be started\n";
        return;
    }

    std::cout << "[Drone  " << drone_id << "] Engines are armed\n ";

    //Takeoff
    if (action.takeoff() != Action::Result::Success){
        std::cerr <<"[Drone " << drone_id << "] Takeoff failed\n";
        return;
    
    }
    std::cout << "[Drone " << drone_id << "] Taking Off\n";

    //Waiting in the air
    sleep_for(seconds(10));

    //Landing
    std::cout << "[Drone " << drone_id << "] Landing command given\n";
    action.land();

}

int main() {
    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};

    std::cout << "Listening the swarm UAV ports...\n";

    //Adding the drones to the system
    mavsdk.add_any_connection("udp://:14541");
    mavsdk.add_any_connection("udp://:14542");
    mavsdk.add_any_connection("udp://:14543");

    //Waiting for all drones to be integrated into the system
    while (mavsdk.systems().size() < 3){
        std::cout << "The number of drones connected: " << mavsdk.systems().size() << "/3\n";
        sleep_for(seconds(2));

    }
    std::cout << "\n--- The drones connected to the sytem successfully ---\n\n";

    auto systems = mavsdk.systems();
    std::vector<std::thread> threads;

    //Start a seperate thread for each drone
    int id = 1;
    for (auto system : systems){
        threads.push_back(std::thread(flight, system, id++));
    }

    //Waiting for all drones landing
    for (auto& t : threads){
        t.join();
    }

    std::cout << "\nSwarm flight completed successfully\n";
    return 0;
}