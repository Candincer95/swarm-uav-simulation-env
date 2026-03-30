#include <iostream>
#include <thread>
#include <chrono>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

int main() {
    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};

    //Connect to SITL UDP port
    ConnectionResult connection_result = mavsdk.add_any_connection("udp://:14540");
    if (connection_result != ConnectionResult::Success){
        std::cerr << "Connection Error:" << connection_result << '\n';
        return 1;
    }

    std::cout << "Waiting for the connection to SITL Simulation\n";

    while (mavsdk.systems().size()==0){
        sleep_for(seconds(1));
    }
    auto system = mavsdk.systems().front();
    std::cout << "Drone connected successfully\n";
    
    //Starting Plugins
    auto telemetry = Telemetry{system};
    auto action = Action{system};

    //Waiting for EKF2 controls
    std::cout <<"Waiting for GPS\n";
    while (!telemetry.health().is_global_position_ok){
        sleep_for(seconds(1));

    }
    std::cout <<"GPS is OK, drone is ready to flight\n";

    //Starting engines
    std::cout <<"Engines are starting\n";
    if (action.arm() != Action::Result::Success){
        std::cerr <<"Arm process failed\n";
        return 1;

    }

    //Takeoff
    std::cout <<"Taking off\n";
    if(action.takeoff() != Action::Result::Success){
        std::cerr <<"Takeoff failed\n";
        return 1;
    }
    sleep_for(seconds(8)); //Given time to climb to the desired altitude

    //Waypoint Follow
    std::cout <<"Moving towards Waypoint\n";
    auto position = telemetry.position();

    //To given point that 20 meters horizontally and vertically
    double target_lat = position.latitude_deg + 0.0002;
    double target_lon = position.longitude_deg + 0.0002;
    float target_alt = position.absolute_altitude_m + 5.0f; //5 meters higher

    //Send the drone to the specified coordinates

    action.goto_location(target_lat, target_lon, target_alt, 0.0f);
    sleep_for(seconds(15)); //Given time to reach the location

    //Landing
    std::cout << "Landing command given\n";
    if (action.land() != Action::Result::Success){
        std::cerr << "Landing is failed\n";
        return 1;
    }

    //Waiting for the drone to touch the ground
    while (telemetry.in_air()){
        std::cout << "Landing...\n";
        sleep_for(seconds(3));
    }
    std::cout <<"Mission accomplished, drone is on the ground.\n";
    return 0;



}