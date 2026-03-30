cmake_minimum_required(VERSION 3.10)
project(Autonomous_Flight)
project(Swarm_UAV)

set(CMAKE_CXX_STANDARD 17)


find_package(MAVSDK REQUIRED)
find_package(Threads REQUIRED)

add_executable(Autonomous_Flight Autonomous_Flight.cpp)
add_executable(Swarm_UAV Swarm_UAV.cpp)


target_link_libraries(Autonomous_Flight MAVSDK::mavsdk)
target_link_libraries(Swarm_UAV MAVSDK::mavsdk Threads::Threads)

