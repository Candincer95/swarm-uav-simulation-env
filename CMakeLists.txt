cmake_minimum_required(VERSION 3.10)
project(Autonomous_Flight)
project(Swarm_UAV)
project(Swarm_MAnager_Node)

set(CMAKE_CXX_STANDARD 17)


find_package(MAVSDK REQUIRED)
find_package(Threads REQUIRED)

add_executable(Autonomous_Flight Autonomous_Flight.cpp)
add_executable(Swarm_UAV Swarm_UAV.cpp)
add_executable(Swarm_Manager_Node Swarm_Manager_Node.cpp)


target_link_libraries(Autonomous_Flight MAVSDK::mavsdk)
target_link_libraries(Swarm_UAV MAVSDK::mavsdk Threads::Threads)
target_link_libraries(Swarm_Manager_Node MAVSDK::mavsdk Threads::Threads)


