cmake_minimum_required(VERSION 3.10)

project(Swarm_UAV_Simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(MAVSDK REQUIRED)
find_package(Threads REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_executable(Autonomous_Flight archive/Autonomous_Flight.cpp)
add_executable(Swarm_UAV archive/Swarm_UAV.cpp)
add_executable(Swarm_Manager_Node Swarm_Manager_Node.cpp)
add_executable(Forest_Generator Forest_Generator.cpp)


target_link_libraries(Autonomous_Flight MAVSDK::mavsdk)
target_link_libraries(Swarm_UAV MAVSDK::mavsdk Threads::Threads)
target_link_libraries(Swarm_Manager_Node
	MAVSDK::mavsdk
	Threads::Threads)
	

install(TARGETS Swarm_Manager_Node DESTINATION lib/${Swarm_UAV_Simulation})


