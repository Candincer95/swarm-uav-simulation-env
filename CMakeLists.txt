cmake_minimum_required(VERSION 3.10)

project(Swarm_UAV_Simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 REQUIRED COMPONENTS Widgets Core)

find_package(MAVSDK REQUIRED)
find_package(Threads REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_executable(Swarm_Manager_Node Swarm_Manager_Node.cpp)
add_executable(Forest_Generator Forest_Generator.cpp)
add_executable(gui_panel gui_control/gui_panel.cpp)


target_link_libraries(Swarm_Manager_Node
	MAVSDK::mavsdk
	Threads::Threads)
	
target_link_libraries(gui_panel
	Qt5::Widgets
	Qt5::Core)
	

install(TARGETS Swarm_Manager_Node DESTINATION lib/${Swarm_UAV_Simulation})



