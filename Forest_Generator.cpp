#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <random>
#include <iomanip>

int main() {
    // SAVE FILE DIRECTLY TO 'WORLDS' DIRECTORY
    std::string home_dir = getenv("HOME");
    std::string output_path = home_dir + "/swarm-uav-simulation-env/worlds/Forest_world.sdf";

    std::ofstream file(output_path);

    if (!file.is_open()) {
        std::cerr << "[ERROR] File could not be created! Please ensure you have a directory named 'worlds' in your repo." << std::endl;
        return 1;
    }

    file << "<?xml version=\"1.0\" ?>\n"
         << "<sdf version=\"1.9\">\n"
         << "  <world name=\"Forest_world\">\n"
         << "    <physics type=\"ode\">\n"
         << "      <max_step_size>0.004</max_step_size>\n"
         << "      <real_time_factor>1.0</real_time_factor>\n"
         << "    </physics>\n"
         
         << "    <plugin name=\"gz::sim::systems::Physics\" filename=\"gz-sim-physics-system\"/>\n"
         << "    <plugin name=\"gz::sim::systems::SceneBroadcaster\" filename=\"gz-sim-scene-broadcaster-system\"/>\n"
         << "    <plugin name=\"gz::sim::systems::UserCommands\" filename=\"gz-sim-user-commands-system\"/>\n"
         << "    <plugin name=\"gz::sim::systems::Sensors\" filename=\"gz-sim-sensors-system\">\n"
         << "      <render_engine>ogre2</render_engine>\n"
         << "    </plugin>\n"
         << "    <plugin name=\"gz::sim::systems::Imu\" filename=\"gz-sim-imu-system\"/>\n"
         << "    <plugin name=\"gz::sim::systems::NavSat\" filename=\"gz-sim-navsat-system\"/>\n"
         << "    <plugin name=\"gz::sim::systems::Magnetometer\" filename=\"gz-sim-magnetometer-system\"/>\n"
         << "    <plugin name=\"gz::sim::systems::AirPressure\" filename=\"gz-sim-air-pressure-system\"/>\n"
         << "    <plugin name=\"gz::sim::systems::Contact\" filename=\"gz-sim-contact-system\"/>\n\n"
         
         // GPS COORDINATES (Prevents PX4 from crashing)
         << "    <spherical_coordinates>\n"
         << "      <surface_model>EARTH_WGS84</surface_model>\n"
         << "      <world_frame_orientation>ENU</world_frame_orientation>\n"
         << "      <latitude_deg>47.397742</latitude_deg>\n"
         << "      <longitude_deg>8.545594</longitude_deg>\n"
         << "      <elevation>0</elevation>\n"
         << "      <heading_deg>0</heading_deg>\n"
         << "    </spherical_coordinates>\n\n"

         << "    <light type=\"directional\" name=\"sun\">\n"
         << "      <cast_shadows>true</cast_shadows>\n"
         << "      <pose>0 0 10 0 0 0</pose>\n"
         << "      <diffuse>0.8 0.8 0.8 1</diffuse>\n"
         << "      <specular>0.2 0.2 0.2 1</specular>\n"
         << "      <direction>-0.5 0.1 -0.9</direction>\n"
         << "    </light>\n\n"

         << "    <model name=\"ground_plane\">\n"
         << "      <static>true</static>\n"
         << "      <link name=\"link\">\n"
         << "        <collision name=\"collision\">\n"
         << "          <geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry>\n"
         << "        </collision>\n"
         << "        <visual name=\"visual\">\n"
         << "          <geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry>\n"
         << "          <material>\n"
         << "            <ambient>0.2 0.4 0.2 1</ambient>\n"
         << "            <diffuse>0.2 0.4 0.2 1</diffuse>\n"
         << "          </material>\n"
         << "        </visual>\n"
         << "      </link>\n"
         << "    </model>\n\n";

    // EMBED DRONES DIRECTLY INTO THE MAP (launch.sh will attach to these models)
    file << "    \n"
         << "    <include>\n"
         << "      <name>x500_1</name>\n"
         << "      <uri>model://x500</uri>\n"
         << "      <pose>0 0 1 0 0 0</pose>\n"
         << "    </include>\n\n"
         << "    \n"
         << "    <include>\n"
         << "      <name>x500_2</name>\n"
         << "      <uri>model://x500</uri>\n"
         << "      <pose>0 4 1 0 0 0</pose>\n"
         << "    </include>\n\n"
         << "    \n"
         << "    <include>\n"
         << "      <name>x500_3</name>\n"
         << "      <uri>model://x500</uri>\n"
         << "      <pose>0 -4 1 0 0 0</pose>\n"
         << "    </include>\n\n";

    // PROCEDURAL TREE GENERATION
    std::random_device rd;
    std::mt19937 gen(rd()); // MERSENNE TWISTER ALGORITHM FOR RANDOM TREE LOCATIONS
    std::uniform_real_distribution<float> dis_x(-40.0f, 40.0f);
    std::uniform_real_distribution<float> dis_y(10.0f, 80.0f);

    int tree_count = 100;
    for (int i = 0; i < tree_count; ++i) {
        float x = dis_x(gen);
        float y = dis_y(gen);

        file << "    <model name=\"tree_" << i << "\">\n"
             << "      <static>true</static>\n"
             << "      <pose>" << std::fixed << std::setprecision(2) << x << " " << y << " 0 0 0 0</pose>\n"
             << "      <link name=\"link\">\n"
             << "        <collision name=\"trunk_col\">\n"
             << "          <pose>0 0 2.5 0 0 0</pose>\n"
             << "          <geometry><cylinder><radius>0.4</radius><length>5</length></cylinder></geometry>\n"
             << "        </collision>\n"
             << "        <visual name=\"trunk_vis\">\n"
             << "          <pose>0 0 2.5 0 0 0</pose>\n"
             << "          <geometry><cylinder><radius>0.4</radius><length>5</length></cylinder></geometry>\n"
             << "          <material>\n"
             << "            <ambient>0.3 0.15 0.05 1</ambient>\n"
             << "            <diffuse>0.3 0.15 0.05 1</diffuse>\n"
             << "          </material>\n"
             << "        </visual>\n"
             << "        <collision name=\"leaves_col\">\n"
             << "          <pose>0 0 8.5 0 0 0</pose>\n"
             << "          <geometry><cone><radius>2.5</radius><length>7</length></cone></geometry>\n"
             << "        </collision>\n"
             << "        <visual name=\"leaves_vis\">\n"
             << "          <pose>0 0 8.5 0 0 0</pose>\n"
             << "          <geometry><cone><radius>2.5</radius><length>7</length></cone></geometry>\n"
             << "          <material>\n"
             << "            <ambient>0.0 0.4 0.0 1</ambient>\n"
             << "            <diffuse>0.0 0.4 0.0 1</diffuse>\n"
             << "          </material>\n"
             << "        </visual>\n"
             << "      </link>\n"
             << "    </model>\n";
    }

    file << "  </world>\n</sdf>\n";
    file.close();
    std::cout << "[SUCCESS] Forest_world.sdf successfully generated in the 'worlds' directory with 3 embedded drones." << std::endl;
    return 0;
}