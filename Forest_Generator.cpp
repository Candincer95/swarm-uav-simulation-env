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
         << "    <plugin name=\"gz::sim::systems::Contact\" filename=\"gz-sim-contact-system\"/>\n"
         << "    <plugin name=\"gz::sim::systems::Thermal\" filename=\"gz-sim-thermal-system\"/>\n"

         // WIND PHYSICS
         << "    <plugin name=\"gz::sim::systems::WindEffects\" filename=\"gz-sim-wind-effects-system\">\n"
         << "      <force_approximation_scaling_factor>1</force_approximation_scaling_factor>\n"
         << "      <linear_velocity>0 0 0</linear_velocity>\n"
         << "    </plugin>\n"
         
         << "    <wind>\n"
         << "       <linear_velocity>0 0 0</linear_velocity>\n"
         << "    </wind>\n\n"


         
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
         << "      <name>x500_001</name>\n"
         << "      <uri>model://x500_depth</uri>\n"
         << "      <pose>0 0 1 0 0 0</pose>\n"
         << "    </include>\n\n"
         << "    \n"
         << "    <include>\n"
         << "      <name>x500_002</name>\n"
         << "      <uri>model://x500_depth</uri>\n"
         << "      <pose>0 4 1 0 0 0</pose>\n"
         << "    </include>\n\n"
         << "    \n"
         << "    <include>\n"
         << "      <name>x500_003</name>\n"
         << "      <uri>model://x500_depth</uri>\n"
         << "      <pose>0 -4 1 0 0 0</pose>\n"
         << "    </include>\n\n";

    // WINDSOCK FLAG
    file << "    \n"
         << "    <model name=\"windsock_flag\">\n"
         << "      <pose>6 0 0 0 0 0</pose>\n"
         << "      <link name=\"pole\">\n"
         << "        <pose>0 0 2 0 0 0</pose>\n"
         << "        <inertial><mass>100</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>\n"
         << "        <collision name=\"col\"><geometry><cylinder><radius>0.08</radius><length>4</length></cylinder></geometry></collision>\n"
         << "        <visual name=\"vis\">\n"
         << "          <geometry><cylinder><radius>0.08</radius><length>4</length></cylinder></geometry>\n"
         << "          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>\n"
         << "        </visual>\n"
         << "      </link>\n"
         << "      <joint name=\"fixed_to_ground\" type=\"fixed\">\n"
         << "        <parent>world</parent>\n"
         << "        <child>pole</child>\n"
         << "      </joint>\n"
         << "      <link name=\"fabric\">\n"
         << "        <pose>0.75 0 3.8 0 0 0</pose>\n"
         << "        <enable_wind>true</enable_wind>\n"
         << "        <inertial>\n"
         << "          <mass>0.15</mass>\n"
         << "          <inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia>\n"
         << "        </inertial>\n"
         << "      <collision name=\"col\"><geometry><box><size>1.5 0.02 0.6</size></box></geometry></collision>\n"
         << "      <visual name=\"vis\">\n"
         << "        <geometry><box><size>1.5 0.02 0.6</size></box></geometry>\n"
         << "        <material><ambient>1.0 0.3 0.0 1</ambient><diffuse>1.0 0.3 0.0 1</diffuse></material>\n"
         << "      </visual>\n"
         << "      </link>\n"
         << "      <joint name=\"hinge\" type=\"universal\">\n"
         << "        <parent>pole</parent>\n"
         << "        <child>fabric</child>\n"
         << "        <pose>-0.75 0 0 0 0 0</pose>\n"
         << "        <axis><xyz>0 0 1</xyz></axis>\n"
         << "        <axis2><xyz>0 1 0</xyz></axis2>\n"
         << "      </joint>\n"
         << "    </model>\n\n";

    // PROCEDURAL TREE GENERATION
    std::random_device rd;
    std::mt19937 gen(rd()); // MERSENNE TWISTER ALGORITHM FOR RANDOM TREE LOCATIONS
    std::uniform_real_distribution<float> dis_x(-80.0f, 80.0f); // Width = 160m
    std::uniform_real_distribution<float> dis_y(10.0f, 150.0f); // Depth = 140m

    int tree_count = 250;
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
    // PROCEDURAL FIRE GENERATION
    // Random fire points within the border of the forest
    std::uniform_real_distribution<float> dis_fire_x(-75.0f, 75.0f);
    std::uniform_real_distribution<float> dis_fire_y(15.0f, 145.0f); // Narrowed the borders

    float fire_x = dis_fire_x(gen);
    float fire_y = dis_fire_y(gen);

    std::cout << "[MISSION] FIRE OUTBREAK DETECTED AT TARGET COORDINATES: X: " 
              << std::fixed << std::setprecision(2) << fire_x << " Y: " << fire_y << std::endl;

     // GROUND TRUTH FILE
     std::string fire_loc_path = home_dir + "/swarm-uav-simulation-env/worlds/fire_ground_truth.txt";
     std::ofstream fire_file(fire_loc_path);
     if (fire_file.is_open()) {
          fire_file << fire_x << "\n" << fire_y << "\n";
          fire_file.close();
     } 

    file << "    \n"
         << "    <model name=\"forest_fire\">\n"
         << "      <static>true</static>\n"
         << "      <pose>" << fire_x << " " << fire_y << " 0 0 0 0</pose>\n"
         << "      <link name=\"link\">\n"
         << "        \n"
         << "        <visual name=\"fire_visual\">\n"
         << "          <pose>0 0 4.0 0 0 0</pose>\n"
         << "          <geometry><sphere><radius>5.0</radius></sphere></geometry>\n"
         << "          <material>\n"
         << "            <ambient>1.0 0.2 0.0 1</ambient>\n"
         << "            <diffuse>1.0 0.2 0.0 1</diffuse>\n"
         << "            <emissive>1.0 0.2 0.0 1</emissive>\n"
         << "          </material>\n"
         << "          \n"
         << "          <plugin filename=\"gz-sim-thermal-system\" name=\"gz::sim::systems::Thermal\">\n"
         << "            <temperature>1200.0</temperature>\n"
         << "          </plugin>\n"
         << "        </visual>\n"
         << "        <light name=\"fire_light\" type=\"point\">\n"
         << "          <pose>0 0 6.0 0 0 0</pose>\n"
         << "          <cast_shadows>false</cast_shadows>\n"
         << "          <intensity>15.0</intensity>\n" // Light intensity
         << "          <diffuse>1.0 0.4 0.0 1</diffuse>\n" // Orange flame color
         << "          <specular>0.8 0.3 0.0 1</specular>\n"
         << "          <attenuation>\n"
         << "            <range>100</range>\n" // It illuminates trees up to 40 meters away.
         << "            <linear>0.1</linear>\n"
         << "            <constant>0.2</constant>\n"
         << "            <quadratic>0.01</quadratic>\n"
         << "          </attenuation>\n"
         << "        </light>\n"
         << "      </link>\n"
         << "    </model>\n\n";
    file << "  </world>\n</sdf>\n";
    file.close();
    std::cout << "[SUCCESS] Forest_world.sdf successfully generated in the 'worlds' directory with " << tree_count << " trees and 3 embedded drones." << std::endl;
    return 0;
}
