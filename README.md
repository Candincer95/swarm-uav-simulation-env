# Swarm UAV Simulation Environment for Forest Fire Detection

This repository contains a standalone, digital twin-based simulation environment for autonomous Swarm UAVs (Unmanned Aerial Vehicles). 

## 1. Problem Definition
Traditional forest fire detection methods (watchtowers, satellite imaging) often suffer from delayed response times, blind spots, and low resolution. This project proposes an autonomous, highly scalable Swarm UAV architecture designed to perform zero-blind-spot area scanning over forest environments. By utilizing a cooperative swarm network, the system dramatically reduces the time required for early fire detection, functioning as a rapid-response digital twin mechanism.

## 2. System Architecture Schema
The system completely bypasses the standard ROS 2 overhead for highly stable network execution, utilizing a direct C++ MAVSDK structure. The general data flow is:

* **Simulation Engine:** Gazebo Harmonic (Procedural World & Physics) <--> PX4-Autopilot (SITL Flight Stack).
* **Swarm Manager (C++ Node):** Communicates with PX4 via MAVLink (UDP). Handles the Unified Comb Sweep Algorithm and formation locking.
* **Control Center (Qt GUI):** Listens to telemetry data via UDP sockets, visualizing the drone footprints and allowing dynamic parameter injection back into the simulation.

## 3. Dependencies & System Requirements
This project is developed and tested on the following technology stack:

* **OS:** Ubuntu 22.04 LTS
* **Flight Stack:** PX4-Autopilot (SITL v1.14+)
* **Simulation:** Gazebo Harmonic
* **Dependencies:** MAVSDK (C++), CMake (v3.10+), GCC (C++17), Qt (for GUI)

## 4. Installation Guide

**1. Clone the repository:**

```bash
git clone [https://github.com/Candincer95/swarm-uav-simulation-env.git](https://github.com/Candincer95/swarm-uav-simulation-env.git)
cd swarm-uav-simulation-env

```

**2. Build the Swarm Manager:**

```bash
mkdir build
cd build
cmake ..
make

```

## 5. How to Run the Simulation

**Step 1: Generate a Custom Forest Environment (Optional)**
The repository comes with a default world, but you can procedurally generate a brand new forest with randomized tree placements and a new fire ground-truth location.

```bash
cd build
./Forest_Generator

```

**Step 2: Start the Physical Environment (Gazebo Harmonic & PX4)**
Open a terminal, navigate to your `PX4-Autopilot` directory, and spawn the multi-vehicle simulation using our custom environment:

```bash
export PX4_GZ_WORLD=forest
Tools/simulation/gz/sitl_multiple_run.sh -n 3

```

**Step 3: Execute the Swarm Manager**
Once the drones are spawned, open a new terminal in this repository's root directory. The script will automatically handle zombie UDP processes (`Address already in use` errors):

```bash
chmod +x launch.sh
./launch.sh

```

## 6. Simulation Parameters & GUI Control

The simulation features a comprehensive Qt-based GUI that acts as an environment control center to adjust operational parameters on the fly.

![Swarm UAV Mission Control](images/gui.png)

**Configurable Parameters:**

* **Environment Dynamics (Wind Injection):** Operators can inject physical wind into the Gazebo environment.
* **Wind Intensity:** Adjustable from 0% to 100% (where 100% equals a physical 10 m/s wind speed).
* **Wind Direction:** Adjustable from 0° to 360° (where 0° indicates wind blowing exactly from the North). A simulated windsock provides visual ground-truth.


* **Cyber Attack (GPS) Constraints:**
* **Deny GPS Signal:** Simulates a complete loss of satellite connection.
* **Inject GPS Noise (Spoof):** Injects artificial error margins into the UAV localization data.
* **Recover to Normal:** Instantly restores normal sensor function and telemetry.



## 7. Data Formats & Mathematical Modeling

* **Coordinate Systems:** The system inherently processes mission data using Cartesian coordinates (X, Y, Z in meters) within the Swarm Manager. For PX4 flight commands, this is converted to WGS84 Geodetic coordinates (Latitude/Longitude) in real-time using a custom cosine scaling factor ($\approx 75440$ meters/degree), adjusted for the longitudinal narrowing at the Gazebo origin.
* **Sensor Footprint Geometry:** The comb sweep algorithm guarantees a 6-meter overlap. The margin is mathematically derived from a 25m operational altitude and the simulated thermal camera FOV (Field of View), ensuring a 20% safety overlap to eliminate blind spots.

## 8. Experiment Reproduction & Expected Outputs

**To Reproduce the Robustness Experiment:**

1. Start the simulation and initiate the Swarm Manager.
2. Observe the Live Swarm Radar on the GUI. The yellow paths represent the continuous thermal scanning footprint.
3. Use the GUI sliders to inject 50% Wind Intensity at a 90° Direction to observe formation elasticity.
4. Trigger the "Inject GPS Noise" button to observe how the Swarm Manager recalculates trajectories.

**Expected Output:**
The swarm will maintain a locked comb formation despite environmental interference. Once any individual UAV's FOV intersects with the dynamically generated fire ground-truth coordinates, the system calculates the Euclidean distance and halts the search. The GUI will instantly output a critical alert: **"FIRE DETECTED"**, pinpointing the discovering UAV ID and the exact X/Y Cartesian coordinates of the hazard.
