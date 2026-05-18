```markdown
# Swarm UAV Simulation Environment for Forest Fire Detection

This repository contains a standalone, digital twin-based simulation environment for autonomous Swarm UAVs (Unmanned Aerial Vehicles). The system is specifically designed for early forest fire detection using procedurally generated environments, dynamic thermal signatures, and cooperative swarm flight algorithms.

## Key Features

* **Procedural Forest & Fire Generation:** Spawns a randomized forest environment and a dynamic thermal fire signature (Ground Truth) in Gazebo Classic.
* **Unified Comb Sweep Algorithm:** Implements a coordinated, sector-based "Lawnmower" pattern with a 6-meter overlap for zero-blind-spot area scanning. The swarm flies in a locked comb formation.
* **Autonomous Swarm Management:** Built with C++ and MAVSDK, bypassing standard ROS 2 overhead for highly stable, standalone network execution and memory management.
* **Automated Network Cleanup:** Includes a custom launch script to automatically handle zombie UDP processes and MAVLink port conflicts (`Address already in use` errors).
* **Real-time Detection & Telemetry:** Monitors individual UAV FOV (Field of View) footprint and calculates the Euclidean distance to the fire source to trigger automated GUI alerts.

## System Requirements

This project is developed and tested on the following technology stack:
* **OS:** Ubuntu 22.04 LTS
* **Flight Stack:** PX4-Autopilot (SITL)
* **Simulation:** Gazebo HArmonic
* **Dependencies:** MAVSDK (C++), CMake (v3.10+), GCC (C++17)

## Installation & Build Guide

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

## How to Run the Simulation

To prevent UDP port conflicts and ensure a clean startup, a custom launch script (`launch.sh`) is provided.

**Step 1: Start the Physical Environment (Gazebo & PX4)**
Open a terminal, navigate to your `PX4-Autopilot` directory, and spawn the multi-vehicle simulation (e.g., for 3 drones):

```bash
Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n 3

```

**Step 2: Execute the Swarm Manager**
Once the drones are spawned in Gazebo, open a new terminal in this repository's root directory. Make sure the script is executable, then run it:

```bash
chmod +x launch.sh
./launch.sh

```

*Note: The `launch.sh` script will automatically kill any ghost MAVLink ports (9090, 14581-14583), clean up the network, and safely start the Swarm Manager Node.*

## Architecture & Mathematical Modeling

* **Geodetic Coordinate Conversion:** The simulation translates Cartesian coordinates (meters) to WGS84 Geodetic coordinates (Latitude/Longitude) in real-time. It utilizes a custom cosine scaling factor ($\approx 75440$ meters/degree) to adjust for the longitudinal narrowing at the Gazebo origin (Zurich, $47.39^{\circ}$ N).
* **Sensor Footprint Geometry:** The swarm overlap margin is mathematically calculated based on a 25m operational altitude and the simulated thermal camera FOV footprint, ensuring a 20% safety overlap margin to eliminate blind spots
