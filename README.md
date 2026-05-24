# Swarm UAV Simulation Environment for Forest Fire Detection

This repository contains a standalone, high-fidelity SITL (Software-in-the-Loop) simulation environment for autonomous Swarm UAVs (Unmanned Aerial Vehicles).

## 1. Problem Definition
Traditional forest fire detection methods (watchtowers, satellite imaging) often suffer from delayed response times, blind spots, and low resolution. This project proposes an autonomous, highly scalable Swarm UAV architecture designed to perform zero-blind-spot area scanning over forest environments. By utilizing a cooperative swarm network, the system dramatically reduces the time required for early fire detection, functioning as a rapid-response, high-fidelity SITL simulation framework.

## 2. System Architecture Schema
The system completely bypasses the standard ROS 2 overhead for highly stable network execution, utilizing a direct C++ MAVSDK structure. The general data flow is:

* **Simulation Engine:** Gazebo Harmonic (Procedural World & Physics) <--> PX4-Autopilot (SITL Flight Stack).
* **Swarm Manager (C++ Node):** Communicates with PX4 via MAVLink (UDP). Handles the Unified Comb Sweep Algorithm, staggered deployment, and formation locking.
* **Control Center (Qt GUI):** Listens to telemetry data via UDP sockets, visualizing the drone footprints and allowing dynamic parameter injection back into the simulation.

## 3. Dependencies & System Requirements
This project is developed and tested on the following technology stack:

* **OS:** Ubuntu 22.04 LTS
* **Flight Stack:** PX4-Autopilot (SITL v1.14+)
* **Simulation:** Gazebo Harmonic
* **Dependencies:** MAVSDK (C++), CMake (v3.10+), GCC (C++17), Qt (for GUI)

## 4. Installation Guide

**1. Clone the repository:**
By default, the simulation expects the repository to be located at `$HOME/swarm-uav-simulation-env`. If you clone it to a different directory, you must set the `SWARM_UAV_ROOT` environment variable.

```bash
git clone [https://github.com/Candincer95/swarm-uav-simulation-env.git](https://github.com/Candincer95/swarm-uav-simulation-env.git)
cd swarm-uav-simulation-env
export SWARM_UAV_ROOT=$(pwd)  # Run this if not cloned to your $HOME directory

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
The repository comes with a default world, but you can procedurally generate a brand new forest with randomized tree placements and a new fire ground-truth location. The generator also supports dynamic vehicle spawning.

**Note on Reproducibility:** Every procedurally generated forest is tied to a specific seed value, which is printed to the console upon generation. If you need to run multiple experiments on the exact same terrain, you can effortlessly recreate a previously generated forest by providing its specific seed.

```bash
cd build
./Forest_Generator               # Generates a random forest and outputs the seed
# ./Forest_Generator <seed_id>   # Recreates a specific forest using the given seed

```


**Step 2: Start the Physical Environment (Gazebo Harmonic & PX4)**
Before running the simulation for the first time, you must link your custom procedural world to PX4's internal world directory. Open a terminal and create a symlink (this only needs to be done once):

```bash
ln -sf ~/swarm-uav-simulation-env/worlds/Forest_world.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/Forest_world.sdf

```

Now, navigate to your `PX4-Autopilot` directory and spawn the multi-vehicle simulation using your newly linked custom environment:

```bash
cd ~/PX4-Autopilot
export PX4_GZ_WORLD=Forest_world
Tools/simulation/gz/sitl_multiple_run.sh -n 3

```

**Note:** The default swarm size is 3. If you wish to scale the system down for single-drone baseline testing, please read Section 9 for the required architectural parameter synchronizations before changing the `-n` flag.

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

* **Environment Dynamics (Wind Injection):** Operators can inject physical wind into the Gazebo environment
* **Wind Intensity:** Adjustable from 0% to 100% (where 100% equals a physical 10 m/s wind speed).
* **Wind Direction:** Adjustable from 0° to 360° (where 0° indicates wind blowing exactly from the North). A simulated windsock provides visual ground-truth.


* **Sensor Fault Injection (GPS):** Designed to test swarm robustness against sensor failures.
* **Inject GPS Denial (Block):** Injects a strict hardware failure condition blocking satellite connections.
* **Inject GPS Noise:** Injects artificial error margins into the UAV localization data.
* **Recover to Normal:** Instantly restores normal sensor function and triggers autonomous swarm regrouping.



## 7. Data Formats & Mathematical Modeling

* **Coordinate Scaling:** The system converts WGS84 Geodetic coordinates (Latitude/Longitude) to Cartesian coordinates (X, Y in meters) using a dynamically calculated longitudinal scaling factor: $Scale_X = R_{earth} \cdot \cos(\text{Latitude} \cdot \frac{\pi}{180})$. This ensures precise local projection and prevents silent failures if the Gazebo origin changes.
* **Geometric Proximity-Based Detection:** Fire detection is evaluated using geometric proximity to a dynamically generated ground-truth coordinate. (Note: Integration of a simulated physical thermal camera via sensor fusion is designated for future work).
* **FOV Derivation:** The scanning radius $R$ is derived from the operational altitude $h$ and the effective Field of View angle $\alpha$, where $R = h \cdot \tan(\alpha/2)$. Using an assumed $\tan(\alpha/2) \approx 0.15$ at $h = 25\text{m}$ yields a scanning radius of $3.75\text{m}$ (effective footprint width = $7.5\text{m}$).
* **Comb Sweep Coverage Guarantee:** The algorithm mathematically guarantees zero blind spots. For a swarm of $N=3$ UAVs with a discrete step size of $S = 18\text{m}$, the theoretical coverage per pass is $C = N \cdot 2R = 3 \cdot 7.5\text{m} = 22.5\text{m}$. The overlap is computed as $C - S = 4.5\text{m}$, ensuring a continuous $20\%$ coverage overlap ratio.

## 8. Experiment Reproduction & Expected Outputs

**To Reproduce the Robustness Experiment:**

1. Start the simulation and initiate the Swarm Manager.
2. Observe the Live Swarm Radar on the GUI. The yellow paths represent the continuous geometric scanning footprint based on the FOV derivation.
3. Use the GUI sliders to inject 50% Wind Intensity at a 90° Direction to observe formation elasticity and collision avoidance handling.
4. Trigger the "Inject GPS Noise" button to observe how the Swarm Manager autonomously disperses the UAVs to maintain safety radiuses.

**Expected Output:**
The swarm will execute a staggered deployment and maintain a locked comb formation despite environmental interference. Once any individual UAV's geometric proximity radius intersects with the hidden fire ground-truth coordinates, the system halts the autonomous mission. The GUI will instantly output a critical alert: "FIRE DETECTED", pinpointing the discovering UAV ID and the exact X/Y Cartesian coordinates of the hazard.

## 9. System Scalability & Known Limitations

The Swarm Manager architecture (UDP MAVLink routing, telemetry broadcasting, and independent node tracking) is dynamically scalable. However, for stable reproduction and physical validation on standard hardware, the optimal baseline is restricted to **1 to 3 UAVs**.

### Synchronizing Swarm Size (1 to 3 Drones)
The system is fully scalable down to a single UAV for baseline testing. However, the number of spawned vehicles must strictly match across the entire architecture to prevent system hanging or segmentation faults. If you change the simulation spawn parameter (e.g., to 1 drone), you **must** update all three layers:

1. **Simulation Engine:** `Tools/simulation/gz/sitl_multiple_run.sh -n 1`
2. **Swarm Manager:** Update `size_t TARGET_DRONES = 1;` in `Swarm_Manager_Node.cpp`.
3. **Environment Generator:** Update `int drone_count = 1;` in `Forest_Generator.cpp` (and rebuild/regenerate the SDF world).

### Known Limitations (Scaling N > 3)
Attempting to scale the system to 4 or more drones will introduce the following bottlenecks:

* **Hardware Constraints & Physics Dilation (Gazebo RTF):** Running multiple PX4 SITL instances concurrently with Gazebo Harmonic's physics engine is intensely CPU-bound. Spawning 4 or more UAVs on standard hardware (e.g., consumer laptops) will cause the simulation's Real-Time Factor (RTF) to drop significantly below 1.0. This time-dilation leads to unstable flight dynamics and erratic autopilot behavior.
* **Algorithmic Constraints (Lane Assignment):** The dynamic lane assignment in the `mode_UnifiedCombSweep()` algorithm is currently optimized and hardcoded for a 3-lane staggered deployment to guarantee the calculated overlap. Scaling beyond 3 drones requires modifying the `lane_index` assignment logic to support infinite $N$-width expansions without path-planning collisions.
* **Formation Geometry Scaling:** The pre-transit `Triangle (V-Shape)` and `Line Abreast` formations are geometrically hardcoded for a 3-agent hierarchy (1 Leader, 2 Wings). Simulating flocking behavior for $N > 3$ requires the future integration of dynamic spatial algorithms (e.g., Boids algorithm)

## Simulation Demo Video

[![Swarm UAV Simulation Demo](https://img.youtube.com/vi/tb9ah8XswYI/0.jpg)](https://youtu.be/tb9ah8XswYI)

*Click the image above to watch the autonomous swarm sweep algorithm and fire detection in action.*
