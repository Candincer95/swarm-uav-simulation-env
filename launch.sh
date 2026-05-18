#!/bin/bash

echo "[SYSTEM] Cleaning up old processes..."
killall -9 px4 gz sim micro-xrce-dds-agent ruby gui_panel 2>/dev/null
sleep 1


trap "echo -e '\n[SYSTEM] Ctrl+C detected. Shutting down simulation and GUI...'; killall -9 px4 gz sim micro-xrce-dds-agent ruby gui_panel 2>/dev/null; exit" SIGINT SIGTERM

WORLD_PATH="$HOME/swarm-uav-simulation-env/worlds/Forest_world.sdf"
PX4_DIR="$HOME/PX4-Autopilot"
export GZ_SIM_RESOURCE_PATH=$HOME/PX4-Autopilot/Tools/simulation/gz/models

echo "[SYSTEM] 1. Starting Micro XRCE-DDS Agent..."
micro-xrce-dds-agent udp4 -p 8888 > /tmp/dds_agent.log 2>&1 &
sleep 1

echo "[SYSTEM] 2. Launching Gazebo independently from repo..."
gz sim $WORLD_PATH &

echo "[SYSTEM] Waiting 15 seconds for Gazebo and models to load..."
sleep 15

echo "[SYSTEM] 3. Spawning Right Follower (UAV 2)..."
(
    cd $PX4_DIR
    # Assign specific namespace (x500_002) and set Z-axis to 1 meter drop
    PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=Forest_world PX4_GZ_MODEL_NAME="x500_002" PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,4,1,0,0,0" PX4_SIM_MODEL=gz_x500_depth ./build/px4_sitl_default/bin/px4 -i 2 > /tmp/uav_2.log 2>&1
) &
sleep 6

echo "[SYSTEM] 4. Spawning Left Follower (UAV 3)..."
(
    cd $PX4_DIR
    # Assign specific namespace (x500_003) and set Z-axis to 1 meter drop
    PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=Forest_world PX4_GZ_MODEL_NAME="x500_003" PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,-4,1,0,0,0" PX4_SIM_MODEL=gz_x500_depth ./build/px4_sitl_default/bin/px4 -i 3 > /tmp/uav_3.log 2>&1
) &
sleep 6


echo "[SYSTEM] 4.5. Launching GUI Control Panel..."
GUI_PATH="$HOME/swarm-uav-simulation-env/gui_control/gui_panel"
if [ -f "$GUI_PATH" ]; then
    $GUI_PATH &
    echo "[OK] GUI Panel opened successfully."
else
    echo "[WARNING] gui_panel not found at $GUI_PATH! Did you put it in the gui_control folder and compile it?"
fi
# -------------------------------

echo "[SYSTEM] 5. Spawning Leader UAV (UAV 1) in FOREGROUND..."
echo "========================================================"
echo "Simulation and GUI are running! Press [Ctrl + C] to safely exit."
echo "========================================================"

cd $PX4_DIR
# Assign specific namespace (x500_001) and set Z-axis to 1 meter drop
PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=Forest_world PX4_GZ_MODEL_NAME="x500_001" PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,0,1,0,0,0" PX4_SIM_MODEL=gz_x500_depth ./build/px4_sitl_default/bin/px4 -i 1


killall -9 px4 gz sim micro-xrce-dds-agent ruby gui_panel 2>/dev/null
