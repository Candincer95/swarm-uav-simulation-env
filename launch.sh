#!/bin/bash

echo "--- [SYSTEM] PRE-FLIGHT CLEANUP ---"
echo "Cleaning up zombie UDP processes and old nodes..."
killall -9 Swarm_Manager gui_panel 2>/dev/null

echo "--- [SYSTEM] LAUNCHING CONTROL CENTER ---"
# 1. Start the Qt GUI in the background
./build/gui_panel &

# Wait 2 seconds for the GUI to initialize and start listening to ports
sleep 2

echo "--- [SYSTEM] LAUNCHING SWARM MANAGER ---"
# 2. Start the C++ Swarm Manager in the foreground
./build/Swarm_Manager
