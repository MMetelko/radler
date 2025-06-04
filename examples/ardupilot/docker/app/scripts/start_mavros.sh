# In your start_mavros.sh script, add a delay to ensure sim_vehicle is fully started
#!/bin/bash
# Wait for sim_vehicle to fully initialize
sleep 10
# Start MAVROS with additional logging
ros2 launch mavros descert_adj.launch.py fcu_url:="udp://127.0.0.1:14550@" > /tmp/mavros.log 2>&1