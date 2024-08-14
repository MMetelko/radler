#!/bin/bash
set -e

# setup ros2 environment
source "$ROS2_PREFIX/$ROS2_DISTRO/setup.bash"

# build radler pubsub example
cd ~/radler 
git pull
./radler.sh --ws_dir $ROS2_WS/src compile examples/house_thermo/house_thermo.radl --plant plant --ROS
cd $ROS2_WS
colcon build \
    --cmake-args \
      -DSECURITY=ON --no-warn-unused-cli \
    --symlink-install
source "$ROS2_WS/install/local_setup.bash"

#sros2
cd ~
ros2 security create_keystore sros2_keys
ros2 security create_key sros2_keys /house_thermo/buttons
ros2 security create_key sros2_keys /house_thermo/heater
ros2 security create_key sros2_keys /house_thermo/house
ros2 security create_key sros2_keys /house_thermo/thermometer
ros2 security create_key sros2_keys /house_thermo/thermostat

exec "$@"
