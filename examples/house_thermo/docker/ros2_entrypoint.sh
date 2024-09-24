#!/bin/bash
#set -e

#MM added for local run
source "setup_ros_env.sh"

# setup ros2 environment
source "$ROS2_PREFIX/$ROS2_DISTRO/setup.bash"

# build radler house_thermo example
#MM cd ~/radler 
#MM git pull
#MM (next command added for debugging locally)

cd ../../..
echo $PWD
./radler.sh --ws_dir $ROS2_WS/src compile examples/house_thermo/house_thermo.radl --plant plant --ROS
cd $ROS2_WS
colcon build \
    --cmake-args \
      -DSECURITY=ON --no-warn-unused-cli \
    --symlink-install
source "$ROS2_WS/install/local_setup.bash"

#sros2
cd ~

# MM: Workaround for ROS2 launch files until they are available in the radler repo
#MM copies directories modified for this setup only
#cp radler/examples/house_thermo/launch_files/* "$ROS2_WS/src/ros/house_thermo/launch/."
cp ~/temp/mmversion/radler/examples/house_thermo/launch_files/*.py "$ROS2_WS/src/ros/house_thermo/launch/."
mkdir -p "$ROS2_WS/src/ros/house_thermo/launch/config"
#cp radler/examples/house_thermo/config/* "$ROS2_WS/src/ros/house_thermo/launch/config/."
cp ~/temp/mmversion/radler/examples/house_thermo/config/*.yaml "$ROS2_WS/src/ros/house_thermo/launch/config/."
mkdir -p "$ROS2_WS/install/house_thermo/share/house_thermo/config"
ln -s "$ROS2_WS/src/ros/house_thermo/launch/config/house_computer_params.yaml" "$ROS2_WS/install/house_thermo/share/house_thermo/config/house_computer_params.yaml"
ln -s "$ROS2_WS/src/ros/house_thermo/launch/config/house_heater_params.yaml" "$ROS2_WS/install/house_thermo/share/house_thermo/config/house_heater_params.yaml"
mkdir -p "$ROS2_WS/install/house_thermo/share/house_thermo/launch"
ln -s "$ROS2_WS/src/ros/house_thermo/launch/house_thermo.plant.house_computer.launch.py" "$ROS2_WS/install/house_thermo/share/house_thermo/launch/house_thermo.plant.house_computer.launch.py"
ln -s "$ROS2_WS/src/ros/house_thermo/launch/house_thermo.plant.house_heater.launch.py" "$ROS2_WS/install/house_thermo/share/house_thermo/launch/house_thermo.plant.house_heater.launch.py"

ros2 security create_keystore sros2_keys
ros2 security create_key sros2_keys /house_thermo/buttons
ros2 security create_key sros2_keys /house_thermo/heater
ros2 security create_key sros2_keys /house_thermo/house
ros2 security create_key sros2_keys /house_thermo/thermometer
ros2 security create_key sros2_keys /house_thermo/thermostat

# Create directory to store application data from the nodes
mkdir -p "$ROS2_WS/data"

#MM exec "$@"

