#!/bin/bash
set -e

# setup ros2 environment
source "$ROS2_PREFIX/$ROS2_DISTRO/setup.bash"

# make sure daikon environment is setup
source ~/.profile

# build radler house_thermo example
cd ~/radler 
git pull
./radler.sh --ws_dir $ROS2_WS/src compile examples/ardupilot/afs.radl --plant plant --ROS
cd $ROS2_WS
colcon build \
    --cmake-args \
      -DSECURITY=ON --no-warn-unused-cli \
    --symlink-install
source "$ROS2_WS/install/local_setup.bash"

#sros2
# From vagrant/sros_keystore.bash
cd $ROS2_WS
ros2 security create_keystore sros2_keys
ros2 security create_key sros2_keys /afs/mavros
ros2 security create_key sros2_keys /afs/gateway
ros2 security create_key sros2_keys /afs/afs_battery
ros2 security create_key sros2_keys /afs/afs_esp
ros2 security create_key sros2_keys /afs/afs_log

# From vagrant sros_env.bash
mkdir -p $ROS2_WS/install/afs/lib/afs
cd $ROS2_WS/install/afs/lib/afs
ln -sf ../../../../build/afs/gateway .
ln -sf ../../../../build/afs/afs_battery .
ln -sf ../../../../build/afs/afs_esp .
ln -sf ../../../../build/afs/afs_log .
cd -

exec "$@"

