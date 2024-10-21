#!/bin/bash
set -ex

RUN_FLUXBOX=${RUN_FLUXBOX:-yes}
RUN_XTERM=${RUN_XTERM:-yes}

case $RUN_FLUXBOX in
  false|no|n|0)
    rm -f /app/conf.d/fluxbox.conf
    ;;
esac

case $RUN_XTERM in
  false|no|n|0)
    rm -f /app/conf.d/xterm.conf
    ;;
esac

# setup ardupilot environment
source ~/.ardupilot_env

# Finish pulling the ardupilot repository
git submodule update --init --recursive

# setup ros2 environment
source "$ROS2_PREFIX/$ROS2_DISTRO/setup.bash"

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
cd ~
ros2 security create_keystore sros2_keys
ros2 security create_key sros2_keys /afs/mavros
ros2 security create_key sros2_keys /afs/gateway
ros2 security create_key sros2_keys /afs/afs_battery
ros2 security create_key sros2_keys /afs/afs_esp
ros2 security create_key sros2_keys /afs/afs_log

# From vagrant sros_env.bash
#mkdir -p $ROS2_WS/install/afs/lib/afs
mkdir -p $ROS2_WS/install/afs/bin
cd $ROS2_WS/install/afs/bin
ln -sf ../../../build/afs/gateway .
ln -sf ../../../build/afs/afs_battery .
ln -sf ../../../build/afs/afs_esp .
ln -sf ../../../build/afs/afs_log .
cd /ardupilot

exec supervisord -c /app/supervisord.conf
#exec "$@"

# Run sim_vehicle.py
#sim_vehicle.py -v ArduCopter --console --map -m --out=127.0.0.1:14550

# Run ros2 launch mavros apm.launch.py
#ros2 launch mavros apm.launch.py

# Run gateway
#source ~/ros2_ws/install/local_setup.bash
#~/ros2_ws/install/afs/bin/gateway

# Run afs_battery
#source ~/ros2_ws/install/local_setup.bash
#~/ros2_ws/install/afs/bin/afs_battery

