#!/bin/bash
sleep 10
source ${ROS2_PREFIX}/${ROS2_DISTRO}/setup.bash
ros2 launch mavros apm.launch.py