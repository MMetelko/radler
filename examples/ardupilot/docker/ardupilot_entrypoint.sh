#!/bin/bash
set -ex

RUN_FLUXBOX=${RUN_FLUXBOX:-yes}
RUN_XTERM=${RUN_XTERM:-yes}

case $RUN_FLUXBOX in
  false|no|n|0)
    rm -f /app-novnc/conf.d/fluxbox.conf
    ;;
esac

case $RUN_XTERM in
  false|no|n|0)
    rm -f /app-novnc/conf.d/xterm.conf
    ;;
esac

# setup ardupilot environment
source ~/.ardupilot_env

# setup ros2 environment
source "$ROS2_PREFIX/$ROS2_DISTRO/setup.bash"
source "$ROS2_WS/install/local_setup.bash"

cd /ardupilot

exec supervisord -c /app-novnc/supervisord.conf

# See xterm.conf under app/conf.d to see the commands for the 4 different windows

