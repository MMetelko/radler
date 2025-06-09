#!/bin/bash
set -ex

USER_HOME="/home/ardupilot"
LOCK_FILE="/tmp/ardupilot_initialized.lock"

# Check if the lock file exists
if [ ! -f "$LOCK_FILE" ]; then
  touch "$LOCK_FILE" # create lock file

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

  # Generate the CycloneDDS config with the runtime IP
  echo "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>
  <CycloneDDS xmlns=\"https://cdds.io/config\">
  <Domain id=\"any\">
    <General>
    <NetworkInterfaceAddress>${ARDUPILOT_RUN_HOST}</NetworkInterfaceAddress>
    </General>
  </Domain>
  </CycloneDDS>" > ${USER_HOME}/cyclonedds.xml

  # Set the environment variable
  export CYCLONEDDS_URI=file://${USER_HOME}/cyclonedds.xml
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_DOMAIN_ID=0

  # setup ardupilot environment
  source ~/.ardupilot_env

  # setup ros2 environment
  source "$ROS2_PREFIX/$ROS2_DISTRO/setup.bash"
  source "$ROS2_WS/install/local_setup.bash"

  cd /ardupilot

  exec supervisord -c /app-novnc/supervisord.conf
  # See xterm.conf under app/conf.d to see the commands for the 4 different windows

else
  echo "ArduPilot environment already initialized.  Skipping initialization."
fi

