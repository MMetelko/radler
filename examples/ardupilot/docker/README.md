# ArduPilot Example in a Docker Container

This code provides a Docker container environment that runs the ArduPilot example and uses NoVNC to allow access into a vehicle simulation setup and feedback windows.  Like in the original instructions, commands can be input into the "Vehicle Sim" window.

The original instructions for running this example can be found at (../README.rst).  This instruction set was for a Vagrant container setup and has been translated here into a Docker container.  The original instructions will help to understand this example and the windows that are created within web browser setup.

## Installation

Set up the SITL/MAVROS/Radler in a Docker container:

```
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
cp -r /path/to/radler-ros2-branch/example/ardupilot/docker/* .
docker build -t radler/ardupilot:novnc .
```

>Note: For VU local setup, the branch used is "thermo-example" which was started from "ros2" branch.

## Run Demonstration

This demonstration brings up 4 terminal windows that can be used by viewed by connection to a NoVNC connection webpage.  To start the demo, type `docker compose up`.  This action will start the processes indicated in `ardupilot_entrypoint.sh` to compile the Radler code and complete the system runtime setup.

To view the demonstration, open a web browser and go to `localhost:8090/vnc.html`.  This page contains a "Connect" button to select.  Once connected, a desktop view is provided for the demonstration.  Four xterm windows will appear with 3 of them being delayed for 3 minutes while the first one compiles the ArduPilot environment.  These 4 xterm windows automatically run the commands to start the demonstration.

The 4 xterm windows are as follows:
- ***Vehicle Sim***: starts the SITL simulator 
  - Two pop window appear one with a map of the SITL environment location and the other is a console showing the ground control status information
- ***MAVROS***: connects MAVROS with the SITL and displays the Arducopter's configuration
- ***Radler Gateway***: launches the Radler gateway ROS2 node
- ***Radler Battery***: launches the Radler afs_battery ROS2 node

Once all windows are up and running, the "Vehicle Sim" window can be used to control the flight of the Arducopter. Change the Arducopter's mode to GUIDED, arm throttle, then takeoff to an altitude (e.g., 30 meters) and one can observe the console window changing battery level and altitude.  The map also shows the location of the Arducopter on its flight.

Commands to type (hit return first to get a prompt):
```
mode guided
arm throttle
takeoff 30
position 100 100 0
```

>Note: the commands that run in each of the xterm windows can be found in app/conf.d/xterm.conf file

## Acknowledgements

The docker file setup for the ArduPilot simulation was created analyzing both the https://github.com/ArduPilot/ardupilot and https://github.com/SRI-CSL/radler repositories.  Files referenced were setups related to the vagrant environment and the Dockerfile that already existed but did not have to complete simulation environment.  The NoVNC elements utilized the https://github.com/theasp/docker-novnc repository example.