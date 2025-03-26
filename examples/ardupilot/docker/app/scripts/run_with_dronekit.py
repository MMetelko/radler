#!/usr/bin/python

import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
import time
from math import radians, cos
from pymavlink import mavutil
import os
import signal
import sys
import pexpect

controller = None

class DroneController:
    def __init__(self, connection_str):
        self.connection_str = connection_str
        self.vehicle = None
    
    def connect(self):
        self.vehicle = connect(self.connection_str, wait_ready=True, timeout=60)
        self.target_system = self.vehicle._master.target_system
        print(f"Connected to the vehicle with target system ID = {self.target_system}.")
        self.setup_listeners()

    def setup_listeners(self):
        @self.vehicle.on_attribute('last_heartbeat')
        def heartbeat_timeout(self, attr_name, value):
            if value > 30:
                print("No heartbeat in 30 seconds")
                self.reconnect()
        
    def reconnect(self):
        max_retries = 5
        for attempt in range(max_retries):
            try:
                if self.vehicle:
                    self.vehicle.close()
                self.vehicle = connect(self.connection_str, wait_ready=True, timeout=60)
                print("Successfully reconnected to vehicle.")
                self.setup_listeners()
                return True
            except Exception as e:
                print(f"Connection attempt {attempt+1} failed: {str(e)}")
                time.sleep(5)
        print("Failed to reconnect after maximum attempts.")
        return False
       
    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms the vehicle and flies to aTargetAltitude.
        """
        print("Basic pre-arm checks")
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialize...")
            time.sleep(1)

        print("Changing to GUIDED mode...")
        self.vehicle.mode = VehicleMode("GUIDED")
        while self.vehicle.mode != 'GUIDED':
            print(" Waiting for guiding mode...")
            time.sleep(1)

        print("Arming motors...")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude)

        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
                print("Reached target altitude.")
                break
            elif self.vehicle.mode == 'RTL':
                print("A fail-safe mechanism changed the vehicle mode to RTL.")
                break
            time.sleep(1)

    def get_location_offset_meters(self, original_location, dNorth, dEast, altDelta):
        """
        Returns a LocationGlobalRelative object containing the latitude/longitude `dNorth` and `dEast` meters from the
        specified `original_location`. The returned Location has the same `altDelta` value as `original_location`.
        """
        earth_radius = 6378137.0  # Radius of "spherical" earth

        # Coordinate offsets in radians
        dLat = dNorth / earth_radius
        dLon = dEast / (earth_radius * cos(radians(original_location.lat)))

        # New position in decimal degrees
        newlat = original_location.lat + dLat * (180.0 / 3.14159)
        newlon = original_location.lon + dLon * (180.0 / 3.14159)
        newalt = original_location.alt + altDelta
        return LocationGlobalRelative(newlat, newlon, newalt)
    
    def goto_position_ned(self, dNorth, dEast, dAlt, tolerance=1):
        """
        Move the vehicle to a position `dNorth` and `dEast` meters away from the current position, maintaining altitude change `dAlt`.
        """
        current_location = self.vehicle.location.global_relative_frame
        target_location = self.get_location_offset_meters(current_location, dNorth, dEast, dAlt)
        print(f"Moving to relative position (NORTH: {dNorth}m, EAST: {dEast}m, ALT: {dAlt}m)")
        self.vehicle.simple_goto(target_location)
        while True:
            current_location = self.vehicle.location.global_relative_frame
            dist = self.get_location_offset_meters(current_location, dNorth, dEast, dAlt)
            if dist <= tolerance:
                print("Reached target location")
                break
            time.sleep(1)

    def land_and_wait_for_altitude(self):
        """
        Command the vehicle to land and wait until it reaches an altitude of 0 meters.
        """
        print("Initiating landing...")
        self.vehicle.mode = VehicleMode("LAND")
        while self.vehicle.mode != 'LAND':
            print(" Waiting for landing mode...")
            time.sleep(1)

        # Wait until the vehicle reaches an altitude of 0 meters
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt <= 0.1:
                print("Landed. Altitude: ", self.vehicle.location.global_relative_frame.alt)
                break
            time.sleep(1)

    def run_sim(self, vertMovement=100, hortMovement=100, altitude=30, use_waypoints=False):
        # Main script starts here
        try:
            print(f"Taking off to indicated altitude of {altitude} (in meters)")
            self.arm_and_takeoff(altitude)

            if use_waypoints:
                # Stay at the altitude for 3 minutes
                #time.sleep(180)
                # If geofence breach happens, it does RTL
                if self.vehicle.mode != 'RTL':
                    print("Changing to AUTO mode...")
                    self.vehicle.mode = VehicleMode("AUTO")
                    while self.vehicle.mode != 'AUTO':
                        print(" Waiting for auto mode...")
                        time.sleep(1)
                    print("Starting mission ...")
                    # After mission completes, the mode switches to RTL and then disarms
                    last_command = None
                    while self.vehicle.armed:
                        current_command = self.vehicle.commands.next
                        if current_command != last_command:
                            print(f" Currently on command: {current_command}...")
                            last_command = current_command
                        time.sleep(1)
                    print("Mission Complete!  Returning to launch...")
                else:  # Fence was probably breached or battery too low, so vehicle got commanded to 'RTL'
                    print("Mission has been interrupted by fail safe mechanisms of Radler.  Returning to launch...")

            else:
                #print(f"Flying to relative position: North/South = {vertMovement}, East/West = {hortMovement}, Altitute Change = 0)")
                self.goto_position_ned(vertMovement, hortMovement, 0)

            #MM TODO: this may interfere with Radler battery low actions - checkout later
            #self.land_and_wait_for_altitude()
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            
        finally:
            print("Completed vehicle operations.")
    
    # Function to enable or disable the GPS
    def config_gps_enable_param(self, gps_state):
        if gps_state:
            self.vehicle.parameters['GPS1_TYPE'] = 1
        else:
            self.vehicle.parameters['GPS1_TYPE'] = 0            

        # Verify the change
        print(f"Adjusted GPS1_TYPE value: {self.vehicle.parameters['GPS1_TYPE']}")
        self.vehicle.wait_ready('parameters', timeout=300)
           
    # Load simulation parameters
    def load_sim_params(self):
        sim_parameters_file_path = os.path.join("/home/ardupilot/radler/examples", "ardupilot", "sitl_config", "sim_parameters.txt")
        with open(sim_parameters_file_path, 'r') as f:
            for line in f:
                if line.startswith('#'):
                    continue
                name, value = line.strip().split('=')
                self.vehicle.parameters[name.strip()] = float(value.strip())
                time.sleep(0.1)  # Small delay to avoid overwhelming the connection

            print("Parameters loaded.  Waiting for them to take effect...")
            self.vehicle.wait_ready('parameters', timeout=300)
   
    # Upload Mission Waypoints
    def upload_waypoint(self, i, wp, max_retries=3):
        for attempt in range(max_retries):
            self.vehicle._master.mav.mission_item_int_send(
                self.target_system, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                i, *wp[1:])
            ack = self.vehicle._master.recv_match(type='MISSION_ACK', blocking=True, timeout=15)
            if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                return True
        return False
        
    def load_mission_waypoints(self):
        try:
            mission_waypoint_file_path = os.path.join("/home/ardupilot/radler/examples", "ardupilot", "sitl_config", "mission.txt")

            cmds = self.vehicle.commands
            cmds.clear()
            
            # Read waypoints from file
            with open(mission_waypoint_file_path, 'r') as f:
                next(f)  # Skip header row
                for line in f:
                    parts = line.strip().split('\t')
                    if len(parts) == 12:
                        linearray=line.split('\t')
                        seq=int(linearray[0])
                        currentwp=int(linearray[1])
                        frame=int(linearray[2])
                        command=int(linearray[3])
                        param1=float(linearray[4])
                        param2=float(linearray[5])
                        param3=float(linearray[6])
                        param4=float(linearray[7])
                        x=float(linearray[8])
                        y=float(linearray[9])
                        z=float(linearray[10])
                        autocontinue=int(linearray[11].strip())
                        cmd = Command( 
                                    self.target_system, 
                                    mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
                                    seq, frame, command, 
                                    currentwp, autocontinue,
                                    param1, param2, param3, param4,
                                    x, y, z)
                        cmds.add(cmd)
                        
            cmds.upload()
            # Note: The following warning appears on the console, but it does show "Flight plan received"
            #    Got MISSION_ACK: TYPE_MISSION: ACCEPTED
            #    AP: got MISSION_ITEM; GCS should send MISSION_ITEM_INT
            #    Got MISSION_ACK: TYPE_MISSION: ACCEPTED
            #    AP: Flight plan received
            print(f"Mission uploaded: {cmds.count} waypoints")      
        except Exception as e:
            print(f"Unexpected mission error: {str(e)}")

    def display_fence(self):
        try:
            # Now interact with MAVProxy using pexpect
            child = pexpect.spawn('mavproxy.py --master=udp:127.0.0.1:14550', timeout=120)
            print("Spawned mavproxy...")

            # Expect MAVProxy to start and present its command prompt
            child.expect('MAV>', timeout=60)
            print("Ready to send mavproxy command...")

            # Send the 'fence list' command to MAVProxy
            child.sendline('fence list')
            print("Sent mavproxy fence list command")
  
            # Wait for the response, which might include info about the fence loaded
            prompt_pattern = '([A-Z]+>)'
            child.expect(prompt_pattern, timeout=60)
            print(f"MAVProxy '{child.match.group(1)}' prompt received.")

            # You can print the response or handle it as needed.
            child.expect('MAV>', timeout=60)
            print(child.before.decode('utf-8'))

            # Close the MAVProxy process
            time.sleep(1)
            child.close()
            print("Closed mavproxy communication link.")
        except Exception as e:
            print(f"Unexpected mavproxy communication error: {str(e)}")
  
    # Setup Geofence
    def load_geofence(self):
        try:            
            # Load fence.txt file
            fence_file_path = os.path.join("/home/ardupilot/radler/examples", "ardupilot", "sitl_config", "fence.txt")
            with open(fence_file_path, 'r') as f:
                points = [line.strip().split('\t') for line in f if line.strip()]

            # First setup the desired parameters
            fence_params = {
                'FENCE_ACTION': 1,
                'FENCE_ALT_MAX': 150.0,
                'FENCE_RADIUS': 500.0,
                'FENCE_TOTAL': len(points) - 1,
                'FENCE_TYPE': 7
            }
            
            # First check if fence has already been loaded
            fence_total = self.vehicle.parameters['FENCE_TOTAL']
            print(f"Current fence_total is {fence_total}.")
            if (fence_total > 0) and (fence_params['FENCE_TOTAL'] == fence_total):
                print("The geofence is already loaded.")
            else:
                for param, value in fence_params.items():
                    self.vehicle.parameters[param] = value
                    print(f"{param}: {self.vehicle.parameters[param]}")
                                
                # Set fence points, do not load the last point
                for i, point in enumerate(points[:-1]):
                    lat, lon = map(float, point)
                    msg = self.vehicle.message_factory.fence_point_encode(
                        target_system=self.target_system,
                        target_component=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
                        idx=i,
                        count=len(points) - 1,
                        lat=lat,
                        lng=lon
                    )
                    self.vehicle.send_mavlink(msg)
                    self.vehicle.flush()
                    # Note: this number was determine on a local system by trial and error, 
                    # it may need to be increased in the server situation
                    # It does indicate the following possible messages on the console, 
                    # but resolves with "fence OK" and "pre-arm good"
                    #    AP: AC_Fence: invalid polygon vertex count 1
                    #    pre-arm fail
                    #    AP: AC_Fence: invalid polygon vertex count 2
                    #    AP: PreArm: Polygon fence(s) invalid
                    #    fence breach
                    #    fence OK
                    #    pre-arm good
                    time.sleep(0.3)  # slight delay to ensure message delivery
                    
                uploaded_fence_pts_total = self.vehicle.parameters['FENCE_TOTAL']
                print(f"Fence uploaded: {uploaded_fence_pts_total} points")
                self.vehicle.wait_ready('parameters', timeout=300)

                # Verify fence points
                if self.vehicle.parameters['FENCE_TOTAL'] == (len(points) - 1):
                    print("Geofence successfully uploaded and verified.")
                else:
                    print("Geofence upload may have failed. Please verify.")
                    
            # Show the fence
            self.vehicle.parameters['FENCE_ENABLE'] = 1
            self.vehicle.flush()
            self.vehicle.wait_ready('parameters', timeout=300)
            
            #self.display_fence()
            print("Geofence made visible.")
    
        except Exception as e:
            print(f"Error loading geofence: {str(e)}")

    # Function to send custom MAVLink battery reset command
    def send_batreset(self):
        """
        Send a custom MAVLink command to reset the battery state in the simulation.
        """
        try:
            msg = self.vehicle.message_factory.command_long_encode(
                self.target_system, 
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
                mavutil.mavlink.MAV_CMD_BATTERY_RESET, # command
                0,    # confirmation
                -1, 100, 0, 0, 0, 0, 0  
            )      
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            print("Battery reset command sent.")
        except Exception as e:
            print(f"Error resetting Battery level: {str(e)}")
        
    def wait_for_disarm(self):
        while self.vehicle.armed:
            print(" Waiting for disarmed mode...")
            time.sleep(1)
        print("Vehicle is disarmed")

    def emergency_reset(self):
        self.vehicle.mode = VehicleMode("GUIDED")
        while self.vehicle.mode != 'GUIDED':
            print(" Waiting for guiding mode...")
            time.sleep(1)

        self.vehicle.mode = VehicleMode("RTL")
        while self.vehicle.mode != 'RTL':
            print(" Waiting for RTL mode...")
            time.sleep(1)
            
        self.wait_for_disarm()
        
        self.vehicle.mode = VehicleMode("STABILIZE")
        while self.vehicle.mode != 'STABILIZE':
            print(" Waiting for stabilize mode...")
            time.sleep(1)
        print("Emergency reset complete")
       
    # Reset battery
    def reset_simulation(self):
        # Make sure current mission waypoint is index 0
        msg = self.vehicle.message_factory.mission_set_current_encode(
                self.target_system, 
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
                0,  # seq (the sequence number of the mission item)
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()  
            
        # Make sure flight mode is reset
        if self.vehicle.mode == 'AUTO':
            self.emergency_reset()
        elif self.vehicle.mode == 'LAND':
            self.vehicle.mode = VehicleMode("GUIDED")
            while not self.vehicle.mode.name == "GUIDED":
                print("Waiting for mode change to GUIDED...")
                time.sleep(1)
            print("Mode changed to GUIDED.")
            
            self.wait_for_disarm()
                
            self.vehicle.mode = VehicleMode("STABILIZE")
            while self.vehicle.mode != 'STABILIZE':
                print(" Waiting for stabilize mode...")
                time.sleep(1)            
        else:
            # After a flight (successful mission flight or guided/landed flight), 
            # the RTL mode would have been commanded and will end up in "DISARMED"
            # Set back to "STABILIZE" to be prepared for the next flight.
            self.wait_for_disarm()
                
            self.vehicle.mode = VehicleMode("STABILIZE")
            while self.vehicle.mode != 'STABILIZE':
                print(" Waiting for stabilize mode...")
                time.sleep(1)
            
            print("Vehicle is now in STABILIZE mode.")
        
        self.send_batreset()
        print("System Battery Power is reset")
        
    def reboot_autopilot(self):
        # Disable the geofence
        self.vehicle.parameters['FENCE_ENABLE'] = 0
        self.vehicle.flush()
        self.vehicle.wait_ready('parameters', timeout=300)
        print("Geofence disabled.")
        
        print("Rebooting autopilot...")
        self.vehicle.reboot()
        print("Reboot command sent. Waiting for reboot...")
        time.sleep(30)  # Wait for reboot
        self.reconnect()
        
        # Ensure GPS fix
        while self.vehicle.gps_0.fix_type < 2:
            print("Waiting for GPS fix...")
            time.sleep(1)
        print("GPS fix acquired")
        
        # Ensure EKF is healthy
        while not self.vehicle.ekf_ok:
            print("Waiting for EKF to be ready...")
            time.sleep(1)
        print("EKF is ready")

        print("Ardupilot system has been rebooted")
    
    def close(self):
        if self.vehicle is not None:
            print("Closing vehicle connection")
            self.vehicle.close()
            print("Vehicle connection closed.")
            self.vehicle = None


def main():
    global controller
    
    def signal_handler(sig, frame):
        print("Interrupt received, shutting down...")
        if controller:
            controller.close()
        sys.exit(0)
    
    # Create the parser
    parser = argparse.ArgumentParser(description="Running Ardupilot Flight Sequence with dronekit API")

    subparsers = parser.add_subparsers(dest='command', required=True)
    
    # Add the arguments
    subparsers.add_parser('disableGPS', help='Disable GPS')
    subparsers.add_parser('enableGPS', help='Enable GPS')
    subparsers.add_parser('reset', help='Reset the system battery')
    subparsers.add_parser('reboot', help='Reboot the Ardupilot simulation')
    subparsers.add_parser('loadFence', help='Load the geofence file (fence.txt) and show fence on map')
    subparsers.add_parser('loadMission', help='Load mission waypoints from file (mission.txt)')

    run_sim_parser = subparsers.add_parser('runSimulation', help='Run the simulation')
    run_sim_group = run_sim_parser.add_mutually_exclusive_group()

    run_sim_group.add_argument('--useWaypoints', action='store_true', help='Use mission waypoints in simulation')
    
    movement_group = run_sim_group.add_argument_group('movement')
    movement_group.add_argument('--vertMovement', type=int, choices=range(-100, 101), metavar='[-100 to 100]',
                        help="Relative Vertical movement")
    movement_group.add_argument('--hortMovement', type=int, choices=range(-100, 101), metavar='[-100 to 100]',
                        help="Relative Horizontal movement")
    
    run_sim_parser.add_argument('--altitude', type=int, choices=range(30, 201), metavar='[30 to 200]',
                        help="Altitude in meters (required)", required=True)
    
    # Parse the arguments
    args = parser.parse_args()

    # Connect to the vehicle (ARDUPILOT SIMULATOR)
    vehicle_connection = 'udp:127.0.0.1:14551'
    print(f"Connecting to vehicle on: {vehicle_connection}")
    
    try:
        controller = DroneController(vehicle_connection)
        controller.connect()
        
        # Set up signal handler after creating controller
        signal.signal(signal.SIGINT, signal_handler)
    
        if args.command == 'disableGPS':
            controller.config_gps_enable_param(False)
        elif args.command == 'enableGPS':
            controller.config_gps_enable_param(True)
            controller.reboot_autopilot()
        elif args.command == 'reset':
            controller.reset_simulation()
        elif args.command == 'reboot':
            controller.reboot_autopilot()
        elif args.command == 'loadFence':
            controller.load_geofence()
        elif args.command == 'loadMission':
            controller.load_mission_waypoints()
        elif args.command == 'runSimulation':
            if args.useWaypoints:
                print(f"Running simulation with Mission Waypoints. Takeoff altitude: {args.altitude} meters")
                #controller.load_sim_params()
                controller.load_mission_waypoints()
                controller.load_geofence()
                controller.run_sim(altitude=args.altitude, use_waypoints=True)
            else:
                print(f"Running simulation with Vertical: {args.vertMovement}, Horizontal: {args.hortMovement}, Altitude: {args.altitude} (in meters) ")
                controller.run_sim(vertMovement=args.vertMovement, hortMovement=args.hortMovement, altitude=args.altitude)
    except Exception as e:
        print(f"An error occurred: {str(e)}")
    finally:    
        if controller:
            controller.close()

# Entry point
if __name__ == "__main__":
    main()
