#!/usr/bin/python

import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
import time
from math import radians, sin, cos
from pymavlink import mavutil
import csv
import os


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
                print("Reached target altitude")
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
                    while self.vehicle.armed:
                        print(f" Currently on command: {self.vehicle.commands.next}...")
                        time.sleep(1)
                    print("Mission Complete!  Returning to launch...")
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
    def load_mission_waypoints(self):
        try:
            # Clear any existing missions
            cmds = self.vehicle.commands
            cmds.download()
            cmds.wait_ready()
            
            cmds.clear()
            cmds.upload()
            
            mission_waypoint_file_path = os.path.join("/home/ardupilot/radler/examples", "ardupilot", "sitl_config", "mission.txt")
            with open(mission_waypoint_file_path, 'r') as f:
                next(f)  # Skip header row
                for line in f:
                    parts = line.strip().split('\t')
                    if len(parts) == 12:
                        seq, currentwp, frame, command, param1, param2, param3, param4, x, y, z, autocontinue = parts
                        cmd = mavutil.mavlink.MAVLink_mission_item_int_message(
                                    self.target_system, 
                                    mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
                                    int(seq), int(frame), int(command), 
                                    int(currentwp), int(autocontinue),
                                    float(param1), float(param2), float(param3), float(param4),
                                    int(float(x) * 1e7), int(float(y) * 1e7), float(z))
                        cmds.add(cmd)
        
            cmds.upload()
            cmds.wait_ready()
            print(f"Mission uploaded: {cmds.count} waypoints")            
        except Exception as e:
            print(f"Error loading mission: {str(e)}")
    
    # Setup Geofence
    def load_geofence(self):
        try:
            # First setup the desired parameters
            fence_params = {
                'FENCE_ACTION': 1,
                'FENCE_ALT_MAX': 150.0,
                'FENCE_RADIUS': 500.0,
                'FENCE_TOTAL': 8,
                'FENCE_TYPE': 7
            }
            
            for param, value in fence_params.items():
                self.vehicle.parameters[param] = value
                print(f"{param}: {self.vehicle.parameters[param]}")
            
            # Load fence.txt file
            fence_file_path = os.path.join("/home/ardupilot/radler/examples", "ardupilot", "sitl_config", "fence.txt")
            with open(fence_file_path, 'r') as f:
                points = [line.strip().split('\t') for line in f if line.strip()]
            
           # Set fence points
            for i, point in enumerate(points):
                lat, lon = map(float, point)
                msg = self.vehicle.message_factory.fence_point_encode(
                    self.target_system,
                    mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
                    i,
                    len(points),
                    lat,
                    lon
                )
                self.vehicle.send_mavlink(msg)
                self.vehicle.flush()

            print(f"Fence uploaded: {len(points)} points")
            self.vehicle.wait_ready('parameters', timeout=300)

            # Verify fence points
            if self.vehicle.parameters['FENCE_TOTAL'] == len(points):
                print("Geofence successfully uploaded and verified.")
            else:
                print("Geofence upload may have failed. Please verify.")
                
            # Show the fence
            msg = self.vehicle.message_factory.command_long_encode(
                self.target_system, 
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
                mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE,
                0,       # confirmation
                2,       # param1: 2 for show fence
                0, 0, 0, 0, 0, 0
            )  # param2-7 not used
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            print("Geofence made visible.")
            self.vehicle.wait_ready('parameters', timeout=300)
    
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
        # Make sure flight mode is reset
        if self.vehicle.mode == 'AUTO':
            self.emergency_reset()
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
    
    def __del__(self):
        if self.vehicle is not None:
            print("Closing vehicle connection")
            self.vehicle.close()
            print("Vehicle connection closed.")


def main():
    # Create the parser
    parser = argparse.ArgumentParser(description="Running Ardupilot Flight Sequence with dronekit API")

    subparsers = parser.add_subparsers(dest='command', required=True)
    
    # Add the arguments
    subparsers.add_parser('disableGPS', help='Disable GPS')
    subparsers.add_parser('enableGPS', help='Enable GPS')
    subparsers.add_parser('reset', help='Reset the system battery')
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
    
    run_sim_parser.add_argument('--altitude', type=int, choices=range(10, 200), metavar='[10 to 200]',
                        help="Altitude in meters (required)", required=True)
    
    # Parse the arguments
    args = parser.parse_args()

    # Connect to the vehicle (ARDUPILOT SIMULATOR)
    vehicle_connection = 'udp:127.0.0.1:14551'
    print(f"Connecting to vehicle on: {vehicle_connection}")
    
    try:
        controller = DroneController(vehicle_connection)
        controller.connect()
    
        if args.command == 'disableGPS':
            controller.config_gps_enable_param(False)
        elif args.command == 'enableGPS':
            controller.config_gps_enable_param(True)
        elif args.command == 'reset':
            controller.reset_simulation()
            print("System Battery Power is reset")
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
        if 'controller' in locals():
            del controller

# Entry point
if __name__ == "__main__":
    main()
