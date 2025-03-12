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
        self.vehicle = connect(self.connection_str, wait_ready=True)
        self.target_system = self.vehicle._master.target_system
        print(f"Connected to the vehicle with target system ID = {self.target_system}.")
        
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

    def goto_position_ned(self, dNorth, dEast, dAlt):
        """
        Move the vehicle to a position `dNorth` and `dEast` meters away from the current position, maintaining altitude change `dAlt`.
        """
        current_location = self.vehicle.location.global_relative_frame
        target_location = self.get_location_offset_meters(current_location, dNorth, dEast, dAlt)
        print(f"Moving to relative position (NORTH: {dNorth}m, EAST: {dEast}m, ALT: {dAlt}m)")
        self.vehicle.simple_goto(target_location)
        # Adjust time to ensure vehicle reaches the target
        #MM TODO: adjust this value to meet the needs of the radler system.  In other words, make sure the return is due to battery level, not block by this timeout.
        time.sleep(60)

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

    def run_sim(self, vertMovement, hortMovement, altitude, fixedMission):
        # Main script starts here
        try:
            print(f"Taking off to indicated altitude of {altitude} (in meters)")
            self.arm_and_takeoff(altitude)

            time.sleep(10)

            if fixedMission:
                print("Changing to AUTO mode...")
                self.vehicle.mode = VehicleMode("AUTO")
                while self.vehicle.mode != 'AUTO':
                    print(" Waiting for auto mode...")
                    time.sleep(1)
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
        # MM TODO: another idea of how to see gps setting
        # Access GPS information
        # gps = vehicle.gps_0

        # print(f"GPS: {gps}")
        # print(f"Fix type: {gps.fix_type}")
        # print(f"Num satellites: {gps.satellites_visible}")
        # print(f"Latitude: {gps.lat}")
        # print(f"Longitude: {gps.lon}")
        # print(f"Altitude: {gps.alt}")

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
        # Clear any existing missions
        cmds = self.vehicle.commands
        cmds.clear()
        cmds.upload()
        
        mission_waypoint_file_path = os.path.join("/home/ardupilot/radler/examples", "ardupilot", "sitl_config", "mission.txt")
        with open(mission_waypoint_file_path, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # Skip header row
            for row in reader:
                lat, lon, alt = map(float, row[:3])
                cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                            lat, lon, alt)
                cmds.add(cmd)
        cmds.upload()
        print(f"Mission uploaded: {cmds.count} waypoints")
        self.vehicle.wait_ready('parameters', timeout=300)
    
    # Setup Geofence
    def load_geofence(self):
        fence_file_path = os.path.join("/home/ardupilot/radler/examples", "ardupilot", "sitl_config", "fence.txt")
        with open(fence_file_path, 'r') as f:
            points = [line.strip().split(',') for line in f if line.strip()]
        
        # Send fence point count
        self.vehicle.message_factory.fence_point_count_send(0, 0, len(points))
        
        # Send fence points
        for i, point in enumerate(points):
            lat, lon = map(float, point)
            self.vehicle.message_factory.fence_point_send(0, 0, i, lat, lon)
        
        print(f"Fence uploaded: {len(points)} points")
        self.vehicle.wait_ready('parameters', timeout=300)
        
        #MM TODO: if the above does not work, try this mavlink setup
        #for point in fence_points:
        #    self.vehicle.message_factory.send_mavlink(self.vehicle.message_factory.command_long_encode(
        #        0, 0,
        #        mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE,
        #        0,
        #        0, 0, 0, 0,
        #        point[0], point[1], 0
        #    ))
        #self.vehicle.parameters['FENCE_ENABLE'] = 1
           
    # Function to send custom MAVLink battery reset command
    def send_batreset(self):
        """
        Send a custom MAVLink command to reset the battery state in the simulation.
        """
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
        
    # Reset battery
    def reset_simulation(self):
        self.send_batreset()
    
    
    def __del__(self):
        if self.vehicle is not None:
            print("Closing vehicle connection")
            self.vehicle.close()
            print("Vehicle connection closed.")


def main():
    # Create the parser
    parser = argparse.ArgumentParser(description="Running Ardupilot Flight Sequence with dronekit API")

    # Add the arguments
    parser.add_argument('--vertMovement', type=int, choices=range(-100, 101), default=100, metavar='[-100 to 100]',
                        help="Relative Vertical movement, default is 100")
    parser.add_argument('--hortMovement', type=int, choices=range(-100, 101), default=100, metavar='[-100 to 100]',
                        help="Relative Horizontal movement, default is 100")
    parser.add_argument('--altitude', type=int, choices=range(10, 200), default=30, metavar='[10 to 200]',
                        help="Altitude in meters, default of 30")
    parser.add_argument('--disableGPS', action='store_true', help='Disable GPS')
    parser.add_argument('--enableGPS', action='store_true', help='Enable GPS')
    parser.add_argument('--reset', action='store_true',
                        help="Reset the system battery")

    subparsers = parser.add_subparsers(dest='command')
    run_sim_parser = subparsers.add_parser('--runSimulation', action='store_true',
                        help="Run the simulation")
    run_sim_parser.add_argument('--useWaypoints', action='store_true', help='Use mission waypoints in simulation')

    # Parse the arguments
    args = parser.parse_args()

    # Connect to the vehicle (ARDUPILOT SIMULATOR)
    print("Connecting to vehicle on: '127.0.0.1:14551'")
    
    #MM TODO: vehicle = connect('127.0.0.1:14550', wait_ready=True)
    controller = DroneController('127.0.0.1:14551')
    controller.connect()
    
    gps_status_changed = False
    
    if args.disableGPS:
        gps_enable = False
        gps_status_changed = True
    elif args.enableGPS:
        gps_enable = True
        gps_status_changed = True
    else:
        gps_enable = True
        gps_status_changed = False
        
    if gps_status_changed:
        controller.config_gps_enable_param(gps_enable)
    
    # Process the arguments
    if args.reset:
        controller.reset_simulation()
        print("System Battery Power is reset")
        return

    if args.runSimulation:
        fixedMission = False
        if args.useWaypoints:
            print("Running simulation with Mission Waypoints:")
            print(f"Altitude: {args.altitude} meters")
            fixedMission = True
            #controller.load_sim_params()
            controller.load_mission_waypoints()
            controller.load_geofence()
        else:
            print("Running simulation with:")
            print(f"Vertical Movement: {args.vertMovement}")
            print(f"Horizontal Movement: {args.hortMovement}")
            print(f"Altitude: {args.altitude} meters")
            
        controller.run_sim(args.vertMovement, args.hortMovement, args.altitude, fixedMission)
              
    del controller

# Entry point
if __name__ == "__main__":
    main()
