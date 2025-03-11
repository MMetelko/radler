#!/usr/bin/python

import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
import time
from math import radians, sin, cos
from pymavlink import mavutil


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

    def run_sim(self, vertMovement, hortMovement, altitude):
        # Main script starts here
        try:
            print(f"Taking off to indicated altitude of {altitude} (in meters)")
            self.arm_and_takeoff(altitude)

            time.sleep(10)

            #print(f"Flying to relative position: North/South = {vertMovement}, East/West = {hortMovement}, Altitute Change = 0)")
            self.goto_position_ned(vertMovement, hortMovement, 0)

            #MM TODO: this may interfere with Radler battery low actions - checkout later
            #self.land_and_wait_for_altitude()
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            
        finally:
            print("Completed vehicle operations.")
    
    # Function to enable or disable the GPS
    def config_gps_enable_param(self):
        #MM TODO: determine command for SIM_GPS_TYPE, below is battery for an example
        #"""
        #Send a custom MAVLink command to turn on and off GPS system in the simulation.
        #"""
        #msg = self.vehicle.message_factory.command_long_encode(
        #    self.target_system, 
        #    mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
        #    mavutil.mavlink.MAV_CMD_BATTERY_RESET, # command
        #    0,    # confirmation
        #    -1, 100, 0, 0, 0, 0, 0  
        #)
        
        #self.vehicle.send_mavlink(msg)
        #self.vehicle.flush()
        #print("Battery reset command sent.")
        pass
    
    # Load simulation parameters
    def load_sim_params(self):
        #with open(filename, 'r') as f:
        #    for line in f:
        #        if line.startswith('#'):
        #            continue
        #        param, value = line.strip().split()
        #        self.vehicle.parameters[param] = float(value)
        #    print("Parameters loaded.  Waiting for them to take effect...")
        #    self.vehicle.wait_ready('parameters', timeout=300)
        pass
    
    # Upload Mission Waypoints
    def load_mission_waypoints(self):
        # Clear any existing missions
        #cmds = self.vehicle.commands
        #cmds.clear()
        #cmds.upload()

        # Define mission waypoints
        #waypoints = [
        #    (latitude1, longitude1, altitude1),
        #    (latitude2, longitude2, altitude2),
            # Add more waypoints as needed
        #]

        # Add waypoints to the mission
        #for i, wp in enumerate(waypoints):
        #    cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, wp[0], wp[1], wp[2])
        #    cmds.add(cmd)

        # Upload mission
        #cmds.upload()
        pass
    
    # Setup Geofence
    def load_geofence(self):
        # read fence parameters from file
        #fence_points = [
        #    (latitude1, longitude1),
        #    (latitude2, longitude2),
        #]
        #
        #for point in fence_points:
        #    self.vehicle.message_factory.send_mavlink(self.vehicle.message_factory.command_long_encode(
        #        0, 0,
        #        mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE,
        #        0,
        #        0, 0, 0, 0,
        #        point[0], point[1], 0
        #    ))
        #self.vehicle.parameters['FENCE_ENABLE'] = 1
        
        pass
    
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
        
    #MM TODO: need to add code to return to base (what the previous radler low battery code would do)
    # Return to takeoff point, then reset battery
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
    parser.add_argument('--runSimulation', action='store_true',
                        help="Run the simulation")

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
        controller.config_gps_enable_param()
    
    # Process the arguments
    if args.reset:
        controller.reset_simulation()
        print("System Battery Power is reset")
        return

    if args.runSimulation:
        print("Running simulation with:")
        print(f"Vertical Movement: {args.vertMovement}")
        print(f"Horizontal Movement: {args.hortMovement}")
        print(f"Altitude: {args.altitude} meters")
        controller.run_sim(args.vertMovement, args.hortMovement, args.altitude)
              
    del controller

# Entry point
if __name__ == "__main__":
    main()
