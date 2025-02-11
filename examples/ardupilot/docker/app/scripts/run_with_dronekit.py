#!/usr/bin/python

import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from math import radians, sin, cos
from pymavlink import mavutil


class DroneController:
    def __init__(self, connection_str):
        self.connection_str = connection_str
        self.vehicle = None
    
    def connect(self):
        self.vehicle = connect(self.connection_str, wait_ready=True)
        print("Connected to the vehicle.")
        
    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms the vehicle and flies to aTargetAltitude.
        """
        print("Basic pre-arm checks")
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialize...")
            time.sleep(1)

        print("Arming motors")
        self.vehicle.mode = VehicleMode("GUIDED")
        while self.vehicle.mode != 'GUIDED':
            print(" Waiting for guiding mode...")
            time.sleep(1)

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
        print(f"Moving to position (NORTH: {dNorth}m, EAST: {dEast}m, ALT: {dAlt}m)")
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
            print("Changing to GUIDED mode")
            self.vehicle.mode = VehicleMode("GUIDED")
            while self.vehicle.mode != 'GUIDED':
                print(" Waiting for GUIDED mode...")
                time.sleep(1)

            print("Arming the vehicle")
            self.vehicle.armed = True
            while not self.vehicle.armed:
                print(" Waiting for arming...")
                time.sleep(1)

            print(f"Taking off to indicated altitude of {altitude} (in meters)")
            self.arm_and_takeoff(altitude)

            time.sleep(10)

            print(f"Flying to relative position: North/South = {vertMovement}, East/West = {hortMovement}, Altitute Change = 0)")
            self.goto_position_ned(vertMovement, hortMovement, 0)

            #MM TODO: this may interfere with Radler battery low actions - checkout later
            #self.land_and_wait_for_altitude()
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            
        finally:
            print("Completed vehicle operations.")
    
    # Function to send custom MAVLink battery reset command
    def send_batreset(self):
        """
        Send a custom MAVLink command to reset the battery state in the simulation.
        """
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0, # target_system, target_component
            mavutil.mavlink.MAV_CMD_BATTERY_RESET, # command
            0,    # confirmation
            0, 0, 0, 0, 0, 0, 0  
        )
        
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        print("Battery reset command sent.")
    
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
    parser.add_argument('--altitude', type=int, choices=range(30, 51), default=30, metavar='[30 to 50]',
                        help="Altitude in meters, default of 30")
    parser.add_argument('--reset', action='store_true',
                        help="Reset the system battery")
    parser.add_argument('--runSimulation', action='store_true',
                        help="Run the simulation")

    # Parse the arguments
    args = parser.parse_args()

    # Connect to the vehicle (ARDUPILOT SIMULATOR)
    print("Connecting to vehicle on: '127.0.0.1:14550'")
    
    #MM TODO: vehicle = connect('127.0.0.1:14550', wait_ready=True)
    controller = DroneController('127.0.0.1:14550')
    controller.connect()
    
    # Process the arguments
    if args.reset:
        controller.send_batreset()
        print("System Battery Power is reset")
        return

    if args.runSimulation:
        print("Running simulation with:")
        print(f"Vertical Movement: {args.vertMovement}")
        print(f"Horizontal Movement: {args.hortMovement}")
        print(f"Altitude: {args.altitude} meters")
        controller.run_sim(args.vertMovement, args.hortMovement, args.altitude)
        
    #MM TODO: for debugging
    # Print the currently available flight modes
    #print("Supported modes: ", vehicle.mode_mapping())

# Entry point
if __name__ == "__main__":
    main()









