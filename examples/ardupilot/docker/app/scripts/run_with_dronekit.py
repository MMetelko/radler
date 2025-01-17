from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from math import radians, sin, cos

# Connect to the vehicle (ARDUPILOT SIMULATOR)
print("Connecting to vehicle on: '127.0.0.1:14550'")
vehicle = connect('127.0.0.1:14550', wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms the vehicle and flies to aTargetAltitude.
    """
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != 'GUIDED':
        print(" Waiting for guiding mode...")
        time.sleep(1)

    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def get_location_offset_meters(original_location, dNorth, dEast, altDelta):
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

def goto_position_ned(vehicle, dNorth, dEast, dAlt):
    """
    Move the vehicle to a position `dNorth` and `dEast` meters away from the current position, maintaining altitude change `dAlt`.
    """
    current_location = vehicle.location.global_relative_frame
    target_location = get_location_offset_meters(current_location, dNorth, dEast, dAlt)
    print(f"Moving to position (NORTH: {dNorth}m, EAST: {dEast}m, ALT: {dAlt}m)")
    vehicle.simple_goto(target_location)
    # Adjust time to ensure vehicle reaches the target
    #MM TODO: adjust this value to meet the needs of the radler system.  In other words, make sure the return is due to battery level, not block by this timeout.
    time.sleep(60)

def land_and_wait_for_altitude(vehicle):
    """
    Command the vehicle to land and wait until it reaches an altitude of 0 meters.
    """
    print("Initiating landing...")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.mode != 'LAND':
        print(" Waiting for landing mode...")
        time.sleep(1)

    # Wait until the vehicle reaches an altitude of 0 meters
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt <= 0.1:
            print("Landed. Altitude: ", vehicle.location.global_relative_frame.alt)
            break
        time.sleep(1)


# Main script starts here
try:
    print("Changing to GUIDED mode")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != 'GUIDED':
        print(" Waiting for GUIDED mode...")
        time.sleep(1)

    print("Arming the vehicle")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off to 30 meters altitude")
    arm_and_takeoff(30)

    time.sleep(10)

    print("Flying to relative position NED (100, 100, 0)")
    goto_position_ned(vehicle, 100, 100, 0)

    #MM TODO: this may interfere with Radler battery low actions - checkout later
    #land_and_wait_for_altitude(vehicle)

finally:
    vehicle.close()
    print("Completed vehicle operations.")
