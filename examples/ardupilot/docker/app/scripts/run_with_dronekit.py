#!/usr/bin/python

import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
import time
from math import radians, cos
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega
import os
import signal
import sys
import subprocess
import socket
import traceback
import threading


controller = None

class DroneController:
    def __init__(self, connection_str):
        self.connection_str = connection_str
        self.vehicle = None
        self.bridge_thread = None
        self.bridge_process = None
        self.fence_uploaded = False
        
        # Store the fixed launch location (from the mission file's first waypoint)
        self.original_launch_location = LocationGlobal(
            -35.3632624,  # Fixed latitude from the first waypoint
            149.1652375,  # Fixed longitude from the first waypoint
            584.08        # Fixed altitude from the first waypoint
        )
        
    def connect(self):
        self.start_mavlink_bridge()
        
        self.vehicle = connect(self.connection_str, wait_ready=True, timeout=60)
        self.target_system = self.vehicle._master.target_system
        print(f"Connected to the vehicle with target system ID = {self.target_system}.")
        self.setup_listeners()
        
        print(f"Original launch location stored: {self.original_launch_location.lat}, "
              f"{self.original_launch_location.lon}, {self.original_launch_location.alt}")

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
    
    def start_mavlink_bridge(self):        
        def run_bridge():
            try:
                bridge_script = "/app-novnc/scripts/mavlink_bridge.py"
                log_file = open("/tmp/bridge.log", "w")
                self.bridge_process = subprocess.Popen(
                    ['python3', bridge_script],
                    stdout=log_file,
                    stderr=log_file,
                    text=True
                )
                print(f"Started MAVLink bridge with PID {self.bridge_process.pid}")
                self.bridge_process.wait()
            except Exception as e:
                print(f"Bridge process error: {e}")
        
        # Start bridge in background thread
        self.bridge_thread = threading.Thread(target=run_bridge, daemon=True)
        self.bridge_thread.start()
        print("Started MAVLink bridge in background")
        
        # Give it a moment to initialize
        time.sleep(2)
       
    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms the vehicle and flies to aTargetAltitude.
        """
        print("Basic pre-arm checks")
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialize...")
            time.sleep(1)

        print("Changing to GUIDED mode...")
        
        # Try to switch to GUIDED mode with timeout and mode-cycling
        guided_mode_attempts = 0
        max_guided_attempts = 3
        guided_timeout = 5  # seconds
        
        while guided_mode_attempts < max_guided_attempts:
            guided_mode_attempts += 1
            self.vehicle.mode = VehicleMode("GUIDED")
            
            # Wait for specified timeout for GUIDED mode
            start_time = time.time()
            while self.vehicle.mode != 'GUIDED':
                if time.time() - start_time > guided_timeout:
                    print(f" GUIDED mode timeout (attempt {guided_mode_attempts}/{max_guided_attempts})")
                    break            
                print(" Waiting for guiding mode...")
                time.sleep(1)
                
            # If we successfully entered GUIDED mode, break the loop
            if self.vehicle.mode == 'GUIDED':
                print("Successfully entered GUIDED mode")
                break

            # If we're still stuck, try cycling through STABILIZE mode first
            print("Trying to cycle through STABILIZE mode...")
            self.vehicle.mode = VehicleMode("STABILIZE")
            time.sleep(2)  # Give it time to change modes
            
        # If we still couldn't enter GUIDED mode after all attempts, abort
        if self.vehicle.mode != 'GUIDED':
            print("Failed to enter GUIDED mode after multiple attempts. Aborting takeoff.")
            return False

        print("Arming motors...")
        self.vehicle.armed = True
        
        # Wait for arming with timeout
        arm_timeout = 10  # seconds
        start_time = time.time()
        while not self.vehicle.armed:
            if time.time() - start_time > arm_timeout:
                print("Arming timeout. Aborting takeoff.")
                return False
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude)

        # Wait for altitude with timeout
        altitude_timeout = 180  # seconds
        start_time = time.time()
        while True:
            current_altitude = self.vehicle.location.global_relative_frame.alt
            print(f" Altitude: {current_altitude}")
            
            if current_altitude >= aTargetAltitude * 0.95:
                print("Reached target altitude.")
                return True
            elif self.vehicle.mode == 'RTL':
                print("A fail-safe mechanism changed the vehicle mode to RTL.")
                return False
            elif time.time() - start_time > altitude_timeout:
                print("Altitude timeout. Vehicle did not reach target altitude.")
                return False
            
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
            takeoff_success = self.arm_and_takeoff(altitude)
            
            if not takeoff_success:
                print("Takeoff failed. Aborting mission.")
                return False

            if use_waypoints:
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

            # Don't use, this may interfere with Radler battery low actions - checkout later
            #self.land_and_wait_for_altitude()
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            
        finally:
            print("Completed vehicle operations.")
    
    # Function to enable or disable the GPS
    def config_gps_enable_param(self, gps_state):
        try:
            if gps_state:
                self.vehicle.parameters['GPS1_TYPE'] = 1
                print("Setting GPS1_TYPE to 1 (enabled)")
            else:
                self.vehicle.parameters['GPS1_TYPE'] = 0   
                print("Setting GPS1_TYPE to 0 (disabled)")         

            # Wait for parameter change to take effect
            self.vehicle.flush()
            self.vehicle.wait_ready('parameters', timeout=5)
            
            # For enabling GPS, we need to force EKF to recognize GPS
            if gps_state:
                # Send EKF action to accept GPS
                action_msg = self.vehicle.message_factory.command_long_encode(
                    self.target_system,
                    mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    0,  # confirmation
                    0, 0, 0, 0, 0, 0, 0  # Reset EKF modes
                )
                self.vehicle.send_mavlink(action_msg)
                self.vehicle.flush()
                print("Sent command to reset EKF")
                time.sleep(2)  # Give time for the command to process
                
                # Check GPS lock status
                gps_tries = 10
                while gps_tries > 0:
                    if self.vehicle.gps_0.fix_type >= 3:
                        print(f"GPS has fix. Type: {self.vehicle.gps_0.fix_type}")
                        break
                    print(f"Waiting for GPS fix. Current: {self.vehicle.gps_0.fix_type}")
                    time.sleep(1)
                    gps_tries -= 1
                
            print(f"GPS1_TYPE parameter now: {self.vehicle.parameters['GPS1_TYPE']}")
            return True
        except Exception as e:
            print(f"Error setting GPS parameter: {str(e)}")
            return False
           
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
   
    def upload_mission_with_int(self, mission_file_path):
        target_system = self.vehicle._master.target_system
        target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1

        try:
            mission_items = []
            # Read waypoints from file
            with open(mission_file_path, 'r') as f:
                next(f)  # Skip header row
                for line in f:
                    parts = line.strip().split('\t')
                    if len(parts) == 12:
                        seq = int(parts[0])
                        current = int(parts[1])
                        frame = int(parts[2])
                        command = int(parts[3])
                        param1 = float(parts[4])
                        param2 = float(parts[5])
                        param3 = float(parts[6])
                        param4 = float(parts[7])
                        x = float(parts[8])
                        y = float(parts[9])
                        z = float(parts[10])
                        autocontinue = int(parts[11].strip())
                        
                        mission_items.append({
                            'seq': seq,
                            'current': current,
                            'frame': frame,
                            'command': command,
                            'param1': param1,
                            'param2': param2,
                            'param3': param3,
                            'param4': param4,
                            'x': x,
                            'y': y,
                            'z': z,
                            'autocontinue': autocontinue
                        }) 
            
            print(f"Read {len(mission_items)} mission items from file")
            if not mission_items:
                print("No valid mission items found")
                return False
    
            # Start mission upload process
            print("Starting mission upload with INT commands")
            self.vehicle._master.mav.mission_count_send(
                target_system,
                target_component,
                len(mission_items)
            )
            
            start_time = time.time()
            timeout = 60
            
            # Process all waypoints
            uploaded_waypoints = 0

            while uploaded_waypoints < len(mission_items):
                if time.time() - start_time > timeout:
                    print(f"Mission upload timed out after {timeout}s")
                    return False
                    
                msg = self.vehicle._master.recv_match(
                    type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'],
                    blocking=True,
                    timeout=3
                )
                
                if msg is None:
                    # If no response after 1 second, re-send mission count
                    if (time.time() - start_time) % 6 < 0.5:  # Re-send every ~6 seconds
                        print("Re-sending mission count...")
                        self.vehicle._master.mav.mission_count_send(
                            target_system,
                            target_component,
                            len(mission_items)
                        )
                        time.sleep(0.5)
                    continue
                
                # Got a request - process it
                req_seq = msg.seq
                if req_seq >= len(mission_items):
                    print(f"Requested item {req_seq} but we only have {len(mission_items)} items")
                    return False
                    
                print(f"Got request for mission item {req_seq}")
                wp = mission_items[req_seq]
        
                # Send the mission item using the appropriate format
                if msg.get_type() == 'MISSION_REQUEST_INT':
                    self.vehicle._master.mav.mission_item_int_send(
                        target_system,
                        target_component,
                        req_seq,
                        wp['frame'],
                        wp['command'],
                        wp['current'],
                        wp['autocontinue'],
                        wp['param1'],
                        wp['param2'],
                        wp['param3'],
                        wp['param4'],
                        int(wp['x'] * 10000000),  # Convert to int (lat)
                        int(wp['y'] * 10000000),  # Convert to int (lon)
                        wp['z']  # Altitude remains as float
                    )
                else:
                    # Fall back to non-INT version
                    self.vehicle._master.mav.mission_item_send(
                        target_system,
                        target_component,
                        req_seq,
                        wp['frame'],
                        wp['command'],
                        wp['current'],
                        wp['autocontinue'],
                        wp['param1'],
                        wp['param2'],
                        wp['param3'],
                        wp['param4'],
                        wp['x'],
                        wp['y'],
                        wp['z']
                    )
                    
                uploaded_waypoints += 1
                time.sleep(0.2)
                                
            # Wait for mission acceptance
            print("Waiting for mission acceptance...")
            ack_msg = self.vehicle._master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
            if not ack_msg:
                print("No mission acceptance received")
                return False

            if ack_msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print(f"Mission not accepted: {ack_msg.type}")
                return False
            
            # Verify by downloading
            print("Verifying mission...")
            cmds = self.vehicle.commands
            cmds.download()
            cmds.wait_ready(timeout=30)
            
            print(f"Mission verification: Found {cmds.count} waypoints")
            return cmds.count > 0
            
        except Exception as e:
            print(f"Error uploading mission: {str(e)}")
            return False
                
    def load_mission_waypoints(self):
        try:            
            mission_waypoint_file_path = os.path.join("/home/ardupilot/radler/examples", "ardupilot", "sitl_config", "mission.txt")
            
            # Clear any existing mission
            self.vehicle.commands.clear()
            self.vehicle.commands.upload()
            time.sleep(2)  # Give time for clear to process
            
            # Connect directly to the vehicle's MAVLink connection
            master = self.vehicle._master
            
            # Read waypoints from mission file
            waypoints = []
            with open(mission_waypoint_file_path, 'r') as f:
                next(f)  # Skip header row
                for line in f:
                    parts = line.strip().split('\t')
                    if len(parts) == 12:
                        waypoints.append({
                            'seq': int(parts[0]),
                            'frame': int(parts[2]),
                            'command': int(parts[3]),
                            'current': int(parts[1]),
                            'autocontinue': int(parts[11].strip()),
                            'param1': float(parts[4]),
                            'param2': float(parts[5]),
                            'param3': float(parts[6]),
                            'param4': float(parts[7]),
                            'x': float(parts[8]),
                            'y': float(parts[9]),
                            'z': float(parts[10])
                        })
            
            if not waypoints:
                print("No waypoints found in mission file")
                return False
                
            # Start mission upload - send count first
            print(f"Uploading {len(waypoints)} waypoints...")
            
            # Send mission count command
            master.mav.mission_count_send(
                master.target_system,
                master.target_component,
                len(waypoints),
                0  # Mission type: 0 = mission
            )
            
            # Wait for mission request
            ack_received = False
            for i in range(30):  # Timeout after 30 seconds
                msg = master.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK'], blocking=True, timeout=1)
                if msg is not None:
                    print(f"Received {msg.get_type()} message: {msg}")
                    if msg.get_type() == 'MISSION_REQUEST_INT' or msg.get_type() == 'MISSION_REQUEST':
                        seq = msg.seq
                        if seq < len(waypoints):
                            wp = waypoints[seq]
                        
                            # Always use mission_item_int_send for better precision
                            print(f"Sending waypoint {seq}: command={wp['command']}, lat={wp['x']}, lon={wp['y']}, alt={wp['z']}")
                        
                            master.mav.mission_item_int_send(
                                master.target_system,
                                master.target_component,
                                seq,
                                wp['frame'],
                                wp['command'],
                                wp['current'],
                                wp['autocontinue'],
                                wp['param1'],
                                wp['param2'],
                                wp['param3'],
                                wp['param4'],
                                int(wp['x'] * 1e7),  # lat * 10^7
                                int(wp['y'] * 1e7),  # lon * 10^7
                                wp['z']
                            )
                        else:
                            print(f"WARNING: Requested waypoint {seq} but only have {len(waypoints)} waypoints")
                    elif msg.get_type() == 'MISSION_ACK':
                        if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                            print("Mission accepted!")
                            ack_received = True
                            break
                        else:
                            print(f"Mission upload failed with error: {msg.type}")
                            return False
                        
                # Also check for general "Flight plan received" indication in another way
                vehicle_msgs = master.recv_match(type='STATUSTEXT', blocking=False)
                if vehicle_msgs and "Flight plan received" in vehicle_msgs.text:
                    print("Detected 'Flight plan received' in status text")
                    ack_received = True
                    break
                
            # Even if we didn't get an explicit ACK, try to verify by downloading
            # This is because ArduPilot sometimes doesn't send a proper ACK even when successful
            print("Downloading mission to verify upload...")
            self.vehicle.commands.download()
            
            # Wait for the download to complete with timeout
            download_timeout = 30  # seconds
            start_time = time.time()
            while not self.vehicle.commands.wait_ready(timeout=1):
                if time.time() - start_time > download_timeout:
                    print("Timeout waiting for mission download")
                    if ack_received:
                        print("But mission ACK was received, proceeding anyway")
                        return True
                    return False
            
            # If we got here, mission download completed
            if self.vehicle.commands.count > 0:
                print(f"Mission verified: {self.vehicle.commands.count} waypoints found")
                return True
            else:
                print("Mission verification failed: No waypoints found after upload")
                # Last resort check - if we received ACK but commands.count is 0, something is wrong
                # with Dronekit's tracking, not necessarily with the upload
                if ack_received:
                    print("But mission ACK was received, proceeding anyway")
                    return True
                return False
            
        except Exception as e:
            print(f"Unexpected mission error: {str(e)}")
            import traceback
            traceback.print_exc()
            return False
    
    def show_fence_via_mavlink(self):
        """Make fence visible through existing vehicle connection"""
        try:
            print("Making fence visible through existing vehicle connection...")
            
            # Get current fence parameters
            fence_total = int(self.vehicle.parameters.get('FENCE_TOTAL', 0))
            
            if fence_total > 0:
                print(f"Making fence with {fence_total} points visible")
                
                # Approach 1: Toggle FENCE_ENABLE parameter (this has been working reliably)
                current_enable = int(self.vehicle.parameters.get('FENCE_ENABLE', 0))
                print(f"Toggling fence visibility (current state: {current_enable})")
                
                # Disable fence briefly
                self.vehicle.parameters['FENCE_ENABLE'] = 0
                self.vehicle.flush()
                time.sleep(0.5)
            
                # Re-enable fence
                self.vehicle.parameters['FENCE_ENABLE'] = 1
                self.vehicle.flush()
                time.sleep(0.5)
                
                # Approach 2: Send a DO_FENCE_ENABLE command (modern approach)
                msg = self.vehicle.message_factory.command_long_encode(
                    self.target_system,  # target system
                    mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,  # target component
                    mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE,  # command
                    0,  # confirmation
                    1,  # enable
                    0, 0, 0, 0, 0, 0  # params 2-7 (not used)
                )
                self.vehicle.send_mavlink(msg)
                self.vehicle.flush()
                
                print("Sent fence visibility commands via existing vehicle connection")
                return True
            else:
                print("No fence points to display")
                return False
                
        except Exception as e:
            print(f"Error making fence visible via MAVLink: {e}")
            traceback.print_exc()
            return False
    
    def load_geofence(self):
        try:            
            # First check if fence is already loaded with correct point count
            fence_total = int(self.vehicle.parameters['FENCE_TOTAL'])
            fence_enabled = int(self.vehicle.parameters['FENCE_ENABLE'])
            fence_action = int(self.vehicle.parameters['FENCE_ACTION'])
            fence_type = int(self.vehicle.parameters['FENCE_TYPE'])
            
            print(f"Current fence status: TOTAL={fence_total}, ENABLE={fence_enabled}, TYPE={fence_type}, ACTION={fence_action}")

            # If fence is already properly set up (correct points and parameters)
            if (fence_total >= 3 and fence_enabled == 1 and (fence_type & 7) == 7 and 
                hasattr(self, 'fence_uploaded') and self.fence_uploaded):
                print("Geofence is already properly set up and enabled.")
                
                # Send show commands to make the fence visible even if already loaded
                self.show_fence_on_map()
                return True
            
            # COMPLETELY disable fence before making ANY changes
            print("Completely disabling fence before upload...")
            self.vehicle.parameters['FENCE_ENABLE'] = 0
            self.vehicle.parameters['FENCE_TYPE'] = 0  # Disable all fence types
            self.vehicle.flush()
            time.sleep(2)  # Longer delay to ensure fence is fully disabled
            
            # CRITICAL: Clear existing fence points by setting total to 0
            print("Clearing existing fence points...")
            self.vehicle.parameters['FENCE_TOTAL'] = 0
            self.vehicle.flush()
            time.sleep(2)  # Wait for fence to be cleared
            
            # Load fence.txt file
            fence_file_path = os.path.join("/home/ardupilot/radler/examples", "ardupilot", "sitl_config", "fence.txt")
            with open(fence_file_path, 'r') as f:
                points = [line.strip().split('\t') for line in f if line.strip()]

            # Check if last point is identical to first point, if not add it
            first_point = points[0]
            last_point = points[-1]
            if first_point != last_point:
                points.append(first_point)
                print("Added closing point to make a complete polygon")
            
            fence_points_total = len(points)
            
            # Check if we have enough points
            if fence_points_total < 3:
                print(f"ERROR: Fence file contains only {fence_points_total} points. At least 3 are required.")
                return False

            # First set total points (CRITICAL: must be done before uploading any points)
            self.vehicle.parameters['FENCE_TOTAL'] = fence_points_total
            self.vehicle.flush()
            time.sleep(2)  # Give autopilot time to allocate memory for points
            
            # Upload points one by one using the fence_point_encode method
            for i, point in enumerate(points):
                lat, lon = map(float, point)
                print(f"Uploading point {i+1}/{fence_points_total}: {lat}, {lon}")
                
                msg = self.vehicle.message_factory.fence_point_encode(
                    self.target_system,
                    mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                    i,  # point index
                    fence_points_total,  # total fence points
                    lat,
                    lon
                )
                self.vehicle.send_mavlink(msg)
                self.vehicle.flush()
                time.sleep(0.5)  # Longer delay for reliable delivery
            
            # Wait for points to be processed
            time.sleep(3)
                   
            # Now set the fence parameters
            print("Setting final fence parameters...")
            fence_params = {
                'FENCE_TYPE': 7,  # Enable polygon fence (bit 1=2) plus others
                'FENCE_ACTION': 2,  # Report only
                'FENCE_ALT_MAX': 150.0,
                'FENCE_RADIUS': 500.0,
                'FENCE_MARGIN': 10.0,  # Add margin parameter
            }
        
            for param, value in fence_params.items():
                print(f"Setting {param} = {value}")
                self.vehicle.parameters[param] = value
                self.vehicle.flush()
                time.sleep(0.5)
                        
            # Finally enable the fence
            print("Enabling geofence...")
            self.vehicle.parameters['FENCE_ENABLE'] = 1
            self.vehicle.flush()
            time.sleep(2)
            
            # Try to make fence visible on MAP
            self.show_fence_via_mavlink()
         
            # Set flag to indicate fence is uploaded
            self.fence_uploaded = True
            
            # Success
            return True                
                
        except Exception as e:
            print(f"Error loading geofence: {str(e)}")
            traceback.print_exc()
            return False

    def show_fence_on_map(self):
        """Use multiple methods to ensure fence is visible on map"""
        try:
            print("Attempting to make fence visible on the map...")
            
            # Connect to MAVProxy console
            console_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            console_connection.settimeout(5)
            
            try:
                console_connection.connect(('127.0.0.1', 14777))
                print("Connected to MAVProxy console")
                
                # Try multiple commands with clear feedback
                commands = [
                    "fence list",
                    "fence draw",
                    "fence show",
                    "map",  # Refresh map
                    "param show FENCE_ENABLE",  # Verify fence is enabled
                    "param show FENCE_TYPE",    # Verify fence type
                    "param show FENCE_TOTAL"    # Verify fence point count
                ]
                        
                for cmd in commands:
                    print(f"Sending MAVProxy command: {cmd}")
                    console_connection.sendall(f"{cmd}\n".encode())
                    time.sleep(0.5)  # Give time for command to process
                
                console_connection.close()
                print("Successfully sent fence visualization commands")
                return True
                
            except Exception as e:
                print(f"Socket connection failed: {e}")
                
                # Fallback: Try to draw the fence directly
                self.draw_fence_on_map()
                return False
            
        except Exception as e:
            print(f"Error showing fence on map: {str(e)}")
            traceback.print_exc()
            return False

    def draw_fence_on_map(self):
        """Draw fence polygon directly on the map as fallback"""
        try:
            # Connect to MAVProxy console
            console_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            console_connection.settimeout(5)
            console_connection.connect(('127.0.0.1', 14777))
            
            # Load fence points
            fence_file_path = os.path.join("/home/ardupilot/radler/examples", "ardupilot", "sitl_config", "fence.txt")
            with open(fence_file_path, 'r') as f:
                points = [line.strip().split('\t') for line in f if line.strip()]
            
            # Create commands to draw lines between all points
            print("Drawing fence lines directly on map...")
            
            for i in range(len(points)-1):
                lat1, lon1 = map(float, points[i])
                lat2, lon2 = map(float, points[i+1])
                cmd = f"map line {lat1} {lon1} {lat2} {lon2} red\n"
                console_connection.sendall(cmd.encode())
                time.sleep(0.2)

            # Close the loop
            lat1, lon1 = map(float, points[-1])
            lat2, lon2 = map(float, points[0])
            cmd = f"map line {lat1} {lon1} {lat2} {lon2} red\n"
            console_connection.sendall(cmd.encode())
            
            console_connection.close()
            print("Manually drew fence on map")
            return True
        except Exception as e:
            print(f"Error drawing fence on map: {str(e)}")
            traceback.print_exc()
            return False         
                                
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
       
    def restart_radler_code(self):
        subprocess.run(["pkill", "-f", "afs_function"])
        subprocess.run(["pkill", "-f", "afs_gateway"])
        
    def reset_vehicle_position(self):
        """Reset vehicle position using MAVLink"""
        try:
            print("Resetting vehicle position via MAVLink...")

            # Already in GUIDED mode from reset_simulation, but check anyway
            if self.vehicle.mode.name != "GUIDED":
                print(f"Changing from {self.vehicle.mode.name} to GUIDED mode for position commands...")
                self.vehicle.mode = VehicleMode("GUIDED")
                time.sleep(0.5)
                    
            # Get coordinates from original launch location
            fixed_lat = self.original_launch_location.lat
            fixed_lon = self.original_launch_location.lon
            fixed_alt = self.original_launch_location.alt
            
            print(f"Resetting to position: {fixed_lat}, {fixed_lon}, {fixed_alt}")
            
            # Step 1: Send SET_HOME_POSITION command to reset home
            msg = self.vehicle.message_factory.command_long_encode(
                self.target_system,  # target_system
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,  # target_component
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,  # command
                0,  # confirmation
                0,  # Set home to specified location (not current location)
                0, 0, 0,  # unused
                fixed_lat,
                fixed_lon,
                fixed_alt
            )
            # Send command
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            print(f"Home position reset to original coordinates: {fixed_lat}, {fixed_lon}, {fixed_alt}")
            time.sleep(0.5)
                
            # 2. Reset EKF origin - critical for local position calculations
            try:
                # Try to access the constant first to see if it exists
                origin_cmd = getattr(mavutil.mavlink, 'MAV_CMD_SET_GPS_GLOBAL_ORIGIN', None)
                if origin_cmd is not None:
                    msg = self.vehicle.message_factory.command_long_encode(
                        self.target_system,
                        mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                        origin_cmd,
                        0,  # confirmation
                        self.target_system,  # System ID
                        0, 0,  # unused
                        int(fixed_lat * 1e7),  # latitude (degrees * 1e7)
                        int(fixed_lon * 1e7),  # longitude (degrees * 1e7)
                        int(fixed_alt * 1000)   # altitude (mm)
                    )
                    self.vehicle.send_mavlink(msg)
                    self.vehicle.flush()
                    print("EKF origin reset via MAVLink")
            except Exception:
                # Silently continue if command not available
                pass
            
            # 3. ArduPilot SITL-specific position reset command
            try:
                msg = self.vehicle.message_factory.command_long_encode(
                    self.target_system,
                    mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                    42501,  # MAV_CMD_DO_SITL_SET_POSITION (ArduPilot specific)
                    0,  # confirmation
                    fixed_lat,
                    fixed_lon,
                    fixed_alt,
                    0, 0, 0, 0  # unused
                )
                self.vehicle.send_mavlink(msg)
                self.vehicle.flush()
                print("Sent ArduPilot SITL position reset command")
                time.sleep(0.5) 
            except Exception as e:
                # Continue if this specific command is not available
                print(f"SITL position command not supported: {e}")   
             
            # 4. Set position target using SET_POSITION_TARGET_GLOBAL_INT
            lat_int = int(fixed_lat * 1e7)
            lon_int = int(fixed_lon * 1e7)
            alt_int = int(fixed_alt * 1000)  # mm
            
            msg = self.vehicle.message_factory.set_position_target_global_int_encode(
                0,  # time_boot_ms
                self.target_system,
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,  # coordinate frame
                0b0000111111111000,  # type_mask (only use position)
                lat_int,  # lat_int
                lon_int,  # lon_int
                alt_int,  # alt
                0, 0, 0,  # velocity
                0, 0, 0,  # acceleration
                0, 0      # yaw, yaw_rate
            )
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            print("Position target sent via MAVLink")
            time.sleep(0.5)
        
            # 5. Verify position update
            current_pos = self.vehicle.location.global_frame
            print(f"Position after reset: {current_pos.lat}, {current_pos.lon}, {current_pos.alt}")
            print(f"Target position was: {fixed_lat}, {fixed_lon}, {fixed_alt}")
            
            return True
            
        except Exception as e:
            print(f"Failed to reset position via MAVLink: {e}")
            traceback.print_exc()
            return False       

    def reset_rc_channels(self):
        """Reset RC channels to neutral/minimum values with verification"""
        try:
            print("Resetting RC channels to neutral positions...")
            
            # Method 1: Using rc_channels_override_send
            self.vehicle._master.mav.rc_channels_override_send(
                self.vehicle._master.target_system,
                self.vehicle._master.target_component,
                1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500
            )
            self.vehicle.flush()
            time.sleep(1)
            
            # Verify Method 1
            if self.verify_rc_channels():
                print("RC channels reset successful with Method 1")
                return True
                
            print("Method 1 didn't fully reset RC channels, trying Method 2...")
            
            # Method 2: Using command_long approach
            msg = self.vehicle.message_factory.command_long_encode(
                self.target_system,
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                mavutil.mavlink.MAV_CMD_DO_RC_OVERRIDE,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            time.sleep(1)
            
            # Verify Method 2
            if self.verify_rc_channels():
                print("RC channels reset successful with Method 2")
                return True
                
            print("Method 2 didn't fully reset RC channels, trying Method 3...")
        
            # Method 3: Clear and then set specific values
            self.vehicle._master.mav.rc_channels_override_send(
                self.vehicle._master.target_system,
                self.vehicle._master.target_component,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            self.vehicle.flush()
            time.sleep(1)
            
            self.vehicle._master.mav.rc_channels_override_send(
                self.vehicle._master.target_system,
                self.vehicle._master.target_component,
                1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500
            )
            self.vehicle.flush()
            time.sleep(1)
            
            # Final verification
            if self.verify_rc_channels():
                print("RC channels reset successful with Method 3")
                return True
            
            print("Warning: All RC channel reset methods attempted without full success")
            return False
            
        except Exception as e:
            print(f"Error resetting RC channels: {e}")
            return False
        
    def verify_rc_channels(self):
        """Verify that RC channels are at neutral/expected values"""
        try:
            # Request RC channels reading
            self.vehicle._master.mav.request_data_stream_send(
                self.vehicle._master.target_system,
                self.vehicle._master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
                10,  # 10 Hz
                1    # Start
            )
            
            # Wait for RC_CHANNELS message
            msg = self.vehicle._master.recv_match(
                type='RC_CHANNELS', 
                blocking=True, 
                timeout=2
            )
            
            if msg is None:
                print("Could not get RC channel readings")
                return False
                
            # Check if channels are at expected values (with some tolerance)
            roll_ok = abs(msg.chan1_raw - 1500) < 50
            pitch_ok = abs(msg.chan2_raw - 1500) < 50
            throttle_ok = abs(msg.chan3_raw - 1000) < 50
            yaw_ok = abs(msg.chan4_raw - 1500) < 50
            
            print(f"RC status: Roll={msg.chan1_raw} ({roll_ok}), "
                f"Pitch={msg.chan2_raw} ({pitch_ok}), "
                f"Throttle={msg.chan3_raw} ({throttle_ok}), "
                f"Yaw={msg.chan4_raw} ({yaw_ok})")
            
            # Return true only if all channels are at expected values
            return roll_ok and pitch_ok and throttle_ok and yaw_ok
        
        except Exception as e:
            print(f"Error verifying RC channels: {e}")
            return False
    
    # Reset battery
    def reset_simulation(self):
        """
        Reset simulator to initial state without rebooting the autopilot
        """
        print("Starting simulator reset...")
        
        # 1. First, cancel any ongoing mission and return to a stable state
        try:
            # If we're in AUTO mode, exit to a safe mode first
            if self.vehicle.mode.name == 'AUTO':
                print("Exiting AUTO mode...")
                self.vehicle.mode = VehicleMode("GUIDED")  # Use GUIDED for upcoming position commands
                time.sleep(1)
        except Exception as e:
            print(f"Mode change error: {e}")
            
        # 2. Reset RC channels more aggressively by calling multiple times
        print("Resetting RC channels...")
        success = self.reset_rc_channels()
        
        # If the first attempt didn't fully succeed, try once more
        if not success:
            print("First RC reset attempt didn't fully succeed, trying again...")
            time.sleep(2)  # Give system time to settle
            self.reset_rc_channels()

        # 3. Disarm the vehicle if armed
        if self.vehicle.armed:
            print("Disarming vehicle...")
            self.vehicle.armed = False
            start_time = time.time()
            while self.vehicle.armed and time.time() - start_time < 10:
                print("Waiting for disarm...")
                time.sleep(1)

        # 4. Reset vehicle position 
        if not self.vehicle.armed:
            # Reset position using direct SITL interface
            self.reset_vehicle_position()           

        # 5. Reset mission to first waypoint
        try:
            msg = self.vehicle.message_factory.mission_set_current_encode(
                self.target_system, 
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                0  # seq (first waypoint)
            )
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            print("Mission reset to first waypoint")
        except Exception as e:
            print(f"Error resetting mission: {e}")

        # 6. Reset the battery level
        try:
            self.send_batreset()
            print("Battery level reset to 100%")
        except Exception as e:
            print(f"Error resetting battery: {e}")

        # 7. Ensure GPS is enabled
        try:
            # First disable then re-enable GPS to force a clean state
            self.config_gps_enable_param(False)
            time.sleep(1)
            self.config_gps_enable_param(True)
            print("GPS enabled")
        except Exception as e:
            print(f"Error enabling GPS: {e}")
        
        # 8. Set to STABILIZE mode (basic, safe mode)
        try:
            print("Switching to STABILIZE mode...")
            self.vehicle.mode = VehicleMode("STABILIZE")
            
            # Wait with reasonable timeout
            start_time = time.time()
            while self.vehicle.mode.name != 'STABILIZE' and time.time() - start_time < 30:
                print(f"Current mode: {self.vehicle.mode.name}, waiting for STABILIZE...")
                time.sleep(1)
                
            if self.vehicle.mode.name == 'STABILIZE':
                print("Successfully entered STABILIZE mode")
            else:
                print(f"Warning: Could not enter STABILIZE mode, current mode is {self.vehicle.mode.name}")
        except Exception as e:
            print(f"Error changing mode: {e}")
    
        # 9. Restart Radler processes if needed
        try:
            self.restart_radler_code()
            print("Radler processes restarted")
        except Exception as e:
            print(f"Error restarting Radler code: {e}")

        # Wait for system to stabilize
        print("Waiting for system to stabilize...")
        time.sleep(5)
        
        # Verify system state
        print(f"Reset complete. Current status: Mode={self.vehicle.mode.name}, Armed={self.vehicle.armed}, Battery={self.vehicle.battery.level}%")
        return True
        
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
            
        # Terminate bridge process if it exists
        if self.bridge_process:
            try:
                print("Terminating bridge process...")
                self.bridge_process.terminate()
                # Optional: Wait briefly to ensure termination
                import time
                time.sleep(0.5)
                
                # If it's still running, try to kill it
                if self.bridge_process.poll() is None:
                    self.bridge_process.kill()
                    print("Killed bridge process")
            except Exception as e:
                print(f"Error terminating bridge process: {e}")
                # Fallback: Try to kill any bridge processes
                try:
                    import subprocess
                    subprocess.call(['pkill', '-f', 'mavlink_bridge.py'])
                    print("Terminated all bridge processes")
                except Exception as e2:
                    print(f"Error during process cleanup: {e2}")
        
        # The bridge_thread is daemon=True so Python won't wait for it to finish
        # when the main program exits
        print("Cleanup complete")    


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
        # Wait a moment for bridge to initialize
        time.sleep(2)
        
        controller.connect()
        
        # Set up signal handler after creating controller
        signal.signal(signal.SIGINT, signal_handler)
    
        if args.command == 'disableGPS':
            try:
                controller.config_gps_enable_param(False)
                print("GPS disabled successfully")
            except Exception as e:
                print(f"Error disabling GPS: {str(e)}")
            finally:
                print("GPS disable operation completed")
                
        elif args.command == 'enableGPS':
            try:
                controller.config_gps_enable_param(True)
                print("GPS enabled successfully")
            except Exception as e:
                print(f"Error enabling GPS: {str(e)}")
            finally:
                print("GPS enable operation completed")
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
                waypoint_status = controller.load_mission_waypoints()
                if waypoint_status:
                    controller.load_geofence()
                    controller.run_sim(altitude=args.altitude, use_waypoints=True)
                else:
                    print(f"ABORTED MISSION! ")
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
