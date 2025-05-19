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
import subprocess

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
        mission_items = []

        try:
            # Read waypoints from file
            with open(mission_file_path, 'r') as f:
                next(f)  # Skip header row
                for i, line in enumerate(f):
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
                        
            if not mission_items:
                print("No valid mission items found in file")
                return False
            
            print(f"Read {len(mission_items)} mission items from file")
        
            # Start mission upload process
            print("Starting mission upload with INT commands")
            self.vehicle._master.mav.mission_count_send(
                target_system,
                target_component,
                len(mission_items)
            )
            
            timeout_start = time.time()
            timeout_sec = 30
            
            # Wait for the first request
            while True:
                if time.time() - timeout_start > timeout_sec:
                    print("Timeout waiting for initial mission request")
                    return False
                    
                msg = self.vehicle._master.recv_match(
                    type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'],
                    blocking=True,
                    timeout=1
                )
                
                if msg is not None:
                    break
                
                # Send count again if no response
                if time.time() - timeout_start > 5:
                    print("Resending mission count")
                    self.vehicle._master.mav.mission_count_send(
                        target_system,
                        target_component,
                        len(mission_items)
                    )
                        
            # Process all item requests        
            for i in range(len(mission_items)):
                # Wait for request for item
                msg = self.vehicle._master.recv_match(
                    type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'],
                    blocking=True,
                    timeout=10
                )
            
                if msg is None:
                    print(f"No mission request received for item {i}")
                    return False
                
                req_seq = msg.seq
                if req_seq >= len(mission_items):
                    print(f"Received request for item {req_seq}, but only have {len(mission_items)} items")
                    return False
    
                item = mission_items[req_seq]
            
                # If we got a MISSION_REQUEST, use the older command format
                if msg.get_type() == 'MISSION_REQUEST':
                    print(f"Warning: Got MISSION_REQUEST instead of MISSION_REQUEST_INT for item {req_seq}")
                    self.vehicle._master.mav.mission_item_send(
                        target_system,
                        target_component,
                        req_seq,
                        item['frame'],
                        item['command'],
                        item['current'],
                        item['autocontinue'],
                        item['param1'],
                        item['param2'],
                        item['param3'],
                        item['param4'],
                        item['x'],
                        item['y'],
                        item['z']
                    )
                else:
                    # Use the newer INT format
                    self.vehicle._master.mav.mission_item_int_send(
                        target_system,
                        target_component,
                        req_seq,
                        item['frame'],
                        item['command'],
                        item['current'],
                        item['autocontinue'],
                        item['param1'],
                        item['param2'],
                        item['param3'],
                        item['param4'],
                        int(item['x'] * 1e7),  # Convert to int (lat)
                        int(item['y'] * 1e7),  # Convert to int (lon)
                        item['z']  # Altitude remains as float
                    )
        
            # Wait for mission ack
            msg = self.vehicle._master.recv_match(
                type='MISSION_ACK',
                blocking=True,
                timeout=10
            )
        
            if msg is None:
                print("No mission acknowledgment received")
                return False
                
            if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print(f"Mission upload failed with error: {msg.type}")
                return False
                
            print(f"Mission successfully uploaded ({len(mission_items)} items)")
            return True
 
        except Exception as e:
            print(f"Error uploading mission: {str(e)}")
            return False
        
    def load_mission_waypoints(self):
        try:
            mission_waypoint_file_path = os.path.join("/home/ardupilot/radler/examples", "ardupilot", "sitl_config", "mission.txt")
            
            # Clear any existing mission
            cmds = self.vehicle.commands
            cmds.clear()
            cmds.upload()
            
            # Upload the new mission using INT messages
            success = self.upload_mission_with_int(mission_waypoint_file_path)
            
            if not success:
                print("Failed to upload mission")
                return False
            
            # Verify the mission
            cmds.download()
            cmds.wait_ready()
            
            print(f"Mission downloaded: {cmds.count} waypoints") 
            
            if cmds.count == 0:
                print("Mission verification failed: no waypoints found after download")
                return False
                
            print("Mission verified successfully")
            return True
            
        except Exception as e:
            print(f"Unexpected mission error: {str(e)}")
            return False

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
            child.expect('MAV>', timeout=60)
            print(f"MAVProxy command completed.")

            # Close the MAVProxy process
            #time.sleep(1)
            child.close()
            print("Closed mavproxy communication link.")
        except Exception as e:
            print(f"Unexpected mavproxy communication error: {str(e)}")
  
    def make_fence_visible_mavlink(self):
        """Make fence visible using direct MAVLink commands"""
        try:
            print("Making fence visible via MAVLink commands...")
            
            # 1. Ensure fence is enabled
            self.vehicle.parameters['FENCE_ENABLE'] = 1
            self.vehicle.flush()
            time.sleep(0.5)
        
            # 2. Set fence display parameters
            # This parameter doesn't exist in regular ArduPilot, but MAVProxy might use it
            try:
                self.vehicle.parameters['FENCE_DISPLAY'] = 1
            except:
                print("FENCE_DISPLAY parameter not available (this is normal)")
            
            # 3. Send fence message request to all GCS
            self.vehicle._master.mav.command_long_send(
                self.target_system,
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                0,  # confirmation
                mavutil.mavlink.MAVLINK_MSG_ID_FENCE_STATUS,
                0, 0, 0, 0, 0, 0
            )
            
            # 4. Re-upload fence boundary to ensure it's sent to all displays
            current_total = int(self.vehicle.parameters['FENCE_TOTAL'])
            print(f"Re-triggering fence transmission for {current_total} points")
        
            # Change and restore FENCE_TOTAL to trigger fence re-send
            if current_total > 0:
                self.vehicle.parameters['FENCE_TOTAL'] = 0
                self.vehicle.flush()
                time.sleep(0.5)
                self.vehicle.parameters['FENCE_TOTAL'] = current_total
                self.vehicle.flush()
                time.sleep(0.5)
            
            print("Fence visibility commands sent via MAVLink")
            return True
        except Exception as e:
            print(f"Error making fence visible: {str(e)}")
            return False
    
    def load_geofence(self):
        try:            
            # Load fence.txt file
            fence_file_path = os.path.join("/home/ardupilot/radler/examples", "ardupilot", "sitl_config", "fence.txt")
            with open(fence_file_path, 'r') as f:
                points = [line.strip().split('\t') for line in f if line.strip()]

            # Calculate total number of points (excluding any closing point)
            fence_points_total = len(points) - 1  # Don't count the last point if it's a duplicate

            # First setup the desired parameters
            # Note: FENCE_ACTION=2 is report only, set to 1 for Guided mode, 0 is None (disabled fenced mode)
            # Include fail safe settings that could effect the fence actions
            #   Failsafe options: Bit 4 (16): Continue if in auto mission on GCS failsafe
            #                     Bit 1 (2): Continue if in auto mode on RC failsafe
            #                     Bit 0 (1): Clear flight mode actions from fence breach
            fence_params = {
                'FENCE_ACTION': 0,
                'FENCE_ALT_MAX': 150.0,
                'FENCE_RADIUS': 500.0,
                'FENCE_OPTIONS': 1,
                'FENCE_TOTAL': fence_points_total,
                'FENCE_TYPE': 7,
                'FS_EKF_ACTION': 2, # Action when EKF variance exceeds threshold: AltHold (altitude hold mode)
                'FS_EKF_THRESH': 0.600000,
                'FS_GCS_ENABLE': 2,  # Ground station communication failsafe action: Enable Continue with Mission in AUTO Mode (only applies to AUTO mode)
                'FS_OPTIONS': 19  # failsafe options: try 19 first, then 18
            }
            
            # First check if fence has already been loaded
            fence_total = self.vehicle.parameters['FENCE_TOTAL']
            print(f"Current fence_total is {fence_total}.")
            
            if (fence_total > 0) and (fence_params['FENCE_TOTAL'] == fence_total):
                print("The geofence is already loaded with correct point count.")
            else:
                print(f"Loading {fence_points_total} fence points...")
                for param, value in fence_params.items():
                    print(f"Setting {param} to {value}")
                    self.vehicle.parameters[param] = value
                    time.sleep(0.1)
                    
                self.vehicle.flush()
                
                # Wait for parameters to be ready
                print("Waiting for parameters to be ready...")
                self.vehicle.wait_ready('parameters', timeout=30)
                                
                # Disable fence while uploading points
                print("Temporarily disabling fence for upload...")
                self.vehicle.parameters['FENCE_ENABLE'] = 0
                self.vehicle.flush()
                time.sleep(1)
                
                # Clear any existing fence first
                self.clear_fence()
               
                # Upload fence points using the newer command format
                print(f"Uploading {fence_points_total} fence points...")
                for i, point in enumerate(points[:-1]):  # Skip the last point if it's a closing point
                    lat, lon = map(float, point)
                    success = self.upload_fence_point_int(i, fence_points_total, lat, lon)
                    if not success:
                        print(f"Failed to upload fence point {i}")
                        return False
                    time.sleep(0.2)  # Delay between points to avoid overwhelming the system
            
                # Verify fence points were correctly uploaded
                print("Verifying fence upload...")
                if self.verify_fence(fence_points_total) == False:
                    print("Fence verification failed!")
                    return False
                    
                print(f"Fence uploaded: {fence_points_total} points")
            
            # Enable the fence
            print("Enabling geofence...")
            self.vehicle.parameters['FENCE_ENABLE'] = 1
            self.vehicle.flush()
            self.vehicle.wait_ready('parameters', timeout=10)
        
            # Verify fence is enabled
            if self.vehicle.parameters['FENCE_ENABLE'] == 1:
                print("Geofence successfully enabled.")
            else:
                print("Warning: Failed to enable geofence!")
                return False
                
            # Success
            return True                
                
        except Exception as e:
            print(f"Error loading geofence: {str(e)}")
            return False                

    def clear_fence(self):
        """Clear all fence points from the vehicle"""
        print("Clearing existing fence points...")
        
        # Send command to clear all fence points
        result = self.vehicle._master.mav.command_long_send(
            self.target_system,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE,
            0,  # Confirmation
            0,  # Disable fence
            0,  # Clear fence
            0, 0, 0, 0, 0  # Unused parameters
        )
    
        self.vehicle.flush()
        time.sleep(1)  # Give time for clearing
        
        # Check fence count is zero
        if self.vehicle.parameters['FENCE_TOTAL'] != 0:
            print(f"Warning: Failed to clear fence, FENCE_TOTAL = {self.vehicle.parameters['FENCE_TOTAL']}")
            return False
            
        return True                                
                                
    def upload_fence_point_int(self, idx, count, lat, lon):
        """
        Upload a fence point using the newer MAVLink command format
        
        Args:
            idx: Point index (0-based)
            count: Total number of points
            lat: Latitude in decimal degrees
            lon: Longitude in decimal degrees
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            # Convert to integers (multiplied by 10^7 for increased precision)
            lat_int = int(lat * 1e7)
            lon_int = int(lon * 1e7)
            
            print(f"Uploading fence point {idx+1}/{count}: lat={lat}, lon={lon}")
            
            # Use MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION for inclusion polygons
            command = mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
        
            # Send fence point as a command_int message
            self.vehicle._master.mav.command_int_send(
                self.target_system,  # target_system
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,  # target_component
                mavutil.mavlink.MAV_FRAME_GLOBAL,  # frame
                command,  # command
                0,  # current - not used with this command
                0,  # autocontinue - not used with this command
                count,  # param1 - total number of vertices
                0,  # param2 - not used
                0,  # param3 - not used
                idx,  # param4 - vertex index (0-based)
                lat_int,  # x (latitude) in 1e7 degrees
                lon_int,  # y (longitude) in 1e7 degrees
                0  # z - not used
            )
        
            # Wait briefly to ensure command is processed
            time.sleep(0.1)
            self.vehicle.flush()
            
            return True
        
        except Exception as e:
            print(f"Error uploading fence point: {str(e)}")
            return False

    def verify_fence(self, expected_count):
        """
        Verify the fence was uploaded correctly
        
        Args:
            expected_count: Expected number of fence points
            
        Returns:
            bool: True if fence was verified, False otherwise
        """
        try:
            # Check if the FENCE_TOTAL parameter matches what we expect
            fence_total = self.vehicle.parameters['FENCE_TOTAL']
            print(f"Fence verification: FENCE_TOTAL={fence_total}, expected={expected_count}")
            
            if fence_total != expected_count:
                print(f"Fence verification failed: FENCE_TOTAL mismatch")
                return False
                
            # We can't easily verify individual points with the newer API
            # but we can check other fence parameters
            fence_type = self.vehicle.parameters['FENCE_TYPE']
            fence_action = self.vehicle.parameters['FENCE_ACTION']
            
            print(f"Fence parameters: TYPE={fence_type}, ACTION={fence_action}")
            
            return True
        
        except Exception as e:
            print(f"Error verifying fence: {str(e)}")
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
        
    def wait_for_disarm(self):
        while self.vehicle.armed:
            print(" Waiting for disarmed mode...")
            time.sleep(1)
        print("Vehicle is disarmed")

    def emergency_reset(self):
        # self.vehicle.mode = VehicleMode("GUIDED")
        # while self.vehicle.mode != 'GUIDED':
        #     print(" Waiting for guiding mode...")
        #     time.sleep(1)

        # self.vehicle.mode = VehicleMode("RTL")
        # while self.vehicle.mode != 'RTL':
        #     print(" Waiting for RTL mode...")
        #     time.sleep(1)
            
        # self.wait_for_disarm()
        
        # self.vehicle.mode = VehicleMode("STABILIZE")
        # while self.vehicle.mode != 'STABILIZE':
        #     print(" Waiting for stabilize mode...")
        #     time.sleep(1)
        print("Emergency reset complete")
       
    def restart_radler_code(self):
        subprocess.run(["pkill", "-f", "afs_function"])
        subprocess.run(["pkill", "-f", "afs_gateway"])

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
            
        self.send_batreset()
        print("System Battery Power is reset")
        
        self.config_gps_enable_param(True)
        print("Set GPS to enabled") 
                     
        self.vehicle.mode = VehicleMode("STABILIZE")
        while self.vehicle.mode != 'STABILIZE':
            print(" Waiting for stabilize mode...")
            time.sleep(1)            
            
        print("Vehicle is now in STABILIZE mode.")
        # delay while the Radler functions restart
        time.sleep(10)    
              
        self.restart_radler_code() 
        self.reboot_autopilot()
        
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
            controller.make_fence_visible_mavlink()
        elif args.command == 'loadMission':
            controller.load_mission_waypoints()
        elif args.command == 'runSimulation':
            if args.useWaypoints:
                print(f"Running simulation with Mission Waypoints. Takeoff altitude: {args.altitude} meters")
                #controller.load_sim_params()
                waypoint_status = controller.load_mission_waypoints()
                if waypoint_status:
                    controller.load_geofence()
                    controller.make_fence_visible_mavlink()
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
