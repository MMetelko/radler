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
        altitude_timeout = 30  # seconds
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
            timeout = 30
            
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
            uploaded_mission = []
            
            cmds = self.vehicle.commands
            cmds.clear()
            cmds.upload()
            
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
                        uploaded_mission.append(cmd)
                            
            cmds.upload()
            # Note: The following warning appears on the console, but it does show "Flight plan received"
            #    Got MISSION_ACK: TYPE_MISSION: ACCEPTED
            #    AP: got MISSION_ITEM; GCS should send MISSION_ITEM_INT
            #    Got MISSION_ACK: TYPE_MISSION: ACCEPTED
            #    AP: Flight plan received
            print(f"Mission uploaded: {cmds.count} waypoints") 
            
            time.sleep(2)
            cmds.download()
            cmds.wait_ready()
            
            # Don't count the home waypoint (first uploaded value)
            num_uploaded_wp = len(uploaded_mission) - 1
            num_cmds_wp = len(cmds)
            # Value chosen based on GPS used by Ardupilot
            epsilon = 1e-3
            if num_cmds_wp == num_uploaded_wp:
                for i in range(num_uploaded_wp):
                    if abs(round(cmds[i].x, 3) - round(uploaded_mission[i + 1].x, 3)) > epsilon or \
                        abs(round(cmds[i].y, 3) - round(uploaded_mission[i + 1].y, 3)) > epsilon or \
                        abs(round(cmds[i].z, 3) - round(uploaded_mission[i + 1].z, 3)) > epsilon:
                        print(f"Mission verification failed: mismatch at waypoint {i}")
                        print(f"Uploaded coordinates (x,y,z): {uploaded_mission[i + 1].x}, {uploaded_mission[i + 1].y}, {uploaded_mission[i + 1].z}")
                        print(f"Cmds coordinates (x,y,z): {cmds[i].x}, {cmds[i].y}, {cmds[i].z}")
                        return False
                print("Mission verified successfully")
                return True
            else:
                print(f"Mission verification failed: waypoint count mismatch.  Uploaded = {num_uploaded_wp}, Found = {num_cmds_wp}")
                return False        
        
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
        
            # 2. Force fence visibility through parameter changes
            # This approach works by toggling parameters to force a fence redraw
            fence_params_to_toggle = ['FENCE_MARGIN', 'FENCE_RADIUS', 'FENCE_ALT_MAX']
            
            for param_name in fence_params_to_toggle:
                if param_name in self.vehicle.parameters:
                    try:
                        # Store original value
                        original_value = float(self.vehicle.parameters[param_name])
                        
                        # Change the parameter slightly to force a redraw
                        new_value = original_value + (1.0 if original_value < 1000 else 5.0)
                        self.vehicle.parameters[param_name] = new_value
                        self.vehicle.flush()
                        time.sleep(0.5)
                        
                        # Restore original value
                        self.vehicle.parameters[param_name] = original_value
                        self.vehicle.flush()
                        time.sleep(0.5)
                        
                        print(f"Toggled {param_name} parameter to trigger fence redraw")
                        break  # Only need to toggle one parameter successfully
                        
                    except Exception as e:
                        print(f"Couldn't toggle {param_name}: {e}")
            
            # 3. Use DO_FENCE_ENABLE command to ensure fence is active and visible
            self.vehicle._master.mav.command_long_send(
                self.target_system,
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE,
                0,  # confirmation
                1,  # param1: enable fence
                0,  # param2: unused
                0, 0, 0, 0, 0  # unused
            )
            self.vehicle.flush()
                                  
            print("Fence visibility commands sent via MAVLink")
            return True
        
        except Exception as e:
            print(f"Error making fence visible: {str(e)}")
            return False
    
    def load_geofence(self):
        try:            
            # First check if fence is already loaded with correct point count
            fence_total = int(self.vehicle.parameters['FENCE_TOTAL'])
            fence_enabled = int(self.vehicle.parameters['FENCE_ENABLE'])
            fence_action = int(self.vehicle.parameters['FENCE_ACTION'])
            
            print(f"Current fence status: TOTAL={fence_total}, ENABLE={fence_enabled}, ACTION={fence_action}")

            # If fence is already properly set up (correct points and parameters)
            if (fence_total > 0 and fence_enabled == 1 and fence_action == 2 and 
                hasattr(self, 'fence_uploaded') and self.fence_uploaded):
                print("Geofence is already properly set up and enabled.")
                return True

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
                #'FS_EKF_ACTION': 2, # Action when EKF variance exceeds threshold: AltHold (altitude hold mode)
                #'FS_EKF_THRESH': 0.600000,
                #'FS_OPTIONS': 16
            }
                
            # Need to upload points if count doesn't match
            upload_points_needed = (fence_total != fence_points_total or fence_total == 0)
            
            # Disable fence before making any changes
            if fence_enabled:
                print("Temporarily disabling fence for upload...")
                self.vehicle.parameters['FENCE_ENABLE'] = 0
                self.vehicle.flush()
                time.sleep(1)
            
            # Set all required parameters
            print("Setting fence parameters...")
            for param, value in fence_params.items():
                print(f"Setting {param} = {value}")
                self.vehicle.parameters[param] = value
                time.sleep(0.2)  # Brief delay for parameter update
            
            self.vehicle.flush()
            time.sleep(1)
    
            # Upload fence points if needed
            if upload_points_needed:            
                print(f"Loading {fence_points_total} fence points...")
                
                # Set FENCE_TOTAL parameter
                self.vehicle.parameters['FENCE_TOTAL'] = fence_points_total
                self.vehicle.flush()
                time.sleep(1)
 
                for i, point in enumerate(points[:-1]):  # Skip the last point if it's a closing point
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
                    time.sleep(0.2)  # Delay to ensure reliable delivery
                                 
                # Verify the fence was uploaded
                actual_total = int(self.vehicle.parameters['FENCE_TOTAL'])
                if actual_total != fence_points_total:
                    print(f"Warning: Fence verification issue: Expected {fence_points_total} points but got {actual_total}")
                else:
                    print(f"Fence upload verified: {actual_total} points")
            else:
                print("Using existing fence points")
                            
            # Enable the fence
            print("Enabling geofence...")
            self.vehicle.parameters['FENCE_ENABLE'] = 1
            self.vehicle.flush()
                
            # Set a parameter to force redraw
            try:
                orig_radius = self.vehicle.parameters['FENCE_RADIUS']
                self.vehicle.parameters['FENCE_RADIUS'] = orig_radius + 1
                self.vehicle.flush()
                time.sleep(0.5)
                self.vehicle.parameters['FENCE_RADIUS'] = orig_radius
                self.vehicle.flush()
            except:
                pass
        
            # Success
            return True                
                
        except Exception as e:
            print(f"Error loading geofence: {str(e)}")
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
                self.vehicle.mode = VehicleMode("LOITER")  # LOITER is a safe mode to transition from AUTO
                time.sleep(2)
        except Exception as e:
            print(f"Mode change error: {e}")

        # 2. Disarm the vehicle if armed
        if self.vehicle.armed:
            print("Disarming vehicle...")
            self.vehicle.armed = False
            start_time = time.time()
            while self.vehicle.armed and time.time() - start_time < 10:
                print("Waiting for disarm...")
                time.sleep(1)

        # 3. Reset mission to first waypoint
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

        # 4. Reset the battery level
        try:
            self.send_batreset()
            print("Battery level reset to 100%")
        except Exception as e:
            print(f"Error resetting battery: {e}")

        # 5. Ensure GPS is enabled
        try:
            self.config_gps_enable_param(True)
            print("GPS enabled")
        except Exception as e:
            print(f"Error enabling GPS: {e}")
        
        # 6. Set to STABILIZE mode (basic, safe mode)
        try:
            print("Switching to STABILIZE mode...")
            self.vehicle.mode = VehicleMode("STABILIZE")
            
            # Wait with reasonable timeout
            start_time = time.time()
            while self.vehicle.mode.name != 'STABILIZE' and time.time() - start_time < 10:
                print(f"Current mode: {self.vehicle.mode.name}, waiting for STABILIZE...")
                time.sleep(1)
                
            if self.vehicle.mode.name == 'STABILIZE':
                print("Successfully entered STABILIZE mode")
            else:
                print(f"Warning: Could not enter STABILIZE mode, current mode is {self.vehicle.mode.name}")
        except Exception as e:
            print(f"Error changing mode: {e}")
    
        # 7. Reset position (only works if SITL supports position reset)
        try:
            # Using SITL-specific MAVLink command to reset position
            # This might not work in all SITL setups
            self.vehicle._master.mav.command_long_send(
                self.target_system,
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                0,  # confirmation
                1,  # Use current position
                0, 0, 0, 0, 0, 0  # unused
            )
            print("Reset home position")
        except Exception as e:
            print(f"Position reset not supported: {e}")
            
        # 8. Restart Radler processes if needed
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
            #controller.make_fence_visible_mavlink()
        elif args.command == 'loadMission':
            controller.load_mission_waypoints()
        elif args.command == 'runSimulation':
            if args.useWaypoints:
                print(f"Running simulation with Mission Waypoints. Takeoff altitude: {args.altitude} meters")
                #controller.load_sim_params()
                waypoint_status = controller.load_mission_waypoints()
                if waypoint_status:
                    controller.load_geofence()
                    #controller.make_fence_visible_mavlink()
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
