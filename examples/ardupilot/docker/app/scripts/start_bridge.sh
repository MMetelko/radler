#!/bin/bash
# Start the custom MAVLink bridge
echo "Starting MAVLink bridge..."
python3 /home/ardupilot/mavlink_bridge.py > /tmp/bridge.log 2>&1