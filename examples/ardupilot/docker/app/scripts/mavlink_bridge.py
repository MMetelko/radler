#!/usr/bin/env python3
# mavlink_bridge.py

import socket
import threading
import sys
import time

def main():
    # Define endpoints - these are the destinations we're forwarding TO
    endpoint1 = ('127.0.0.1', 14551)  # Dronekit side
    endpoint2 = ('127.0.0.1', 14550)  # MAVProxy/SITL side

    # Create UDP sockets
    sock1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Configure sockets
    sock1.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock2.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    # Bind sockets to different ports to avoid conflicts
    try:
        sock1.bind(('0.0.0.0', 14560))  # Use different port to avoid conflict
        print("Bound to port 14560 for endpoint 1")
    except Exception as e:
        print(f"Could not bind to port 14560: {e}. Using random port.")
        sock1.bind(('0.0.0.0', 0))
        print(f"Using port {sock1.getsockname()[1]} instead")
        
    try:
        sock2.bind(('0.0.0.0', 14570))  # Use different port to avoid conflict
        print("Bound to port 14570 for endpoint 2")
    except Exception as e:
        print(f"Could not bind to port 14570: {e}. Using random port.")
        sock2.bind(('0.0.0.0', 0))
        print(f"Using port {sock2.getsockname()[1]} instead")

    print(f"Starting MAVLink bridge: {endpoint1} <--> {endpoint2}")
    print(f"Bridge listening on: {sock1.getsockname()} and {sock2.getsockname()}")

    # Create threads for each direction
    threading.Thread(target=forward_packets, args=(sock1, endpoint2, "DRONEKIT->SITL"), daemon=True).start()
    threading.Thread(target=forward_packets, args=(sock2, endpoint1, "SITL->DRONEKIT"), daemon=True).start()

    try:
        # Keep main thread alive
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Bridge terminated by user")
    finally:
        sock1.close()
        sock2.close()

def forward_packets(sock_in, endpoint_out, label):
    try:
        print(f"Starting forwarding {label}: {sock_in.getsockname()} -> {endpoint_out}")
        count = 0
        while True:
            # Receive packet
            data, addr = sock_in.recvfrom(65535)
            
            # Forward to other endpoint
            sock_in.sendto(data, endpoint_out)
            
            count += 1
            if count % 100 == 0:  # Log every 100 packets
                print(f"{label}: Forwarded {count} packets")
    except Exception as e:
        print(f"Error in {label} forwarding: {e}")

if __name__ == "__main__":
    main()