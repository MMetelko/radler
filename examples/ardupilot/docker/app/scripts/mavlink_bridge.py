#!/usr/bin/env python3
# mavlink_bridge.py

import socket
import threading
import sys
import time

def main():
    # Define endpoints
    endpoint1 = ('127.0.0.1', 14551)  # Dronekit side
    endpoint2 = ('127.0.0.1', 14550)  # MAVProxy/SITL side

    # Create UDP sockets
    sock1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Bind sockets
    sock1.bind(('0.0.0.0', 14551))
    sock2.bind(('0.0.0.0', 14550))

    print(f"Starting MAVLink bridge: {endpoint1} <--> {endpoint2}")

    # Create threads for each direction
    threading.Thread(target=forward_packets, args=(sock1, endpoint2, "1->2"), daemon=True).start()
    threading.Thread(target=forward_packets, args=(sock2, endpoint1, "2->1"), daemon=True).start()

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