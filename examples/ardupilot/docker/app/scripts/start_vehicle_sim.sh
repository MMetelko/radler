#!/bin/bash
sleep 2
xterm -T "Vehicle Sim" -geometry "90x24+0+500" -e "sim_vehicle.py -N -v ArduCopter --console --map --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 --cmd=\"module load console; console link 14777\" > /tmp/sim_vehicle.log 2>&1"
