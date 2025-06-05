#!/bin/bash

# Create parameter file for MAVProxy
mkdir -p /tmp
cat > /tmp/mavproxy_params.txt << 'EOF'
module load console
console link 14777
echo "Console module loaded"
echo "Console link established on port 14777"
EOF

# Make sure the parameter file has correct permissions
chmod 644 /tmp/mavproxy_params.txt

# Use it with sim_vehicle
xterm -T "Vehicle Sim" -geometry "90x30+0+0" -e "sim_vehicle.py -N -v ArduCopter --console --map --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14560 --out=udp:127.0.0.1:14570 --add-param-file=/tmp/mavproxy_params.txt > /tmp/sim_vehicle.log 2>&1"