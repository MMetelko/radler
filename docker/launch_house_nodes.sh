#!/bin/bash

source install/local_setup.bash

./install/house_thermo/bin/house &
./install/house_thermo/bin/thermometer &
./install/house_thermo/bin/button &

wait