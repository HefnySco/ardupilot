#!/bin/bash

# assume we start the script from the root directory
ROOTDIR=$PWD
$PWD/Tools/autotest/sim_vehicle.py -v ArduCopter -w --model webots-quad:127.0.0.1:5577 --add-param-file=$PWD/libraries/SITL/examples/Webots/webots_2022/quadX_2022.parm 
