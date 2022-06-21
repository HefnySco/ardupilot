#!/bin/bash

# assume we start the script from the root directory
ROOTDIR=$PWD
$PWD/Tools/autotest/sim_vehicle.py -v ArduCopter -w --model webots-tri --add-param-file=$PWD/libraries/SITL/examples/Webots/webots_2022/tricopter_2022.parm  
