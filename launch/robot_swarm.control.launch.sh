#!/bin/bash
source /home/kevin/rosbuild_ws/setup.bash
rosrun OPUSim OPUSim_ControlLaw $1 $2 1 1.0 3.0 -1.0 &
