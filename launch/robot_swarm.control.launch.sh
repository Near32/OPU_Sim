#!/bin/bash
source /home/kevin/rosbuild_ws/setup.bash
rosrun OPUSim OPUSim_ControlLaw $1 $2 0 &
