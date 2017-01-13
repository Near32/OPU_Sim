#!/bin/bash
source /home/kevin/rosbuild_ws/setup.bash
rosrun OPUSim OPUSim_OmniView $1 0 &
rosrun OPUSim OPUSim_RelativeSwarmOdometry $1 0 0 &
rosrun OPUSim OPUSim_Obstacles $1 0 0 &
rosrun OPUSim OPUSim_ControlLaw $1 $2 0 &
