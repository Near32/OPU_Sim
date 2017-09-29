#!/bin/bash
source ~/rosbuild_ws/setup.bash
roslaunch OPUSim robot.real.launch robot_number:=0 R:=2.0 emergencyBreak:=0 Omega:=2.0 thresholdDistAccount:=0.6 thresholdDistAssoc:=5.0 a:=1.5 kv:=0.1 kw:=0.2 kR:=10.0 epsilon:=1.0 debugRSO:=0 Iang:=0.4 &
