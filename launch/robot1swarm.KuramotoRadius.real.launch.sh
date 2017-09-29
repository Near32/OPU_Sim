#!/bin/bash
source ~/rosbuild_ws/setup.bash
roslaunch OPUSim robot.real.launch robot_number:=0 R:=2.0 emergencyBreak:=0 Omega:=2.0 tresholdDistAccount:=0.6 a:=1.5 kv:=0.1 kw:=0.2 kR:=10.0 epsilon:=1.0 debugRSO:=0 Iang:=0.4 &
