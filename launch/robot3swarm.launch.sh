#!/bin/bash
source /home/kevin/rosbuild_ws/setup.bash
roslaunch OPUSim robot.launch robot_number:=0 R:=1.0 emergencyBreak:=0 Omega:=2.0 tresholdDistAccount:=0.6 a:=1.0 kv:=0.1 epsilon:=1.0 debugRSO:=0 &
roslaunch OPUSim robot.launch robot_number:=1 R:=1.0 emergencyBreak:=0 Omega:=2.0 tresholdDistAccount:=0.6 a:=1.0 kv:=0.1 epsilon:=1.0 debugRSO:=0 &
roslaunch OPUSim robot.launch robot_number:=2 R:=1.0 emergencyBreak:=0 Omega:=2.0 tresholdDistAccount:=0.6 a:=1.0 kv:=0.1 epsilon:=1.0 debugRSO:=0 &
