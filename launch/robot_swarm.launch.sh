#!/bin/bash
source /home/kevin/rosbuild_ws/setup.bash
roslaunch OPUSim robot.launch robot_number:=0 R:=0.7 emergencyBreak:=0 Omega:=2.0 tresholdDistAccount:=0.5 a:=1.0 kv:=0.1 epsilon:=10.0 &
roslaunch OPUSim robot.launch robot_number:=1 R:=0.7 emergencyBreak:=0 Omega:=2.0 tresholdDistAccount:=0.5 a:=1.0 kv:=0.1 epsilon:=10.0 &
roslaunch OPUSim robot.launch robot_number:=2 R:=0.7 emergencyBreak:=0 Omega:=2.0 tresholdDistAccount:=0.5 a:=1.0 kv:=0.1 epsilon:=10.0 &
roslaunch OPUSim robot.launch robot_number:=3 R:=0.7 emergencyBreak:=0 Omega:=2.0 tresholdDistAccount:=0.5 a:=1.0 kv:=0.1 epsilon:=10.0 &
roslaunch OPUSim robot.launch robot_number:=4 R:=0.7 emergencyBreak:=0 Omega:=2.0 tresholdDistAccount:=0.5 a:=1.0 kv:=0.1 epsilon:=10.0 &
