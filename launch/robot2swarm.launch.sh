#!/bin/bash
source /home/kevin/rosbuild_ws/setup.bash
roslaunch OPUSim world.spawn.launch &
roslaunch OPUSim robot.spawn.launch robot_number:=0 X:=0.0 Y:=2.0 &
roslaunch OPUSim robot.spawn.launch robot_number:=1	X:=0.5 Y:=-1.5 &
roslaunch OPUSim obstacle.spawn.launch obstacle_number:=0	X:=2.2 Y:=0.0 &
roslaunch OPUSim obstacle.spawn.launch obstacle_number:=1	X:=-2.8 Y:=0.0 &
roslaunch OPUSim robot.launch robot_number:=0 R:=2.0 emergencyBreak:=0 Omega:=2.0 tresholdDistAccount:=0.6 a:=0.5 kv:=0.1 kw:=0.2 kR:=10.0 epsilon:=1.0 debugRSO:=0 Iang:=0.4 &
roslaunch OPUSim robot.launch robot_number:=1 R:=2.0 emergencyBreak:=0 Omega:=2.0 tresholdDistAccount:=0.6 a:=0.5 kv:=0.1 kw:=0.2 kR:=10.0 epsilon:=1.0 debugRSO:=0 Iang:=0.4 &
rosservice call /gazebo/unpause_physics &
