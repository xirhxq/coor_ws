#!/bin/bash
gnome-terminal --tab -- bash -c "ros2 launch mbzirc_ros competition_local.launch.py ign_args:=\" -v 4 -r coast.sdf\" ;exec bash" 
gnome-terminal --tab -- bash -c "ros2 run ros_ign_bridge my_bridge; exec bash"
echo "Type anything to spawn UAVs..."
read a
for i in "$@"; do
	xxx=`expr 0 - $i \* 2 - 1490`
	gnome-terminal --tab -- bash -c "sleep 2; ros2 launch mbzirc_ign spawn.launch.py name:=buav_$i world:=coast model:=mbzirc_quadrotor x:=${xxx} y:=0 z:=4.3 R:=0 P:=0 Y:=0 slot0:=mbzirc_hd_camera slot0_rpy:=\"0 20 0\" gripper:=mbzirc_suction_gripper flightTime:=60; exec bash"
done
echo "Type anything to start control..."
read b
# gnome-terminal --tab -- bash -c "ros2 run base com_manager"
for i in "$@"; do
	gnome-terminal --window -- bash -c "ros2 run base bUAV ${i}"
done
