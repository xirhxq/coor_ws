#!/bin/bash
gnome-terminal --tab -- bash -c "ros2 launch mbzirc_ros competition_local.launch.py ign_args:=\" -v 4 -r coast.sdf\" ;exec bash" 
gnome-terminal --tab -- bash -c "ros2 run ros_ign_bridge my_bridge; exec bash"
gnome-terminal --tab -- bash -c "ros2 launch vessel_det vessel_det_launch_multi_uav.py numbers:='$*'"
echo "Type anything to spawn UAVs..."
read a
gnome-terminal --tab -- bash -c "sleep 2; ros2 launch search spawn_suav.py numbers:='$*'"
# for i in "$@"; do
# 	xxx=`expr 0 - $i \* 2 - 1490`
# 	gnome-terminal --tab -- bash -c "sleep 2; ros2 launch mbzirc_ign spawn.launch.py name:=suav_$i world:=coast model:=mbzirc_quadrotor x:=${xxx} y:=0 z:=4.3 R:=0 P:=0 Y:=0 slot0:=mbzirc_hd_camera slot0_rpy:=\"0 30 0\" gripper:=mbzirc_suction_gripper flightTime:=60; exec bash"
# done
echo "Type anything to start control..."
read b
gnome-terminal --tab -- bash -c "ros2 run search manager"
gnome-terminal --tab -- bash -c "ros2 run search commander"
cnt=0
for i in "$@"; do
	cnt=`expr $cnt + 1`
	if test $cnt -gt 4
	then
		# echo "second row";
		ii=`expr $cnt - 4`
		wd_y=550
	else
		# echo "first row";
		ii=$cnt
		wd_y=33
	fi
    wd_x=`expr $ii \* 340 - 260`
	# gnome-terminal --geometry=36x25+${wd_x}+${wd_y} --tab -- bash -c "ros2 run search sUAV ${i}"
	gnome-terminal --tab -- bash -c "ros2 run search sUAV ${i}"
done
