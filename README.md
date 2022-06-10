# MBZIRC 2023 Coordinated Search How-to-open

1. Coast Environment

```bash
ros2 launch mbzirc_ros competition_local.launch.py ign_args:="-v 4 -r coast.sdf"
```

2. Spawn UAV

```bash
ros2 launch mbzirc_ign spawn.launch.py name:=quadrotor_1 world:=simple_demo model:=mbzirc_quadrotor type:=uav x:=1 y:=2 z:=0.05 R:=0 P:=0 Y:=0 slot0:=mbzirc_hd_camera slot0_rpy:="0 30 0" 
```

3. IGN 2 ROS2 Bridge
```bash
ros2 run ros_ign_bridge my_bridge
```


4. Now we can run our own code
```bash
ros2 run search sUAV 5
```