# MBZIRC 2023 Coordinated Search How-to-open

1. Coast Environment

```bash
ros2 launch mbzirc_ros competition_local.launch.py ign_args:="-v 4 -r coast.sdf"
```

2. Spawn UAV

```bash
ros2 launch mbzirc_ign spawn.launch.py name:=suav_1 world:=coast model:=mbzirc_quadrotor x:=1 y:=2 z:=0.05 R:=0 P:=0 Y:=0 slot0:=mbzirc_hd_camera slot0_rpy:="0 30 0" 
```

3. IGN 2 ROS2 Bridge
```bash
ros2 run ros_ign_bridge my_bridge
```


4. Now we can run our own code
```bash
ros2 run search sUAV 5
```

# Or we can start everything by Shell

```bash
./start_everything.sh 1 2 3 4 5 6 7 8 9 10
```


# What we did to the code of mbzirc_ws

1. Change `mass` of small object from 3kg to 1kg 
   (in `/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models/small case/2/model.sdf`)

2. Change `maxRotVelocity` from 800 to 1800 
   (in `/mbzirc_ws/src/mbzirc/mbzirc_ign/models/mbzirc_quadrotor/model.sdf.erb`)


# c47v: Circling for seven vessels

- For video recording to modeling.
