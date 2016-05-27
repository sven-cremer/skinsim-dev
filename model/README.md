This folder contains generated models, worlds, and configuration parameters.  

To load a world in gazebo:
```
cd /path/to/model/worlds
gazebo skin_patch_grid.world
```


To use with ROS:
```
roslaunch gazebo_ros empty_world.launch
```
and then insert a skin board from the menu. To apply a virtual force:
```
rosservice call /gazebo/apply_body_wrench "body_name: 'spring_2'
reference_frame: 'spring_2'
reference_point: {x: 0.0, y: 0.0, z: 0.0}
wrench:
  force: {x: 10.0, y: 10.0, z: 100.0}
  torque: {x: 0.0, y: 0.0, z: 0.0}
start_time: {secs: 0, nsecs: 0}
duration: {secs: 1, nsecs: 0}" 
```

