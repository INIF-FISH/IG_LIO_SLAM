### ROS2版本iG-LIO

STILL WORKING ON ......

工作基于https://github.com/liangheming/iG-LIO_SAM_LC

#### 命令

建图与定位

```
ros2 launch ig_lio_c map_mapping_launch.py
```

导航与规划

```
ros2 launch nav2_bringup base_robot_launch.py
```

保存pcd地图

```
ros2 service call /SaveMap ig_lio_c_msgs/srv/SaveMap "{save_path: '/home/XXX', resolution: 0.0}"
```

使用保存的pcd地图转换占据地图

```
ros2 service call /CovertMap ig_lio_c_msgs/srv/CovertMap "{pcd_path: '/home/XXX/XXX.pcd',map_file_name: 'XXX_map'}"
```
