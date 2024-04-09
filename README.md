### ROS2版本iG-LIO

STILL WORKING ON ......

| 工作基于                                             |
| ---------------------------------------------------- |
| https://github.com/liangheming/iG-LIO_SAM_LC         |
| https://github.com/KTH-RPL/DynamicMap_Benchmark      |
| https://github.com/CCNYRoboticsLab/imu_tools         |
| https://github.com/ros-planning/navigation2          |
| https://github.com/rst-tu-dortmund/teb_local_planner |
| https://github.com/ANYbotics/grid_map                |

#### 环境

| 软件              | 安装     | 版本                                           |
| ----------------- | -------- | ---------------------------------------------- |
| Eigen3            | apt安装  | 3.3.7                                          |
| ROS2              | apt安装  | galactic                                       |
| Ceres             | 编译安装 | 2.1.0                                          |
| Sophus            | 编译安装 | 1.22.10                                        |
| gtsam             | 编译安装 | 4.20                                           |
| fmt               | 任意     | latest                                         |
| yaml-cpp          | 编译安装 | fit system                                     |
| Glog / gflag      | 任意     | latest                                         |
| Livox-SDK2        | 编译安装 | https://github.com/Livox-SDK/Livox-SDK2        |
| livox_ros_driver2 | 编译安装 | https://github.com/Livox-SDK/livox_ros_driver2 |

#### 配置文件

ThirdParty/navigation2-galactic/nav2_bringup/bringup/params/base_robot_params.yaml

src/ig_lio_c/config/config_fg.yaml

src/ig_lio_c/config/parameters.yaml

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
