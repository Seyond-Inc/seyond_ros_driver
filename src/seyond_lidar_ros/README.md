# About the package
  This is the primary package of the Seyond LiDAR ROS driver.

  **directory structure**

```
├── config                              // lidar config file
├── launch                              // ros1 & ros2 launch file
├── msg                                 // msg file
├── node                                // main file
│   ├── seyond_node.cc
│   └── seyond_test.cc
├── rviz                                // rviz config file
├── src
│   ├── driver
│   │   ├── driver_lidar.cc
│   │   ├── driver_lidar.h
│   │   ├── point_types.h
│   │   ├── ros1_driver_adapter.hpp
│   │   ├── ros2_driver_adapter.hpp
│   │   └── yaml_tools.hpp
│   ├── multi_fusion                    // for multiple lidars fusion
│   │   ├── ros1_multi_fusion.hpp
│   │   └── ros2_multi_fusion.hpp
│   ├── test                            // test node code
│   │   ├── ros1_test.hpp
│   │   └── ros2_test.hpp
│   └── seyond_sdk                      // seyond sdk
├── package.xml
├── CMakeLists.txt
├── LICENSE
└── README.md

```

# Compile

 1. **Copy**
  copy /seyond_lidar_ros/ directory to your ROS/ROS2 workspace
 
 2. **Build Seyond SDK**
  ```bash
  cd seyond_lidar_ros/src/seyond_sdk/build
  ./build_unix.sh
  cd -
  ```
  if not success, please refer to the latest Seyond SDK documentation

 3. **Build ros package**
  ```bash
  source /opt/ros/<ROS_DISTRO>/setup.sh
  // for ROS
  catkin_make install

  // for ROS2
  colcon build
  ```
