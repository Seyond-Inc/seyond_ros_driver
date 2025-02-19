# 介绍

 [English Version](README.md) 

**seyond_lidar_ros** 是一个用于Seyond雷达的ROS/ROS2驱动程序。
该项目基于Seyond-SDK构建，为客户提供了一个实际的参考，展示了如何使用Seyond-SDK。

## 目录结构

```
├── build.bash                          // 编译脚本
├── create_deb.bash                     // 打包脚本
├── doc                                 // 文档
├── src
│   └── seyond_lidar_ros                // ros包
├── LICENSE
├── CHANGELOG.md
├── README_CN.md
└── README.md
```

# 雷达支持

- Falcon-K1
  
- Falcon-K2
  
- Robin-W
  
- Robin-E1X
  

# 环境依赖

以下官方版本已经验证支持

| 发行版 | 平台 | 发布日期 | EOL日期 |
| --- | --- | --- | --- |
| Melodic | Ubuntu 18.04 | May 23rd, 2018 | June 2023 |
| Noetic | Ubuntu 20.04 | May 23rd, 2020 | May 2025 |
| Foxy | Ubuntu 20.04 | June 5th, 2020 | June 2023 |
| Galactic | Ubuntu 20.04 | May 23rd, 2021 | December 2022 |
| Humble | Ubuntu 22.04 | May 23rd, 2022 | May 2027 |
| Jazzy | Ubuntu 24.04 | May 23rd, 2024 | May 2029 |

## 安装Yaml


```
sudo apt-get update
sudo apt-get install -y  libyaml-cpp-dev
```

# 配置参数

请参考[config.yaml](/src/seyond_lidar_ros/config/config.yaml), 支持多雷达输入

# ROS命令行参数介绍

| 参数名 | 默认值 | 描述 |
| --- | --- | --- |
| config_path | ""  | 配置输入，如果配置了该项，其他命令行参数将忽视 |
| log_level | info | 驱动日志输出限制，选项: info warn error |
| replay_rosbag | false | packet rosbag 回放标志，监听外部的packet_topic数据包并解析 |
| packet_mode | false | 启用packet模式，将会开启packet_topic，发布packet数据包 |
| aggregate_num | 20  | 设置packet模式的包聚合数量 |
| frame_id | seyond |     |
| frame_topic | iv_points |     |
| packet_topic | iv_packets |     |
| lidar_name | seyond | 雷达名，用于区分日志消息 |
| lidar_ip | 172.168.1.10 |     |
| port | 8010 | 雷达源服务端口 |
| udp_port | 8010 | 雷达端配置的UDP目的端口 |
| reflectance_mode | true | 反射模式 false:强度模式 true:反射率模式 |
| multiple_return | 1   | 回波模式 1:单回波 2:双回波 |
| continue_live | false | 启用后，无法连接将自动重启驱动 |
| pcap_file | ""  | pcap文件路径，用于回放pcap文件 |
| hv_table_file | ""  | hv_table文件路径，用于解析generic雷达的pcap包 |
| packet_rate | 10000 | 回放pcap包的速度， 如果 value <= 100 :  value MB/s，如果 value  >  100 :  value / 10000 倍速回放 |
| file_rewind | 0   | 额外重播pcap包的次数 0:不额外重播 rewind -1: 循环 |
| max_range | 2000 | 最大距离限制 |
| min_range | 0.4 | 最小距离限制 |
| name_value_pairs | ""  | 额外配置字符串 |
| coordinate_mode | 3   | 坐标轴方向, x/y/z, 0:up/right/forward 3:forward/left/up |
| transform_enable | false | 启用旋转变换参数 |
| x   | 0.0 |     |
| y   | 0.0 |     |
| z   | 0.0 |     |
| pitch | 0.0 |     |
| yaw | 0.0 |     |
| roll | 0.0 |     |
| transform_matrix | ""  | 变换矩阵字符串，用逗号隔开, 如果非空，优先级高于x/y/z/pitch/yaw/roll |

# 快速开始

如果您有自己的ROS/ROS2工作空间，请参考[ROS包构建](src/seyond_lidar_ros/README.md)

**编译**

```
source /opt/ros/<ROS_DISTRO>/setup.sh
./build.bash
```

**运行**

```
source install/setup.bash
roslaunch seyond start.launch
// or
ros2 launch seyond start.py
```

# 额外文档

[在线雷达连接](doc/01_how_to_connect_live_lidar_cn.md)

[解析pcap数据](doc/03_how_to_parse_pcap_data_cn.md)

[点类型](doc/02_how_to_change_point_type_cn.md)

[录制 & 回放](doc/04_how_to_record_data_cn.md)

[点云变换](05_how_to_enable_transform_cn.md)

[点云测试](doc/06_how_to_use_test_node_cn.md)

[deb打包](doc/08_how_to_create_deb_cn.md)

[点云融合](doc/07_how_to_fuse_multiple_lidars_cn.md)
