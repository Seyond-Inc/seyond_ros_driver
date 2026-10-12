# 01_how_to_connect_live_lidar_cn

## 1.1 launch文件介绍

```
start.{launch,py}              // 支持命令行参数， 同时打开rviz
start_with_config.{launch,py}  // 不支持命令行参数，使用config.yaml启动，同时打开rviz
```

## 1.2 使用config.yaml连接雷达

1. 新建一个yaml文件或者使用安装的config.yaml文件，根据需要修改或写入相应配置

```yaml
common:
  log_level: info                                     #Log level: info, warn, error
lidars:
  - lidar:
      topic_prefix: /seyond_lidar                     #Topic prefix  ~/pointcloud, ~/packet, ~/imu
      lidar_ip: 172.168.1.10                          #Lidar ip
      port: 8010                                      #Lidar port
      udp_port: 8010                                  #Udp port, if < 0, use tcp

  - lidar:
      topic_prefix: /seyond_lidar1                    #Topic prefix  ~/pointcloud, ~/packet, ~/imu
      lidar_ip: 172.168.1.11                          #Lidar ip
      port: 8011                                      #Lidar port
      udp_port: 8011                                  #Udp port, if < 0, use tcp
```

2. 启动

```bash
// for ros
roslaunch seyond start_with_config.launch
roslaunch seyond start.launch config_path:=<config_file_path>

// for ros2
ros2 launch seyond start_with_config.py
ros2 launch seyond start.py config_path:=<config_file_path>
```

## 1.3 使用命令行参数启动（仅支持单雷达）

```bash
// for ros
roslaunch seyond start.launch lidar_ip:=172.168.1.10 udp_port:=8010
// for ros2
ros2 launch seyond start.py lidar_ip:=172.168.1.10 udp_port:=8010
```
