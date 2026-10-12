# 01_how_to_connect_live_lidar

## 1.1 Launch file

```
start.{launch,py}              // support parameter configuration and launch rviz
start_with_config.{launch,py}  // launch use config.yaml and launch rviz
```

## 1.2 Connect to lidar using config.yaml

1. Create a new YAML file or use the installed config.yaml file, and modify or input the necessary configurations as needed.

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

2. Start the Driver

```bash
// for ros
roslaunch seyond start_with_config.launch
roslaunch seyond start.launch config_path:=<config_file_path>

// for ros2
ros2 launch seyond start_with_config.py
ros2 launch seyond start.py config_path:=<config_file_path>
```

## 1.3 Start using command line parameters (supports only one lidar).

```bash
// for ros
roslaunch seyond start.launch lidar_ip:=172.168.1.10 udp_port:=8010
// for ros2
ros2 launch seyond start.py lidar_ip:=172.168.1.10 udp_port:=8010
```
