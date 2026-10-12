# 03_how_to_parse_pcap_data_cn

## 3.1 相关参数说明

**pcap_file**: pcap文件路径

**packet_rate**(可选)：文件回放速率，默认为10000，即1.0x

- 当packet_rate=0，尽最大速度回放
- 当packet_rate<=100，按文件大小回放，速度为<packet_rate>MB/s
- 当packet_rate>100，按倍率回放，速度为 (<packet_rate>/10000.0)x

**file_rewind**: 重播次数，默认为0，仅回放一次

- 当file_rewind>=0, 再播<file_rewind>次
- 当file_rewind=-1, 循环播放

注意：**lidar_ip**和**udp_port**需要与pcap文件中的雷达数据一致

## 3.2 config.yaml配置

```yaml
common:
  log_level: info                                     #Log level: info, warn, error
lidars:
  - lidar:
      frame_id: seyond                                #Frame id
      topic_prefix: /seyond_lidar                     #Topic prefix  ~/pointcloud, ~/packet, ~/imu
      lidar_ip: 172.168.1.10                          #Lidar ip
      udp_port: 8010                                  #Udp port, if < 0, use tcp

      pcap_file: ''                                   #Pcap file
      packet_rate: 10000                              #Packet rate
      file_rewind: 0                                  #File rewind
```

```bash
// ros
roslaunch seyond start_with_config.launch
// ros2
ros2 launch seyond start_with_config.py
```

## 3.3 命令行启动

```bash
// ros
roslaunch seyond start.launch pcap_file:=<pcap_file_path> lidar_ip:=<lidar_ip> udp_port:=<udp_port>
// ros2
ros2 launch seyond start.py pcap_file:=<pcap_file_path> lidar_ip:=<lidar_ip> udp_port:=<udp_port>
```
