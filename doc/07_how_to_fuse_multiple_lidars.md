# 07_how_to_fuse_multiple_lidars

## 7.1 Related Parameters

**fusion_enable**: enable fusion

**fusion_topic**: topic for fusion publish

Note: Using this function requires ensuring that all LiDARs are time-synchronized.

## 7.2 config.yaml

Sample Description: Connect two LiDARs and enable fusion_enable.

```yaml
common:
  log_level: info                                     #Log level: info, warn, error
  fusion_enable: true                                 #Multi-lidar fusion enable
  fusion_topic: /seyond_lidar/fusion                  #Multi-lidar fusion topic
  fusion_time_window: 50                              #Fusion time window, unit: ms
  fusion_buffer_size: 10                              #Max buffer size per secondary lidar
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

      coordinate_mode: 3                              #Coordinate mode, x/y/z, 0:up/right/forward 3:forward/left/up
```
