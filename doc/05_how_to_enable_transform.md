# 05_how_to_enable_transform

## 5.1 Related Parameters

**transform_enable**: enable transform.

**x**/**y**/**z**/**pitch**/**yaw**/**roll**: Transformation parameters; angles are in radians.

**transform_matrix**: Transformation matrix as a string; input 16 values separated by commas.

- Example: '1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0'
- Note: it has a higher priority than x/y/z/pitch/yaw/roll. If not used, should be left empty

**coordinate_mode**: Coordinate system mode, default is 3, right-hand coordinate system, x/y/z represent forward/left/up respectively.

- 0: x-up, y-right, z-forward
- 1: x-right, y-forward, z-up
- 2: x-right, y-up, z-forward
- 3: x-forward, y-left, z-up
- 4: x-forward, y-up, z-right

## 5.2 config.yaml

Enable the transform_enable parameter.

```yaml
common:
  log_level: info                                     #Log level: info, warn, error
lidars:
  - lidar:
      frame_topic: /iv_points                         #Frame topic

      lidar_name: seyond                              #Lidar name
      lidar_ip: 172.168.1.10                          #Lidar ip
      port: 8010                                      #Lidar port
      udp_port: 8010                                  #Udp port, if < 0, use tcp

      coordinate_mode: 3                              #Coordinate mode, x/y/z, 0:up/right/forward 3:forward/left/up

      transform_enable: true                          #Transform enable
      x: 1.0                                          #X
      y: 0.2                                          #Y
      z: 3.0                                          #Z
      pitch: 0.2                                      #Pitch
      yaw: 0.1                                        #Yaw
      roll: 0.3                                       #Roll

      transform_matrix: ''                            #Transformation matrix string
```

## 5.3 Dynamic Adjustment

ROS driver supports dynamic parameter adjustment for up to 5 lidars, named as lidar1_x, ..., lidar2_x, ...

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

ROS2 driver does not limit the number of lidars and uses a formatted naming convention: <lidar_name>_x, ...

Therefore, if multiple lidars need to use this feature, ensure that the parameter lidar_name is unique.

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```
