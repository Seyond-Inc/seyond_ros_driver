# 05_how_to_enable_transform_cn

## 5.1 相关参数说明

**transform_enable**: 启用transform

**x**/**y**/**z**/**pitch**/**yaw**/**roll**: 变换参数，角度为弧度制

**transform_matrix**: 变换矩阵字符串，输入16个数值用逗号隔开

- 例: '1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0'
- 注意: 其优先级高于x/y/z/pitch/yaw/roll, 若不使用，请置空

**coordinate_mode**: 坐标模式，默认为3，右手坐标系，x/y/z分别表示前/左/上

- 0： x向上，y向右，z向前
- 1： x向右，y向前，z向上
- 2： x向右，y向上，z向前
- 3： x向前，y向左，z向上
- 4： x向前，y向上，z向右

## 5.2 config.yaml

样例说明：启用transform_enable参数

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

## 5.3 动态调整

ros支持5台雷达的动态参数调整，命名为lidar1_x,…,lidar2_x,…

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

ros2不限制数量，命名格式化为<lidar_name>_x,…

因此如果多台雷达需要使用该功能，需要确保参数lidar_name独立

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```
