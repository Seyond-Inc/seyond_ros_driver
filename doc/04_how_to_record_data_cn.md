# 04_how_to_record_data_cn

## 4.1 相关参数说明

**packet_mode**: 启用packet_topic，将发布packet包

**aggregate_num**(可选)：启用packet_mode后，用于控制packet包发布频率

- 发布频率：每次累积指定数量的包后发布
- 设置值较小时，发布频率较高，由于录制时Qos配置默认为Best effort，录制可能不完整

**replay_rosbag**: 回放packet包模式，当需要解析录制的packet包时启用

## 4.2 录制

```bash
// ros
// 录制 packet包
rosbag record /iv_packets
// 录制 点云
rosbag record /iv_points

// ros2
// 录制 packet包
ros2 bag record /iv_packets
// 录制 点云
ros2 bag record /iv_points
```

## 4.3 回放

```bash
// ros
// 回放 packet包
rosbag play <packet_rosbag> -l
// 使用驱动解析
roslaunch seyond start.launch replay_rosbag:=true

// ros2
// 回放 packet包
ros2 bag play <packet_rosbag> -l
// 使用驱动解析
ros2 launch seyond start.py replay_rosbag:=true
```
