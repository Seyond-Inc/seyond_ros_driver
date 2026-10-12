# 06_how_to_use_test_node_cn

## 6.1 test.launch文件参数

| Parameter         | Default Value | description          |
| ------------      | ------------- | -------------------- |
| topic_prefix      | /seyond_lidar | 待测试的topic prefix  |
| hz                | false         | 输出帧率和帧平均点数    |
| window            | 30            | 频率计算窗口大小       |
| stamp             | false         | 输出帧时间戳          |
| packet_size       | false         | 输出每帧包数量         |
| packet_loss_rate  | false         | 输出丢包率            |

## 6.2 使用

```bash
// ros:

// 输出帧率和帧平均点数
roslaunch seyond test.launch hz:=true window:=30

// 输出每帧的时间戳
roslaunch seyond test.launch stamp:=true

// 输出每帧的packet包数量(注意，驱动侧需要开启packet_mode)
roslaunch seyond test.launch packet_size:=true
```

```bash
// ros2:

// 输出帧率和帧平均点数
ros2 launch seyond test.py hz:=true window:=30

// 输出每帧的时间戳
ros2 launch seyond test.py stamp:=true

// 输出每帧的packet包数量(注意，驱动侧需要开启packet_mode)
ros2 launch seyond test.py packet_size:=true
```
