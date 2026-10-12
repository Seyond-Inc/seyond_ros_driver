# 06_how_to_use_test_node

## 6.1 test.launch

| Parameter         | Default Value | description                        |
| ----------------- | ------------- | ---------------------------------- |
| topic_prefix      | /seyond_lidar | topic prefix to be tested          |
| hz                | false         | print frame hz(include points num) |
| window            | 30            | hz window size                     |
| stamp             | false         | print frame stamp                  |
| packet_size       | false         | print frame packet size            |
| packet_loss_rate  | false         | print packet loss rate             |

## 6.2 Usage

```bash
// ros:

// print frame hz and average points per frame
roslaunch seyond test.launch hz:=true window:=30

// print frame timestamp
roslaunch seyond test.launch stamp:=true

// print the packets num (need to enable packet_mode)
roslaunch seyond test.launch packet_size:=true
```

```bash
// ros2:

// print frame hz and average points per frame
ros2 launch seyond test.py hz:=true window:=30

// print frame timestamp
ros2 launch seyond test.py stamp:=true

// print the packets num (need to enable packet_mode)
ros2 launch seyond test.py packet_size:=true
```
