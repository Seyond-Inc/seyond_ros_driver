# 04_how_to_record_data

## 4.1 Related Parameters

**packet_mode**: Enable to publish packet data on <topic_prefix>/packet.

**aggregate_num**(optional): Controls the frequency of packet publishing when packet_mode is enabled.

- Frequency：Publish after accumulating a specified number of packets.
- A smaller value increases the publishing frequency. Due to QoS being set to Best effort during recording, the recording may be incomplete.

**replay_rosbag**: Enable to replay packet data for parsing recorded packets.

## 4.2 Record

```bash
// ros
// record packet bag
rosbag record /seyond_lidar/packet
// record pointcloud
rosbag record /seyond_lidar/pointcloud

// ros2
// record packet bag
ros2 bag record /seyond_lidar/packet
// record pointcloud
ros2 bag record /seyond_lidar/pointcloud
```

## 4.3 Replay

```bash
// ros
// replay packet bag
rosbag play <packet_rosbag> -l
// enable the driver to parse packet
roslaunch seyond start.launch replay_rosbag:=true

// ros2
// replay packet bag
ros2 bag play <packet_rosbag> -l
// enable the driver to parse packet
ros2 launch seyond start.py replay_rosbag:=true
```
