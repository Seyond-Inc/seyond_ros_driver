# 04_how_to_record_data

## 4.1 Related Parameters

**packet_mode**: Enable packet_topic to publish packet data.

**aggregate_num**(optional): Controls the frequency of packet publishing when packet_mode is enabled.

- Frequency：Publish after accumulating a specified number of packets.
- A smaller value increases the publishing frequency. Due to QoS being set to Best effort during recording, the recording may be incomplete.

**replay_rosbag**: Enable to replay packet data for parsing recorded packets.

## 4.2 Record

```bash
// ros
// record packet bag
rosbag record /iv_packets
// record pointcloud
rosbag record /iv_points

// ros2
// record packet bag
ros2 bag record /iv_packets
// record pointcloud
ros2 bag record /iv_points
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
