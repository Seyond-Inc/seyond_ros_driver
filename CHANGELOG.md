# CHANGELOG 

## [v1.0.0] (2025-02-19)

### Added
- Initial release.


## [v1.0.1] (2025-03-31)

### Added
- Support ROS2-Iron.

### Changed
- Updated package.xml.
- Changed the data type of intensity: from uint16_t to float.
- Update the build script, added some message displays.
- Used static linking for seyond_sdk.
- Updated log message to display the code location.

### Fixed
- Fixed the issue of the bloom-generate build.
- Fixed the submodule link error.


## [v1.0.2] (2025-08-13)

### Changed
- Updated the submodule seyond_sdk to v3.103.1.
- Deleted the unused cmake config.
- Added the submodule instructions to the README.md.

### Fixed
- Fixed replay_rosbag parameter error.
- Fixed the packet loss rate display error.
- Fixed launch file comment error.


## [v1.0.3] (2026-03-09)

### Added
- Support falcon ring_id calculation.
- Support IMU message publishing.

### Changed
- Use first packet timestamp as the frame timestamp.
- Updated Copyright info in source files.
- Updated the submodule seyond_sdk to v3.103.4.
- Synced the LICENSE file with seyond_sdk.

### Fixed
- Updated the package dependencies.
- Fixed documentation errors.


## [v1.0.4] (2026-06-19)

### Added
- Support inno_pc file replay.
- Added async publish thread to decouple pointcloud publishing from data callback.
- Added fusion_time_window and fusion_buffer_size config for multi-lidar fusion.
- Added coordinate transform support for IMU data.

### Changed
- Rewrote multi-lidar fusion with custom time-window sync, removed message_filters dependency.
- Removed name_value_pairs parameter.
- Changed min_range default from 0.4 to 0.1.
- Added default values for LidarConfig and CommonConfig structs.
- Removed fatal error auto-reconnect logic.

### Fixed
- Fixed ros2_driver_adapter.hpp frame_count_ and packets_width_ not initialized error.
- Fixed deb packaging products for Humble and later versions.
- Fixed ret double assignment in lidar_live_process.
- Fixed YAML loader not returning error on exceptions.
- Added pcap file path and lidar_ip length validation.


## [v1.0.5] (2026-10-12)

### Changed
- Replaced the `frame_topic`, `packet_topic` and `lidar_name` parameters with a single `topic_prefix`;
  topics are now derived as `<topic_prefix>/pointcloud`, `<topic_prefix>/packet` and `<topic_prefix>/imu`.
- Changed the default `fusion_topic` from `/iv_points_fusion` to `/seyond_lidar/fusion`.
- Decoupled `packet_mode` and `replay_rosbag`: the packet publisher is created when `packet_mode` is enabled
  and the packet subscriber when `replay_rosbag` is enabled; `replay_rosbag` no longer forces `packet_mode`.
- Point cloud is now always parsed and published, even when `packet_mode` is enabled.
- Updated the required CMake version to 3.10, replaced `ament_target_dependencies`/`rosidl_target_interfaces`
  with `target_link_libraries`/`rosidl_get_typesupport_target`, and renamed the `ROS_FOUND`/`ROS2_FOUND`
  macros to `ROS_MAKE`/`ROS2_MAKE`.
- Updated RViz configs, launch files and documentation to the new topic naming.

### Fixed
- Fixed missing `inline` on `ROSDemo` methods that caused multiple-definition link errors in the test node.
- Added the `pcl_conversions` include directory for the ROS2 build.
