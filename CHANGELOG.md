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


## [v1.0.4pre] (2026-06-19)

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

## [v1.0.8-dual] (unreleased)

### Changed
- Updated the submodule seyond_sdk from v3.103.4 to release-3.103.13.
- Synced the fork with upstream Seyond-Inc/seyond_ros_driver main (4 commits),
  bringing v1.0.4pre. The significant item for the Resort deliver-stage drops is
  "Added async publish thread to decouple pointcloud publishing from data callback" --
  the blocking data-callback publish is the mechanism behind the measured 465-725 ms
  stalls that overflow the SDK deliver queue.

### Fixed (from upstream seyond_sdk 3.103.4 -> 3.103.13)
- Memory corruption in `InnoLidarClient`: `inputParam_` was left uncopied on the
  client-param path; UDP port/IP negotiation reworked with a retry round and
  explicit validation (`get_set_udp_ports_ip`) instead of the old inline item list.
- `stage_client_deliver`: new `disable_do_crc` config key, allowing the per-packet
  CRC to be skipped in the deliver stage.
- `stage_client_read.cpp` substantially reworked (-81 lines) on the read path.
- `robin_nps_adjustment.h` updated (Robin-family NPS adjustment).
- UDP listener now logs the kernel's effective SO_RCVBUF at bind time
  (observability only - it reports the buffer, it does not enlarge it).
- `resource_stats` accounting updated.

### Added (upstream, not used by this fleet)
- Support for hd1-w and E2; HB fault table; HD IMU recording in get_pcd.

