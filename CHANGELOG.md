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


## [v1.0.3x] (unreleased)

### Added
- Support falcon ring_id calculation.

### Changed
- Updated the submodule seyond_sdk to v3.103.4.
- Synced the LICENSE file with seyond_sdk.

### Fixed
- Updated the package dependencies.
