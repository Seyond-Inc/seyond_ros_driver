/**********************************************************************************************************************
Copyright (c) 2025 Seyond
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For Seyond LiDAR SDK Library
(2-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**********************************************************************************************************************/

#pragma once
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <Eigen/Eigen>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <fstream>
#include <condition_variable>


#include "point_types.h"
#include "sdk_common/inno_lidar_packet.h"
#include "sdk_common/ring_id_converter_interface.h"
#include "utils/inno_lidar_log.h"


#ifdef ENABLE_XYZIT
typedef seyond::PointXYZIT SeyondPoint;
#else
typedef pcl::PointXYZI SeyondPoint;
#endif

namespace seyond {

struct CommonConfig {
  std::string log_level;
  bool fusion_enable;
  std::string fusion_topic;
};

struct LidarConfig {
  int32_t index;
  bool replay_rosbag;
  bool packet_mode;
  int32_t aggregate_num;

  std::string frame_id;
  std::string packet_topic;
  std::string frame_topic;

  std::string lidar_name;
  std::string lidar_ip;
  int32_t port;
  int32_t udp_port;
  bool reflectance_mode;
  int32_t multiple_return;
  bool enable_falcon_ring;
  bool enable_imu_msg;

  bool continue_live;

  std::string pcap_file;
  std::string hv_table_file;
  int32_t packet_rate;
  int32_t file_rewind;

  double max_range;
  double min_range;
  std::string name_value_pairs;
  int32_t coordinate_mode;

  bool transform_enable;
  double x;
  double y;
  double z;
  double pitch;
  double yaw;
  double roll;
  std::string transform_matrix;
};

struct TransformParam {
  double x;
  double y;
  double z;
  double pitch;
  double yaw;
  double roll;
};

class DriverLidar {
 public:
  explicit DriverLidar(const LidarConfig& lidar_config);
  ~DriverLidar();

  // static callback warpper
  static void lidar_message_callback_s(int32_t handle, void *ctx, uint32_t from_remote, enum InnoMessageLevel level,
                                       enum InnoMessageCode code, const char *error_message);
  static int32_t lidar_data_callback_s(int32_t handle, void *ctx, const InnoDataPacket *pkt);
  static int32_t lidar_status_callback_s(int32_t handle, void *ctx, const InnoStatusPacket *pkt);
  static void lidar_log_callback_s(void *ctx, enum InnoLogLevel level, const char *header1, const char *header2,
                                   const char *msg);
  // lidar configuration
  static void init_log_s(std::string &log_limit,
                         const std::function<void(int32_t, const char *, const char *)> &callback);
  void start_lidar();
  void stop_lidar();

  void register_publish_packet_callback(const std::function<void(const int8_t*, uint64_t, double, bool)>& callback) {
    packet_publish_cb_ = callback;
  }
  void register_publish_frame_callback(
      const std::function<void(pcl::PointCloud<SeyondPoint> &, double)> &callback) {
    frame_publish_cb_ = callback;
  }
  void register_publish_imu_callback(const std::function<void(std::vector<float> &, uint64_t)>& callback) {
    imu_data_publish_cb_ = callback;
  }
  void init_transform_matrix();
  void transform_pointcloud();
  void convert_and_parse(const int8_t *pkt);

 private:
  // callback group
  int32_t lidar_data_callback(const InnoDataPacket *pkt);
  void lidar_message_callback(uint32_t from_remote, enum InnoMessageLevel level, enum InnoMessageCode code,
                               const char *msg);
  int32_t lidar_status_callback(const InnoStatusPacket *pkt);

  void convert_and_parse(const InnoDataPacket *pkt);
  int32_t lidar_parameter_set();
  void input_parameter_check();
  bool setup_lidar();
  int32_t lidar_live_process();
  int32_t pcap_playback_process();
  int32_t set_config_name_value();
  void start_check_datacallback_thread();
  void data_packet_parse(const InnoDataPacket *pkt);
  template <typename PointType>
  void point_xyz_data_parse(bool is_use_refl, uint32_t point_num, PointType point_ptr);

 public:
  LidarConfig param_;
  // for generic lidar
  bool anglehv_table_init_{false};
  std::vector<char> anglehv_table_;

  pcl::PointCloud<SeyondPoint>::Ptr pcl_pc_ptr;

  std::function<void(const int8_t*, uint64_t, double, bool)> packet_publish_cb_;
  std::function<void(pcl::PointCloud<SeyondPoint>&, double)> frame_publish_cb_;
  std::function<void(std::vector<float>&, uint64_t)> imu_data_publish_cb_;
  static std::function<void(int32_t, const char*, const char*)> ros_log_cb_s_;

  // status
  bool is_running_{false};
  std::thread check_datacallback_thread_;
  std::condition_variable running_cv_;
  std::mutex running_mutex_;
  std::atomic_bool is_receive_data_{false};
  int32_t lidar_handle_{-1};
  bool fatal_error_{false};
  int64_t current_frame_id_{-1};
  std::vector<uint8_t> data_buffer;
  double current_ts_start_;
  double frame_start_ts_;

  // transform
  Eigen::Matrix4f T_2_0_;
  bool transform_degree_flag_{false};

  RingIdConverterInterface *ring_id_converter_{nullptr};
};

}  // namespace seyond
