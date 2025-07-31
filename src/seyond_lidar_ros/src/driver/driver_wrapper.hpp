/*
 *  Copyright (C) 2025 Seyond Inc.
 *
 *  License: Apache License
 *
 *  $Id$
 */

#pragma once
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <fstream>
#include <condition_variable>

#include <seyond/common/common_header.hpp>
#include <seyond/driver/lidar_driver.hpp>
#include <seyond/log/log.hpp>
#include <seyond/msg/point_types.hpp>

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

  bool continue_live;

  std::string pcap_file;
  std::string hv_table_file;
  int32_t packet_rate;
  int32_t file_rewind;

  double max_range;
  double min_range;
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

class ROSDriver {
 public:
  explicit ROSDriver(const LidarConfig& lidar_config) {
    lidar_config_ = lidar_config;
    param_.source_param.source_ip = lidar_config.lidar_ip;
    param_.source_param.source_port = lidar_config.port;
    param_.source_param.pcap_file = lidar_config.pcap_file;
    // param_.source_param.raw_file = lidar_config.inno_pc_file;
    param_.parser_param.hv_table_file = lidar_config.hv_table_file;
    param_.source_param.file_loop = lidar_config.file_rewind == -1 ? true : false;
  }
  ~ROSDriver() {
    stop();
    lidar_driver_ptr_.reset();
  }

  void start() {
    if (lidar_driver_ptr_.get() == nullptr) {
      lidar_driver_ptr_ = std::make_shared<LidarDriver<LidarPointCloud<LidarPoint>>>();
    }

    if (!lidar_config_.replay_rosbag) {
      lidar_driver_ptr_->init(param_);
      lidar_driver_ptr_->start();
    }

    is_running_ = true;
    publish_thread_ = std::thread(std::bind(&ROSDriver::publishFrame, this));
  }

  void stop() {
    if (lidar_driver_ptr_.get() != nullptr) {
      lidar_driver_ptr_->stop();
    }
    is_running_ = false;
    if (publish_thread_.joinable()) {
      publish_thread_.join();
    }
  }

  void parsePacket(const uint8_t* data, size_t size) {
    // lidar_driver_ptr_->directPushPacket(data, size);
  }

  void register_publish_packet_callback(const std::function<void(const uint8_t*, uint64_t, bool)>& callback) {
    packet_publish_cb_ = callback;
  }
  void register_publish_frame_callback(
      const std::function<void(std::shared_ptr<LidarPointCloud<LidarPoint>>)> &callback) {
    frame_publish_cb_ = callback;
  }

  void getHVTable(std::vector<uint8_t>& hv_table_content) {
    if (lidar_driver_ptr_.get() != nullptr) {
      lidar_driver_ptr_->recordHVTable(hv_table_content);
    }
  }

  void publishFrame() {
    while (is_running_) {
      auto point_cloud = lidar_driver_ptr_->getPointCloud();
      if (point_cloud.get() != nullptr) {
        if (frame_publish_cb_) {
          frame_publish_cb_(point_cloud);
        }
        lidar_driver_ptr_->freePointCloud(point_cloud);
      }
    }
  }

  void packetCallback(const uint8_t* buf, size_t size) {
    CommonPacketHeader *header = reinterpret_cast<CommonPacketHeader*>(const_cast<uint8_t*>(buf));
    uint64_t idx = header->idx;
    if (expected_idx_ == 0) {
      expected_idx_ = idx + 1;
      return;
    } else if (idx != expected_idx_) {
      packet_publish_cb_(buf, size, true);
      expected_idx_ = idx;
    } else {
      packet_publish_cb_(buf, size, false);
    }
  }

 public:
  LidarConfig lidar_config_;
  DriverParam param_;
  bool anglehv_table_init_{false};

  std::function<void(const uint8_t*, uint64_t, bool)> packet_publish_cb_;
  std::function<void(std::shared_ptr<LidarPointCloud<LidarPoint>>)> frame_publish_cb_;
  static std::function<void(int32_t, const char*, const char*)> ros_log_cb_s_;

 private:
  bool is_running_{false};
  uint64_t expected_idx_{0};
  std::shared_ptr<LidarDriver<LidarPointCloud<LidarPoint>>> lidar_driver_ptr_;
  std::thread publish_thread_;
};

}  // namespace seyond
