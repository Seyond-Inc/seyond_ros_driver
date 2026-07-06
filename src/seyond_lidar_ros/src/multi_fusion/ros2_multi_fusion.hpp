/**********************************************************************************************************************
Copyright (c) 2024 Seyond
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

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "src/driver/driver_lidar.h"
#include "src/driver/point_types.h"

namespace seyond {

class MultiFusion {
public:
  using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
  using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

  explicit MultiFusion(std::shared_ptr<rclcpp::Node> node_ptr,
                       const std::vector<seyond::LidarConfig> &lidar_configs,
                       const seyond::CommonConfig &common_config);
  ~MultiFusion();

private:
  struct StampedMsg {
    PointCloud2ConstPtr msg;
    int64_t stamp_ms;
  };

  static int64_t toMs(const builtin_interfaces::msg::Time &t) {
    return static_cast<int64_t>(t.sec) * 1000LL +
           static_cast<int64_t>(t.nanosec) / 1000000LL;
  }

  void onPrimary(PointCloud2ConstPtr msg);
  void onSecondary(int32_t index, PointCloud2ConstPtr msg);
  void workerLoop();

  std::shared_ptr<rclcpp::Node> node_ptr_;
  rclcpp::Publisher<PointCloud2Msg>::SharedPtr fusion_frame_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr primary_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
      secondary_subs_;

  int32_t lidar_num_;
  int32_t time_window_ms_;
  int32_t buffer_size_;

  std::deque<StampedMsg> primary_queue_;
  std::mutex primary_mutex_;
  std::condition_variable primary_cv_;

  std::vector<std::deque<StampedMsg>> secondary_buffers_;
  std::vector<std::mutex> secondary_mutexes_;

  std::vector<std::string> topic_names_;

  std::thread worker_thread_;
  std::atomic<bool> running_;
};

inline MultiFusion::MultiFusion(std::shared_ptr<rclcpp::Node> node_ptr,
                                const std::vector<seyond::LidarConfig> &lidar_configs,
                                const seyond::CommonConfig &common_config)
    : node_ptr_(node_ptr),
      lidar_num_(static_cast<int32_t>(lidar_configs.size())),
      time_window_ms_(common_config.fusion_time_window),
      buffer_size_(common_config.fusion_buffer_size),
      secondary_mutexes_(lidar_configs.size() > 1 ? lidar_configs.size() - 1
                                                  : 0),
      secondary_buffers_(lidar_configs.size() > 1 ? lidar_configs.size() - 1
                                                  : 0),
      running_(false) {
  if (lidar_num_ < 2) {
    RCLCPP_WARN(node_ptr_->get_logger(),
                "[Fusion] Skipped: lidar_num=%d, need >= 2 to enable fusion", lidar_num_);
    return;
  }

  rclcpp::QoS qos{rclcpp::KeepLast(buffer_size_)};
  qos.reliable();
  fusion_frame_pub_ = node_ptr_->create_publisher<PointCloud2Msg>(
      common_config.fusion_topic, qos);

  topic_names_.reserve(lidar_num_);
  for (const auto &cfg : lidar_configs) {
    topic_names_.push_back(cfg.frame_topic);
  }

  primary_sub_ = node_ptr_->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_configs[0].frame_topic, qos,
      [this](PointCloud2ConstPtr msg) { this->onPrimary(msg); });

  secondary_subs_.resize(lidar_num_ - 1);
  for (int32_t i = 1; i < lidar_num_; i++) {
    int32_t idx = i - 1;
    secondary_subs_[idx] =
        node_ptr_->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_configs[i].frame_topic, qos,
            [this, idx](PointCloud2ConstPtr msg) {
              this->onSecondary(idx, msg);
            });
  }

  running_ = true;
  worker_thread_ = std::thread(&MultiFusion::workerLoop, this);

  RCLCPP_INFO(node_ptr_->get_logger(),
              "[Fusion] Started: primary=%s, secondaries=%d, time_window=%dms, "
              "buffer_size=%d",
              topic_names_[0].c_str(), lidar_num_ - 1, time_window_ms_,
              buffer_size_);
}

inline MultiFusion::~MultiFusion() {
  running_ = false;
  primary_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

inline void MultiFusion::onPrimary(PointCloud2ConstPtr msg) {
  {
    std::lock_guard<std::mutex> lock(primary_mutex_);
    primary_queue_.push_back({msg, toMs(msg->header.stamp)});
    while (primary_queue_.size() > static_cast<size_t>(buffer_size_)) {
      RCLCPP_WARN(node_ptr_->get_logger(),
                  "[Fusion] Primary queue overflow, dropping oldest frame");
      primary_queue_.pop_front();
    }
  }
  primary_cv_.notify_one();
}

inline void MultiFusion::onSecondary(int32_t index, PointCloud2ConstPtr msg) {
  std::lock_guard<std::mutex> lock(secondary_mutexes_[index]);
  secondary_buffers_[index].push_back({msg, toMs(msg->header.stamp)});
  while (secondary_buffers_[index].size() > static_cast<size_t>(buffer_size_)) {
    secondary_buffers_[index].pop_front();
  }
}

inline void MultiFusion::workerLoop() {
  while (running_) {
    StampedMsg primary;
    {
      std::unique_lock<std::mutex> lock(primary_mutex_);
      primary_cv_.wait(lock,
                       [this] { return !primary_queue_.empty() || !running_; });
      if (!running_)
        return;
      primary = primary_queue_.front();
      primary_queue_.pop_front();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(time_window_ms_));

    pcl::PointCloud<SeyondPoint> merged_cloud;
    pcl::fromROSMsg(*primary.msg, merged_cloud);

    int32_t merged_count = 0;
    std::string merged_info;
    std::string skip_reason;

    for (int32_t i = 0; i < lidar_num_ - 1; i++) {
      std::lock_guard<std::mutex> lock(secondary_mutexes_[i]);
      auto &buf = secondary_buffers_[i];

      while (!buf.empty() &&
             buf.front().stamp_ms < primary.stamp_ms - time_window_ms_) {
        buf.pop_front();
      }

      if (buf.empty()) {
        if (!skip_reason.empty())
          skip_reason += ", ";
        skip_reason += topic_names_[i + 1] + "(empty)";
        continue;
      }

      if (buf.front().stamp_ms > primary.stamp_ms + time_window_ms_) {
        if (!skip_reason.empty())
          skip_reason += ", ";
        skip_reason += topic_names_[i + 1] + "(late)";
        continue;
      }

      int64_t dt_ms = buf.front().stamp_ms - primary.stamp_ms;
      if (!merged_info.empty())
        merged_info += ", ";
      merged_info += topic_names_[i + 1] + "(" + std::to_string(dt_ms) + "ms)";

      pcl::PointCloud<SeyondPoint> sec_cloud;
      pcl::fromROSMsg(*buf.front().msg, sec_cloud);
      merged_cloud += sec_cloud;
      buf.pop_front();
      merged_count++;
    }

    PointCloud2Msg merged_msg;
    pcl::toROSMsg(merged_cloud, merged_msg);
    merged_msg.header = primary.msg->header;
    fusion_frame_pub_->publish(merged_msg);

    RCLCPP_INFO(node_ptr_->get_logger(),
                "[Fusion] stamp=%ldms, merged %d/%d: %s%s%s", primary.stamp_ms,
                merged_count, lidar_num_ - 1, merged_info.c_str(),
                skip_reason.empty() ? "" : ", skipped: ", skip_reason.c_str());
  }
}

} // namespace seyond
