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
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <chrono>

#include "src/driver/point_types.h"
#include "seyond/SeyondScan.h"

class ROSDemo {
 public:
  ROSDemo() = default;
  ~ROSDemo();
  void init();
  void spin();

 private:
  void subscribePointCloud(const sensor_msgs::PointCloud2ConstPtr &msg);
  void printFrameStamp(const sensor_msgs::PointCloud2ConstPtr &msg);
  void printFrameHZ(const sensor_msgs::PointCloud2ConstPtr &msg);

  void subscribePacket(const seyond::SeyondScan::ConstPtr &msg);
  void printPacketSize(const seyond::SeyondScan::ConstPtr &msg);
  void printPacketLossRate(const seyond::SeyondScan::ConstPtr &msg);

 private:
  std::shared_ptr<ros::NodeHandle> nh_;
  std::unique_ptr<ros::NodeHandle> private_nh_;

  ros::Subscriber inno_frame_sub_;
  ros::Subscriber inno_pkt_sub_;

  std::string packet_topic_;
  std::string frame_topic_;
  std::chrono::steady_clock::time_point curr_time_;
  std::chrono::steady_clock::time_point last_time_;

  bool first_time_{true};

  int32_t cur_start_sub_seq_{-1};
  int32_t received_packets_count_{0};
  int32_t total_expected_packets_count_{0};
  uint16_t last_sub_seq_{0};

  int32_t window_{30};
  uint64_t window_points_{0};
  uint32_t packet_size_{0};
  bool is_print_frame_stamp_{false};
  bool is_print_frame_hz_{false};
  bool is_print_packet_size_{false};
  bool is_print_packet_loss_rate_{false};
};

ROSDemo::~ROSDemo() {
  if (is_print_packet_loss_rate_) {
    total_expected_packets_count_ += last_sub_seq_ - cur_start_sub_seq_;
    ROS_INFO(
        "Total packet loss rate: %.5lf %% [%d/%d]",
        1.0 - static_cast<double>(received_packets_count_) / static_cast<double>(total_expected_packets_count_ + 1),
        received_packets_count_, total_expected_packets_count_ + 1);
  }
}

void ROSDemo::init() {
  nh_ = std::make_shared<ros::NodeHandle>();
  private_nh_ = std::make_unique<ros::NodeHandle>("~");
  private_nh_->param("frame_topic", frame_topic_, std::string("iv_points"));
  private_nh_->param("packet_topic", packet_topic_, std::string("iv_packets"));
  private_nh_->param("stamp", is_print_frame_stamp_, false);
  private_nh_->param("hz", is_print_frame_hz_, false);
  private_nh_->param("window", window_, 30);
  private_nh_->param("packet_size", is_print_packet_size_, false);
  private_nh_->param("packet_loss_rate", is_print_packet_loss_rate_, false);

  if (window_ <= 0) {
    ROS_ERROR("window must be greater than 0, set to default 30");
    window_ = 30;
  }

  if (is_print_packet_size_) {
    ROS_INFO("please make sure packet_mode is enabled in the driver");
  }

  inno_frame_sub_ = nh_->subscribe(frame_topic_.c_str(), 10, &ROSDemo::subscribePointCloud, this);
  inno_pkt_sub_ = nh_->subscribe(packet_topic_.c_str(), 100, &ROSDemo::subscribePacket, this);
}

void ROSDemo::spin() {
  ros::spin();
}

void ROSDemo::subscribePointCloud(const sensor_msgs::PointCloud2ConstPtr &msg) {
  if (is_print_frame_stamp_) printFrameStamp(msg);
  if (is_print_frame_hz_) printFrameHZ(msg);
}

void ROSDemo::printFrameStamp(const sensor_msgs::PointCloud2ConstPtr &msg) {
  ROS_INFO("sec: %d, nanosec: %09d", msg->header.stamp.sec, msg->header.stamp.nsec);
}

void ROSDemo::printFrameHZ(const sensor_msgs::PointCloud2ConstPtr &msg) {
  static int32_t i = 0;
  if (first_time_) {
    last_time_ = std::chrono::steady_clock::now();
    first_time_ = false;
    return;
  }
  i++;
  window_points_ += msg->width;
  if (i % window_ == 0) {
    curr_time_ = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = curr_time_ - last_time_;
    last_time_ = curr_time_;
    ROS_INFO("average hz: %6.3f average width: %6ld, window: %d", window_ / diff.count(), window_points_ / window_,
             window_);
    window_points_ = 0;
  }
}

void ROSDemo::subscribePacket(const seyond::SeyondScan::ConstPtr &msg) {
  if (is_print_packet_size_) printPacketSize(msg);
  if (is_print_packet_loss_rate_) printPacketLossRate(msg);
}

void ROSDemo::printPacketSize(const seyond::SeyondScan::ConstPtr &msg) {
  packet_size_ += msg->size;
  if (msg->is_last_scan) {
    ROS_INFO("packet size per frame: %d", packet_size_);
    packet_size_ = 0;
  }
}

void ROSDemo::printPacketLossRate(const seyond::SeyondScan::ConstPtr &msg) {
  packet_size_ += msg->size;
  for (const auto &packet : msg->packets) {
    received_packets_count_++;
    const int8_t *inno_data_pkt = reinterpret_cast<const int8_t *>(packet.data.data());
    // common(26) + idx(8) + sub_idx(2) = 36
    std::memcpy(&last_sub_seq_, inno_data_pkt + 36, sizeof(last_sub_seq_));
    if (cur_start_sub_seq_ < 0) {
      ROS_INFO("Received first packet!");
      cur_start_sub_seq_ = last_sub_seq_;
      continue;
    }

    if (last_sub_seq_ < cur_start_sub_seq_) {
      total_expected_packets_count_ += 65535 - cur_start_sub_seq_ + 1;
      cur_start_sub_seq_ = last_sub_seq_;
      total_expected_packets_count_ += cur_start_sub_seq_;
      continue;
    }

    if (received_packets_count_ % 20000 == 0) {
      total_expected_packets_count_ += last_sub_seq_ - cur_start_sub_seq_;
      cur_start_sub_seq_ = last_sub_seq_;
      double packet_loss_rate =
          1.0 - static_cast<double>(received_packets_count_) / static_cast<double>(total_expected_packets_count_ + 1);
      ROS_INFO("Total packet loss rate: %.5lf %% [%d/%d]", packet_loss_rate * 100.0, received_packets_count_,
               total_expected_packets_count_ + 1);
    }
  }
}
