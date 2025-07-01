/**
 *  Copyright (C) 2025 - Seyond Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */


#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <chrono>

#include "src/driver/point_types.h"
#include "seyond/msg/seyond_scan.hpp"

#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("seyond"), __VA_ARGS__)
#define ROS_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("seyond"), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("seyond"), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("seyond"), __VA_ARGS__)
#define ROS_FATAL(...) RCLCPP_FATAL(rclcpp::get_logger("seyond"), __VA_ARGS__)

class ROSDemo {
 public:
  ROSDemo() = default;
  ~ROSDemo();
  void init();
  void spin();

 private:
  void subscribePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void printFrameStamp(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);
  void printFrameHZ(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

  void subscribePacket(const seyond::msg::SeyondScan::SharedPtr msg);
  void printPacketSize(const seyond::msg::SeyondScan::SharedPtr &msg);
  void printPacketLossRate(const seyond::msg::SeyondScan::SharedPtr &msg);

 private:
  std::shared_ptr<rclcpp::Node> node_ptr_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr inno_frame_sub_{nullptr};
  rclcpp::Subscription<seyond::msg::SeyondScan>::SharedPtr inno_pkt_sub_{nullptr};

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
  node_ptr_ = rclcpp::Node::make_shared("test", rclcpp::NodeOptions()
                                                    .allow_undeclared_parameters(true)
                                                    .automatically_declare_parameters_from_overrides(true)
                                                    .use_intra_process_comms(true));
  node_ptr_->get_parameter_or<std::string>("frame_topic", frame_topic_, "iv_points");
  node_ptr_->get_parameter_or<std::string>("packet_topic", packet_topic_, "iv_packets");
  node_ptr_->get_parameter_or<bool>("stamp", is_print_frame_stamp_, false);
  node_ptr_->get_parameter_or<bool>("hz", is_print_frame_hz_, false);
  node_ptr_->get_parameter_or<int32_t>("window", window_, 30);
  node_ptr_->get_parameter_or<bool>("packet_size", is_print_packet_size_, false);
  node_ptr_->get_parameter_or<bool>("packet_loss_rate", is_print_packet_loss_rate_, false);

  if (window_ <= 0) {
    ROS_ERROR("window must be greater than 0, set to default 30");
    window_ = 30;
  }

  if (is_print_packet_size_) {
    ROS_INFO("please make sure packet_mode is enabled in the driver");
  }

  inno_frame_sub_ = node_ptr_->create_subscription<sensor_msgs::msg::PointCloud2>(
      frame_topic_, 10, std::bind(&ROSDemo::subscribePointCloud, this, std::placeholders::_1));
  inno_pkt_sub_ = node_ptr_->create_subscription<seyond::msg::SeyondScan>(
      packet_topic_, 100, std::bind(&ROSDemo::subscribePacket, this, std::placeholders::_1));
}

void ROSDemo::spin() {
  rclcpp::spin(this->node_ptr_);
}

void ROSDemo::subscribePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (is_print_frame_stamp_) printFrameStamp(msg);
  if (is_print_frame_hz_) printFrameHZ(msg);
}

void ROSDemo::printFrameStamp(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
  ROS_INFO("sec: %d, nanosec: %09d", msg->header.stamp.sec, msg->header.stamp.nanosec);
}

void ROSDemo::printFrameHZ(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
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

void ROSDemo::subscribePacket(const seyond::msg::SeyondScan::SharedPtr msg) {
  if (is_print_packet_size_) printPacketSize(msg);
  if (is_print_packet_loss_rate_) printPacketLossRate(msg);
}

void ROSDemo::printPacketSize(const seyond::msg::SeyondScan::SharedPtr &msg) {
  packet_size_ += msg->size;
  if (msg->is_last_scan) {
    ROS_INFO("packet size per frame: %d", packet_size_);
    packet_size_ = 0;
  }
}

void ROSDemo::printPacketLossRate(const seyond::msg::SeyondScan::SharedPtr &msg) {
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
