/**
 *  Copyright (C) 2025 - Seyond Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#pragma once

#ifdef ROS_JAZZY
#include <pcl_conversions/pcl_conversions/pcl_conversions.h>
#else
#include <pcl_conversions/pcl_conversions.h>
#endif

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>
#include <yaml-cpp/yaml.h>

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "driver_lidar.h"
#include "yaml_tools.hpp"
#include "seyond/msg/seyond_packet.hpp"
#include "seyond/msg/seyond_scan.hpp"
#include "src/multi_fusion/ros2_multi_fusion.hpp"


#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("seyond"), __VA_ARGS__)
#define ROS_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("seyond"), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("seyond"), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("seyond"), __VA_ARGS__)
#define ROS_FATAL(...) RCLCPP_FATAL(rclcpp::get_logger("seyond"), __VA_ARGS__)

class ROSAdapter {
 public:
  ROSAdapter(std::shared_ptr<rclcpp::Node> node_ptr, const seyond::LidarConfig& lidar_config) {
    node_ptr_ = node_ptr;
    driver_ptr_ = std::make_unique<seyond::DriverLidar>(lidar_config);
    inno_scan_msg_ = std::make_unique<seyond::msg::SeyondScan>();
    lidar_config_ = lidar_config;
  }

  ~ROSAdapter() {
    driver_ptr_.reset();
  }

  void init() {
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();
    inno_frame_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_config_.frame_topic, qos);
    driver_ptr_->register_publish_frame_callback(
        std::bind(&ROSAdapter::publishFrame, this, std::placeholders::_1, std::placeholders::_2));

    if (lidar_config_.packet_mode) {
      inno_pkt_pub_ = node_ptr_->create_publisher<seyond::msg::SeyondScan>(lidar_config_.packet_topic, 100);
      inno_pkt_sub_ = node_ptr_->create_subscription<seyond::msg::SeyondScan>(
          lidar_config_.packet_topic, 100, std::bind(&ROSAdapter::subscribePacket, this, std::placeholders::_1));
      driver_ptr_->register_publish_packet_callback(std::bind(&ROSAdapter::publishPacket, this, std::placeholders::_1,
                                                              std::placeholders::_2, std::placeholders::_3,
                                                              std::placeholders::_4));
    }
  }

  void start() {
    driver_ptr_->start_lidar();
  }

  void stop() {
    driver_ptr_->stop_lidar();
  }

 private:
  void subscribePacket(const seyond::msg::SeyondScan::SharedPtr msg) {
    for (const auto& pkt : msg->packets) {
      if (lidar_config_.replay_rosbag && pkt.has_table && !driver_ptr_->anglehv_table_init_) {
        driver_ptr_->anglehv_table_.resize(pkt.table.size());
        std::memcpy(driver_ptr_->anglehv_table_.data(), pkt.table.data(), pkt.table.size());
        driver_ptr_->anglehv_table_init_ = true;
      }
      driver_ptr_->convert_and_parse(reinterpret_cast<const int8_t*>(pkt.data.data()));
    }

    if (msg->is_last_scan) {
      sensor_msgs::msg::PointCloud2 ros_msg;
      driver_ptr_->transform_pointcloud();
      pcl::toROSMsg(*driver_ptr_->pcl_pc_ptr, ros_msg);
      ros_msg.header.frame_id = lidar_config_.frame_id;
      int64_t ts_ns = msg->timestamp * 1000;
      ros_msg.header.stamp.sec = ts_ns / 1000000000;
      ros_msg.header.stamp.nanosec = ts_ns % 1000000000;
      ros_msg.width = driver_ptr_->pcl_pc_ptr->width;
      ros_msg.height = driver_ptr_->pcl_pc_ptr->height;
      inno_frame_pub_->publish(std::move(ros_msg));
      driver_ptr_->pcl_pc_ptr->clear();
    }
  }

  void publishPacket(const int8_t* pkt, uint64_t pkt_len, double timestamp, bool next_idx) {
    if (next_idx) {
      frame_count_++;
      inno_scan_msg_->timestamp = timestamp;
      inno_scan_msg_->size = packets_width_;
      packets_width_ = 0;
      inno_scan_msg_->is_last_scan = true;
      inno_pkt_pub_->publish(std::move(inno_scan_msg_));
      inno_scan_msg_ = std::make_unique<seyond::msg::SeyondScan>();
    } else if (packets_width_ >= lidar_config_.aggregate_num) {
      inno_scan_msg_->is_last_scan = false;
      inno_scan_msg_->size = packets_width_;
      packets_width_ = 0;
      inno_pkt_pub_->publish(std::move(inno_scan_msg_));
      inno_scan_msg_ = std::make_unique<seyond::msg::SeyondScan>();
    }
    seyond::msg::SeyondPacket msg;
    msg.data.resize(pkt_len);
    std::memcpy(msg.data.data(), pkt, pkt_len);
    msg.has_table = false;
    if ((frame_count_ == table_send_hz_) && driver_ptr_->anglehv_table_init_) {
      frame_count_ = 0;
      msg.has_table = true;
      msg.table.resize(driver_ptr_->anglehv_table_.size());
      std::memcpy(msg.table.data(), driver_ptr_->anglehv_table_.data(), driver_ptr_->anglehv_table_.size());
    }
    packets_width_++;
    inno_scan_msg_->packets.emplace_back(msg);
  }

  void publishFrame(const pcl::PointCloud<SeyondPoint>& frame, double timestamp) {
    sensor_msgs::msg::PointCloud2 ros_msg;
    pcl::toROSMsg(frame, ros_msg);
    ros_msg.header.frame_id = lidar_config_.frame_id;
    int64_t ts_ns = timestamp * 1000;
    ros_msg.header.stamp.sec = ts_ns / 1000000000;
    ros_msg.header.stamp.nanosec = ts_ns % 1000000000;
    ros_msg.width = frame.width;
    ros_msg.height = frame.height;
    inno_frame_pub_->publish(std::move(ros_msg));
  }

 private:
  seyond::LidarConfig lidar_config_;
  std::shared_ptr<rclcpp::Node> node_ptr_;
  std::unique_ptr<seyond::DriverLidar> driver_ptr_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inno_frame_pub_{nullptr};
  rclcpp::Publisher<seyond::msg::SeyondScan>::SharedPtr inno_pkt_pub_{nullptr};
  rclcpp::Subscription<seyond::msg::SeyondScan>::SharedPtr inno_pkt_sub_{nullptr};

  std::unique_ptr<seyond::msg::SeyondScan> inno_scan_msg_;

  uint32_t frame_count_;
  uint32_t table_send_hz_{20};
  uint32_t packets_width_;
};

class ROSNode {
 public:
  void init() {
    node_ptr_ = rclcpp::Node::make_shared("seyond", rclcpp::NodeOptions()
                                                        .allow_undeclared_parameters(true)
                                                        .automatically_declare_parameters_from_overrides(true)
                                                        .use_intra_process_comms(true));
    std::string yaml_file;
    node_ptr_->get_parameter_or<std::string>("config_path", yaml_file, "");
    if (!yaml_file.empty()) {
      int32_t ret = seyond::YamlTools::parseConfig(yaml_file, lidar_configs_, common_config_);
      if (ret != 0) {
        ROS_ERROR("Parse config file failed");
        exit(0);
      }
    } else {
      parseParams();
    }

    lidar_num_ = lidar_configs_.size();
    ros_adapters_.resize(lidar_num_);

    seyond::DriverLidar::init_log_s(common_config_.log_level, &ROSNode::rosLogCallback);
    seyond::YamlTools::printConfig(lidar_configs_);

    for (int32_t i = 0; i < lidar_num_; i++) {
      lidar_configs_[i].index = i;
      ros_adapters_[i] = std::make_unique<ROSAdapter>(node_ptr_, lidar_configs_[i]);
      ros_adapters_[i]->init();
    }

    if (common_config_.fusion_enable) {
      fusion_ = std::make_unique<seyond::MultiFusion>(node_ptr_, lidar_configs_, common_config_);
    }
  }

  void start() {
    for (int32_t i = 0; i < lidar_num_; i++) {
      ros_adapters_[i]->start();
    }
  }

  void parseParams() {
    seyond::LidarConfig lidar_config;
    // common
    node_ptr_->get_parameter_or<std::string>("log_level", common_config_.log_level, "info");
    common_config_.fusion_enable = false;

    // Parse parameters for ros
    node_ptr_->get_parameter_or<bool>("replay_rosbag", lidar_config.replay_rosbag, false);
    node_ptr_->get_parameter_or<bool>("packet_mode", lidar_config.packet_mode, false);
    node_ptr_->get_parameter_or<int32_t>("aggregate_num", lidar_config.aggregate_num, 20);
    node_ptr_->get_parameter_or<std::string>("frame_id", lidar_config.frame_id, "seyond");
    node_ptr_->get_parameter_or<std::string>("frame_topic", lidar_config.frame_topic, "iv_points");
    node_ptr_->get_parameter_or<std::string>("packet_topic", lidar_config.packet_topic, "iv_packets");

    // Parse parameters for driver
    node_ptr_->get_parameter_or<std::string>("lidar_name", lidar_config.lidar_name, "seyond");
    node_ptr_->get_parameter_or<std::string>("lidar_ip", lidar_config.lidar_ip, "172.168.1.10");
    node_ptr_->get_parameter_or<int32_t>("port", lidar_config.port, 8010);
    node_ptr_->get_parameter_or<int32_t>("udp_port", lidar_config.udp_port, 8010);
    node_ptr_->get_parameter_or<bool>("reflectance_mode", lidar_config.reflectance_mode, true);
    node_ptr_->get_parameter_or<int32_t>("multiple_return", lidar_config.multiple_return, 1);

    node_ptr_->get_parameter_or<bool>("continue_live", lidar_config.continue_live, false);

    node_ptr_->get_parameter_or<std::string>("pcap_file", lidar_config.pcap_file, "");
    node_ptr_->get_parameter_or<std::string>("hv_table_file", lidar_config.hv_table_file, "");
    node_ptr_->get_parameter_or<int32_t>("packet_rate", lidar_config.packet_rate, 10000);
    node_ptr_->get_parameter_or<int32_t>("file_rewind", lidar_config.file_rewind, 0);

    node_ptr_->get_parameter_or<double>("max_range", lidar_config.max_range, 2000.0);  // unit: meter
    node_ptr_->get_parameter_or<double>("min_range", lidar_config.min_range, 0.4);     // unit: meter
    node_ptr_->get_parameter_or<std::string>("name_value_pairs", lidar_config.name_value_pairs, "");
    node_ptr_->get_parameter_or<int32_t>("coordinate_mode", lidar_config.coordinate_mode, 3);

    node_ptr_->get_parameter_or<bool>("transform_enable", lidar_config.transform_enable, false);
    node_ptr_->get_parameter_or<double>("x", lidar_config.x, 0.0);
    node_ptr_->get_parameter_or<double>("y", lidar_config.y, 0.0);
    node_ptr_->get_parameter_or<double>("z", lidar_config.z, 0.0);
    node_ptr_->get_parameter_or<double>("pitch", lidar_config.pitch, 0.0);
    node_ptr_->get_parameter_or<double>("yaw", lidar_config.yaw, 0.0);
    node_ptr_->get_parameter_or<double>("roll", lidar_config.roll, 0.0);
    node_ptr_->get_parameter_or<std::string>("transform_matrix", lidar_config.transform_matrix, "");

    lidar_configs_.emplace_back(lidar_config);
  }

  void spin() {
    rclcpp::spin(this->node_ptr_);
  }

  static void rosLogCallback(int32_t level, const char* header2, const char* msg) {
    switch (level) {
      case 0:  // INNO_LOG_LEVEL_FATAL
      case 1:  // INNO_LOG_LEVEL_CRITICAL
        ROS_FATAL("%s %s", header2, msg);
        break;
      case 2:  // INNO_LOG_LEVEL_ERROR
      case 3:  // INNO_LOG_LEVEL_TEMP
        ROS_ERROR("%s %s", header2, msg);
        break;
      case 4:  // INNO_LOG_LEVEL_WARNING
      case 5:  // INNO_LOG_LEVEL_DEBUG
        ROS_WARN("%s %s", header2, msg);
        break;
      case 6:  // INNO_LOG_LEVEL_INFO
        ROS_INFO("%s %s", header2, msg);
        break;
      case 7:  // INNO_LOG_LEVEL_TRACE
      case 8:  // INNO_LOG_LEVEL_DETAIL
      default:
        ROS_DEBUG("%s %s", header2, msg);
    }
  }

 private:
  int32_t lidar_num_;
  std::shared_ptr<rclcpp::Node> node_ptr_;

  seyond::CommonConfig common_config_;
  std::vector<seyond::LidarConfig> lidar_configs_;
  std::vector<std::unique_ptr<ROSAdapter>> ros_adapters_;
  std::unique_ptr<seyond::MultiFusion> fusion_;
};
