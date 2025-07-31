/**
 *  Copyright (C) 2025 - Seyond Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#pragma once

#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float64.hpp>
#include <yaml-cpp/yaml.h>

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "driver_wrapper.hpp"
#include "yaml_tools.hpp"
#include "seyond/msg/seyond_packet.hpp"
#include "seyond/msg/seyond_scan.hpp"
#include "src/multi_fusion/ros2_multi_fusion.hpp"


#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("seyond"), __VA_ARGS__)
#define ROS_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("seyond"), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("seyond"), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("seyond"), __VA_ARGS__)
#define ROS_FATAL(...) RCLCPP_FATAL(rclcpp::get_logger("seyond"), __VA_ARGS__)

sensor_msgs::msg::PointCloud2 toRosMsg(const seyond::LidarPointCloud<seyond::LidarPoint> &pc, const std::string& frame_id) {
  sensor_msgs::msg::PointCloud2 ros_msg;
  int fields = 6;  // x, y, z, intensity, ring, timestamp
  ros_msg.fields.clear();
  ros_msg.fields.reserve(fields);
  ros_msg.height = 1;
  ros_msg.width = pc.points.size();

  int offset = 0;
  offset = addPointField(ros_msg, "x", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "y", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "z", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "ring", 1, sensor_msgs::msg::PointField::UINT16, offset);
  offset = addPointField(ros_msg, "timestamp", 1, sensor_msgs::msg::PointField::FLOAT64, offset);

  ros_msg.point_step = offset;
  ros_msg.row_step = ros_msg.width * ros_msg.point_step;
  // ros_msg.is_dense = rs_msg.is_dense;
  ros_msg.data.resize(ros_msg.row_step);

  sensor_msgs::PointCloud2Iterator<float> iter_x_(ros_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y_(ros_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z_(ros_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity_(ros_msg, "intensity");
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_(ros_msg, "ring");
  sensor_msgs::PointCloud2Iterator<double> iter_timestamp_(ros_msg, "timestamp");

  for (size_t i = 0; i < pc.points.size(); i++) {
    const auto &point = pc.points[i];
    *iter_x_ = point.x;
    *iter_y_ = point.y;
    *iter_z_ = point.z;
    *iter_intensity_ = point.intensity;
    *iter_ring_ = point.scan_id;  // Assuming scan_id is used as ring
    *iter_timestamp_ = point.timestamp;

    ++iter_x_;
    ++iter_y_;
    ++iter_z_;
    ++iter_intensity_;
    ++iter_ring_;
    ++iter_timestamp_;
  }
  ros_msg.header.frame_id = frame_id;
  ros_msg.header.stamp.sec = static_cast<uint32_t>(floor(pc.timestamp));
  ros_msg.header.stamp.nanosec = static_cast<uint32_t>(round((pc.timestamp - ros_msg.header.stamp.sec) * 1e9));

  return ros_msg;
}

class ROSAdapter {
 public:
  ROSAdapter(std::shared_ptr<rclcpp::Node> node_ptr, const seyond::LidarConfig& lidar_config) {
    node_ptr_ = node_ptr;
    ros_driver_ptr_ = std::make_unique<seyond::ROSDriver>(lidar_config);
    inno_scan_msg_ = std::make_unique<seyond::msg::SeyondScan>();
    lidar_config_ = lidar_config;
  }

  ~ROSAdapter() {
    ros_driver_ptr_.reset();
  }

  void init() {
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();
    inno_frame_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_config_.frame_topic, qos);
    ros_driver_ptr_->register_publish_frame_callback(
        std::bind(&ROSAdapter::publishFrame, this, std::placeholders::_1));

    if (lidar_config_.packet_mode || lidar_config_.replay_rosbag) {
      inno_pkt_pub_ = node_ptr_->create_publisher<seyond::msg::SeyondScan>(lidar_config_.packet_topic, 100);
      inno_pkt_sub_ = node_ptr_->create_subscription<seyond::msg::SeyondScan>(
          lidar_config_.packet_topic, 100, std::bind(&ROSAdapter::subscribePacket, this, std::placeholders::_1));
      ros_driver_ptr_->register_publish_packet_callback(std::bind(
          &ROSAdapter::publishPacket, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }
  }

  void start() {
    ros_driver_ptr_->start();
  }

  void stop() {
    ros_driver_ptr_->stop();
  }

 private:
  void subscribePacket(const seyond::msg::SeyondScan::SharedPtr msg) {
    for (const auto& pkt : msg->packets) {
      if (lidar_config_.replay_rosbag && pkt.has_table && !ros_driver_ptr_->anglehv_table_init_) {
        // ros_driver_ptr_->anglehv_table_.resize(pkt.table.size());
        // std::memcpy(ros_driver_ptr_->anglehv_table_.data(), pkt.table.data(), pkt.table.size());
        ros_driver_ptr_->anglehv_table_init_ = true;
      }
      ros_driver_ptr_->parsePacket(pkt.data.data(), pkt.data.size());
    }
  }

  void publishPacket(const uint8_t *pkt, uint64_t pkt_len, bool next_idx) {
    if (next_idx) {
      frame_count_++;
      // inno_scan_msg_->timestamp = timestamp;
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
    if ((frame_count_ == table_send_hz_) && ros_driver_ptr_->anglehv_table_init_) {
      frame_count_ = 0;
      // msg.has_table = true;
      // msg.table.resize(ros_driver_ptr_->anglehv_table_.size());
      // std::memcpy(msg.table.data(), ros_driver_ptr_->anglehv_table_.data(), ros_driver_ptr_->anglehv_table_.size());
    }
    packets_width_++;
    inno_scan_msg_->packets.emplace_back(msg);
  }

  void publishFrame(std::shared_ptr<seyond::LidarPointCloud<seyond::LidarPoint>> frame) {
    inno_frame_pub_->publish(std::move(toRosMsg(*frame, lidar_config_.frame_id)));
  }

 private:
  seyond::LidarConfig lidar_config_;
  std::shared_ptr<rclcpp::Node> node_ptr_;
  std::unique_ptr<seyond::ROSDriver> ros_driver_ptr_;

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

    seyond::Logger::getInstance().setLogLevel(seyond::LogLevel::LOG_LEVEL_DEBUG);
    seyond::Logger::getInstance().setLogCallback(std::bind(&ROSNode::rosLogCallback, std::placeholders::_1,
                                                           std::placeholders::_2, std::placeholders::_3,
                                                           std::placeholders::_4, std::placeholders::_5));
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
    // node_ptr_->get_parameter_or<std::string>("name_value_pairs", lidar_config.name_value_pairs, "");
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

  static void rosLogCallback(enum seyond::LogLevel level, const char *file, int line, const char *function,
                             const char *msg) {
    switch (level) {
      case seyond::LOG_LEVEL_CRITICAL:
        ROS_FATAL("<%s:%d>: %s", file, line, msg);
        break;
      case seyond::LOG_LEVEL_ERROR:
        ROS_ERROR("<%s:%d>: %s", file, line, msg);
        break;
      case seyond::LOG_LEVEL_WARN:
        ROS_WARN("<%s:%d>: %s", file, line, msg);
        break;
      case seyond::LOG_LEVEL_INFO:
        ROS_INFO("<%s:%d>: %s", file, line, msg);
        break;
      case seyond::LOG_LEVEL_DEBUG:
      default:
        ROS_DEBUG("<%s:%d>: %s", file, line, msg);
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
