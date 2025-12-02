/**
 *  Copyright (C) 2025 - Seyond Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#pragma once

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "driver_lidar.h"
#include "yaml_tools.hpp"
#include "seyond/SeyondPacket.h"
#include "seyond/SeyondScan.h"
#include "src/multi_fusion/ros1_multi_fusion.hpp"

class ROSAdapter {
 public:
  ROSAdapter(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<ros::NodeHandle> private_nh,
             const seyond::LidarConfig &lidar_config) {
    nh_ = nh;
    private_nh_ = private_nh;
    driver_ptr_ = std::make_unique<seyond::DriverLidar>(lidar_config);
    inno_scan_msg_ = std::make_unique<seyond::SeyondScan>();
    lidar_config_ = lidar_config;
  }

  ~ROSAdapter() {
    driver_ptr_.reset();
  }

  void init() {
    inno_frame_pub_ = nh_->advertise<sensor_msgs::PointCloud2>(lidar_config_.frame_topic.c_str(), 10);
    driver_ptr_->register_publish_frame_callback(
        std::bind(&ROSAdapter::publishFrame, this, std::placeholders::_1, std::placeholders::_2));

    if (lidar_config_.packet_mode || lidar_config_.replay_rosbag) {
      inno_pkt_pub_ = nh_->advertise<seyond::SeyondScan>(lidar_config_.packet_topic.c_str(), 100);
      inno_pkt_sub_ = nh_->subscribe(lidar_config_.packet_topic.c_str(), 100, &ROSAdapter::subscribePacket, this);
      driver_ptr_->register_publish_packet_callback(std::bind(&ROSAdapter::publishPacket, this, std::placeholders::_1,
                                                              std::placeholders::_2, std::placeholders::_3,
                                                              std::placeholders::_4));
    }

    if (lidar_config_.enable_imu_msg) {
      inno_imu_pub_ = nh_->advertise<sensor_msgs::Imu>("/iv_imu", 10);
      driver_ptr_->register_publish_imu_callback(
          std::bind(&ROSAdapter::publishImu, this, std::placeholders::_1, std::placeholders::_2));
    }
  }

  void start() {
    driver_ptr_->start_lidar();
  }

  void stop() {
    driver_ptr_->stop_lidar();
  }

 private:
  void subscribePacket(const seyond::SeyondScan::ConstPtr &msg) {
    for (const auto &pkt : msg->packets) {
      if (lidar_config_.replay_rosbag && pkt.has_table && !driver_ptr_->anglehv_table_init_) {
        driver_ptr_->anglehv_table_.resize(pkt.table.size());
        std::memcpy(driver_ptr_->anglehv_table_.data(), pkt.table.data(), pkt.table.size());
        driver_ptr_->anglehv_table_init_ = true;
      }
      driver_ptr_->convert_and_parse(reinterpret_cast<const int8_t *>(pkt.data.data()));
    }

    if (msg->is_last_scan) {
      sensor_msgs::PointCloud2 ros_msg;
      driver_ptr_->transform_pointcloud();
      pcl::toROSMsg(*driver_ptr_->pcl_pc_ptr, ros_msg);
      ros_msg.header.frame_id = lidar_config_.frame_id;
      ros_msg.header.stamp = ros::Time().fromSec(msg->timestamp * 1e-6);
      ros_msg.width = driver_ptr_->pcl_pc_ptr->width;
      ros_msg.height = driver_ptr_->pcl_pc_ptr->height;
      inno_frame_pub_.publish(std::move(ros_msg));
      driver_ptr_->pcl_pc_ptr->clear();
    }
  }

  void publishPacket(const int8_t *pkt, uint64_t pkt_len, double timestamp, bool next_idx) {
    if (next_idx) {
      frame_count_++;
      inno_scan_msg_->timestamp = timestamp;
      inno_scan_msg_->size = packets_width_;
      packets_width_ = 0;
      inno_scan_msg_->is_last_scan = true;
      inno_pkt_pub_.publish(std::move(*inno_scan_msg_));
      inno_scan_msg_ = std::make_unique<seyond::SeyondScan>();
    } else if (packets_width_ >= lidar_config_.aggregate_num) {
      inno_scan_msg_->is_last_scan = false;
      inno_scan_msg_->size = packets_width_;
      packets_width_ = 0;
      inno_pkt_pub_.publish(std::move(*inno_scan_msg_));
      inno_scan_msg_ = std::make_unique<seyond::SeyondScan>();
    }
    seyond::SeyondPacket msg;
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

  void publishFrame(const pcl::PointCloud<SeyondPoint> &frame, double timestamp) {
    sensor_msgs::PointCloud2 ros_msg;
    pcl::toROSMsg(frame, ros_msg);
    ros_msg.header.frame_id = lidar_config_.frame_id;
    ros_msg.header.stamp = ros::Time().fromSec(timestamp * 1e-6);
    ros_msg.width = frame.width;
    ros_msg.height = frame.height;
    inno_frame_pub_.publish(std::move(ros_msg));
  }

  void publishImu(std::vector<float> &imu_data, uint64_t imu_ts_ns) {
    sensor_msgs::Imu ros_msg;
    ros_msg.header.frame_id = lidar_config_.frame_id;
    ros_msg.header.stamp.sec = imu_ts_ns / 1000000000;
    ros_msg.header.stamp.nsec = imu_ts_ns % 1000000000;
    ros_msg.linear_acceleration.x = imu_data[0];
    ros_msg.linear_acceleration.y = imu_data[1];
    ros_msg.linear_acceleration.z = imu_data[2];
    ros_msg.angular_velocity.x = imu_data[3];
    ros_msg.angular_velocity.y = imu_data[4];
    ros_msg.angular_velocity.z = imu_data[5];
    inno_imu_pub_.publish(std::move(ros_msg));
  }

 private:
  seyond::LidarConfig lidar_config_;
  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<ros::NodeHandle> private_nh_;
  std::unique_ptr<seyond::DriverLidar> driver_ptr_;

  ros::Publisher inno_frame_pub_;
  ros::Publisher inno_pkt_pub_;
  ros::Publisher inno_imu_pub_;
  ros::Subscriber inno_pkt_sub_;

  std::unique_ptr<seyond::SeyondScan> inno_scan_msg_;

  uint32_t frame_count_{0};
  uint32_t table_send_hz_{20};
  uint32_t packets_width_{0};
};

class ROSNode {
 public:
  void init() {
    nh_ = std::make_shared<ros::NodeHandle>();
    private_nh_ = std::make_shared<ros::NodeHandle>("~");
    std::string yaml_file;
    private_nh_->param("config_path", yaml_file, std::string(""));
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
      ros_adapters_[i] = std::make_unique<ROSAdapter>(nh_, private_nh_, lidar_configs_[i]);
      ros_adapters_[i]->init();
    }

    if (common_config_.fusion_enable) {
      fusion_ = std::make_unique<seyond::MultiFusion>(nh_, lidar_configs_, common_config_);
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
    private_nh_->param("log_level", common_config_.log_level, std::string("info"));
    common_config_.fusion_enable = false;

    // Parse parameters for ros
    private_nh_->param("replay_rosbag", lidar_config.replay_rosbag, false);
    private_nh_->param("packet_mode", lidar_config.packet_mode, false);
    private_nh_->param("aggregate_num", lidar_config.aggregate_num, 20);
    private_nh_->param("frame_id", lidar_config.frame_id, std::string("seyond"));
    private_nh_->param("frame_topic", lidar_config.frame_topic, std::string("iv_points"));
    private_nh_->param("packet_topic", lidar_config.packet_topic, std::string("iv_packets"));

    // Parse parameters for driver
    private_nh_->param("lidar_name", lidar_config.lidar_name, std::string("seyond"));
    private_nh_->param("lidar_ip", lidar_config.lidar_ip, std::string("172.168.1.10"));
    private_nh_->param("port", lidar_config.port, 8010);
    private_nh_->param("udp_port", lidar_config.udp_port, 8010);
    private_nh_->param("reflectance_mode", lidar_config.reflectance_mode, true);
    private_nh_->param("multiple_return", lidar_config.multiple_return, 1);
    private_nh_->param("enable_falcon_ring", lidar_config.enable_falcon_ring, false);
    private_nh_->param("enable_imu_msg", lidar_config.enable_imu_msg, false);

    private_nh_->param("continue_live", lidar_config.continue_live, false);

    private_nh_->param("pcap_file", lidar_config.pcap_file, std::string(""));
    private_nh_->param("hv_table_file", lidar_config.hv_table_file, std::string(""));
    private_nh_->param("packet_rate", lidar_config.packet_rate, 10000);
    private_nh_->param("file_rewind", lidar_config.file_rewind, 0);

    private_nh_->param("max_range", lidar_config.max_range, 2000.0);  // unit: meter
    private_nh_->param("min_range", lidar_config.min_range, 0.4);     // unit: meter
    private_nh_->param("name_value_pairs", lidar_config.name_value_pairs, std::string(""));
    private_nh_->param("coordinate_mode", lidar_config.coordinate_mode, 3);

    private_nh_->param("transform_enable", lidar_config.transform_enable, false);
    private_nh_->param("x", lidar_config.x, 0.0);
    private_nh_->param("y", lidar_config.y, 0.0);
    private_nh_->param("z", lidar_config.z, 0.0);
    private_nh_->param("pitch", lidar_config.pitch, 0.0);
    private_nh_->param("yaw", lidar_config.yaw, 0.0);
    private_nh_->param("roll", lidar_config.roll, 0.0);
    private_nh_->param("transform_matrix", lidar_config.transform_matrix, std::string(""));

    lidar_configs_.emplace_back(lidar_config);
  }

  void spin() {
    ros::spin();
  }

  static void rosLogCallback(int32_t level, const char *header2, const char *msg) {
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
  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<ros::NodeHandle> private_nh_;

  seyond::CommonConfig common_config_;
  std::vector<seyond::LidarConfig> lidar_configs_;
  std::vector<std::unique_ptr<ROSAdapter>> ros_adapters_;
  std::unique_ptr<seyond::MultiFusion> fusion_;
};
