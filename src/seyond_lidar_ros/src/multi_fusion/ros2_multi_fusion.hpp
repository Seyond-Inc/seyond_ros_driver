/**
 *  Copyright (C) 2025 - Seyond Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#pragma once
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "src/driver/point_types.h"
#include "src/driver/driver_wrapper.hpp"

namespace seyond {

using SyncPolicy2 =
    message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;
using SyncPolicy3 =
    message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2,
                                                    sensor_msgs::msg::PointCloud2>;
using SyncPolicy4 =
    message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2,
                                                    sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;
using SyncPolicy5 =
    message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2,
                                                    sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2,
                                                    sensor_msgs::msg::PointCloud2>;
class MultiFusion {
 private:
  uint32_t lidar_num_;
  std::shared_ptr<rclcpp::Node> node_ptr_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fusion_frame_pub_;
  std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>> lidar_subs_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy2>> sync_2_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy3>> sync_3_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy4>> sync_4_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy5>> sync_5_;

 public:
  explicit MultiFusion(std::shared_ptr<rclcpp::Node> node_ptr, std::vector<seyond::LidarConfig> lidar_configs,
                       seyond::CommonConfig common_config);
  ~MultiFusion();

  void callback_2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud1_msg,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud2_msg);
  void callback_3(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud1_msg,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud2_msg,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud3_msg);
  void callback_4(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud1_msg,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud2_msg,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud3_msg,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud4_msg);
  void callback_5(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud1_msg,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud2_msg,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud3_msg,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud4_msg,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud5_msg);
};

MultiFusion::MultiFusion(std::shared_ptr<rclcpp::Node> node_ptr, std::vector<seyond::LidarConfig> lidar_configs,
                         seyond::CommonConfig common_config) {
  node_ptr_ = node_ptr;
  lidar_num_ = lidar_configs.size();

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.reliable();
  fusion_frame_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(common_config.fusion_topic, qos);

  if (lidar_num_ < 2 || lidar_num_ > 5) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Invalid lidar_num: %u", lidar_num_);
    return;
  }

  lidar_subs_.resize(lidar_num_);
  for (uint32_t i = 0; i < lidar_num_; ++i) {
    lidar_subs_[i] = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
        node_ptr_, lidar_configs[i].frame_topic);
  }

  switch (lidar_num_) {
    case 2:
      sync_2_ = std::make_shared<message_filters::Synchronizer<SyncPolicy2>>(SyncPolicy2(20), *lidar_subs_[0],
                                                                           *lidar_subs_[1]);
      sync_2_->registerCallback(
          std::bind(&MultiFusion::callback_2, this, std::placeholders::_1, std::placeholders::_2));
      break;
    case 3:
      sync_3_ = std::make_shared<message_filters::Synchronizer<SyncPolicy3>>(SyncPolicy3(20), *lidar_subs_[0],
                                                                            *lidar_subs_[1], *lidar_subs_[2]);
      sync_3_->registerCallback(std::bind(&MultiFusion::callback_3, this, std::placeholders::_1, std::placeholders::_2,
                                         std::placeholders::_3));
      break;
    case 4:
      sync_4_ = std::make_shared<message_filters::Synchronizer<SyncPolicy4>>(
          SyncPolicy4(20), *lidar_subs_[0], *lidar_subs_[1], *lidar_subs_[2], *lidar_subs_[3]);
      sync_4_->registerCallback(std::bind(&MultiFusion::callback_4, this, std::placeholders::_1, std::placeholders::_2,
                                         std::placeholders::_3, std::placeholders::_4));
      break;
    case 5:
      sync_5_ = std::make_shared<message_filters::Synchronizer<SyncPolicy5>>(
          SyncPolicy5(20), *lidar_subs_[0], *lidar_subs_[1], *lidar_subs_[2], *lidar_subs_[3], *lidar_subs_[4]);
      sync_5_->registerCallback(std::bind(&MultiFusion::callback_5, this, std::placeholders::_1, std::placeholders::_2,
                                         std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));
      break;
  }
}

MultiFusion::~MultiFusion() {
}

void MultiFusion::callback_2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud1_msg,
                             const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud2_msg) {
  pcl::PointCloud<seyond::PointXYZIT> merged_cloud, cloud1, cloud2;
  pcl::fromROSMsg(*cloud1_msg, cloud1);
  pcl::fromROSMsg(*cloud2_msg, cloud2);

  merged_cloud += cloud1;
  merged_cloud += cloud2;
  sensor_msgs::msg::PointCloud2 merged_cloud_msg;
  pcl::toROSMsg(merged_cloud, merged_cloud_msg);
  merged_cloud_msg.header = cloud1_msg->header;
  fusion_frame_pub_->publish(merged_cloud_msg);
}

void MultiFusion::callback_3(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud1_msg,
                             const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud2_msg,
                             const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud3_msg) {
  pcl::PointCloud<seyond::PointXYZIT> merged_cloud, cloud1, cloud2, cloud3;
  pcl::fromROSMsg(*cloud1_msg, cloud1);
  pcl::fromROSMsg(*cloud2_msg, cloud2);
  pcl::fromROSMsg(*cloud3_msg, cloud3);

  merged_cloud += cloud1;
  merged_cloud += cloud2;
  merged_cloud += cloud3;
  sensor_msgs::msg::PointCloud2 merged_cloud_msg;
  pcl::toROSMsg(merged_cloud, merged_cloud_msg);
  merged_cloud_msg.header = cloud1_msg->header;
  fusion_frame_pub_->publish(merged_cloud_msg);
}

void MultiFusion::callback_4(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud1_msg,
                             const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud2_msg,
                             const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud3_msg,
                             const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud4_msg) {
  pcl::PointCloud<seyond::PointXYZIT> merged_cloud, cloud1, cloud2, cloud3, cloud4;
  pcl::fromROSMsg(*cloud1_msg, cloud1);
  pcl::fromROSMsg(*cloud2_msg, cloud2);
  pcl::fromROSMsg(*cloud3_msg, cloud3);
  pcl::fromROSMsg(*cloud4_msg, cloud4);

  merged_cloud += cloud1;
  merged_cloud += cloud2;
  merged_cloud += cloud3;
  merged_cloud += cloud4;
  sensor_msgs::msg::PointCloud2 merged_cloud_msg;
  pcl::toROSMsg(merged_cloud, merged_cloud_msg);
  merged_cloud_msg.header = cloud1_msg->header;
  fusion_frame_pub_->publish(merged_cloud_msg);
}

void MultiFusion::callback_5(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud1_msg,
                             const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud2_msg,
                             const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud3_msg,
                             const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud4_msg,
                             const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud5_msg) {
  pcl::PointCloud<seyond::PointXYZIT> merged_cloud, cloud1, cloud2, cloud3, cloud4, cloud5;
  pcl::fromROSMsg(*cloud1_msg, cloud1);
  pcl::fromROSMsg(*cloud2_msg, cloud2);
  pcl::fromROSMsg(*cloud3_msg, cloud3);
  pcl::fromROSMsg(*cloud4_msg, cloud4);
  pcl::fromROSMsg(*cloud5_msg, cloud5);

  merged_cloud += cloud1;
  merged_cloud += cloud2;
  merged_cloud += cloud3;
  merged_cloud += cloud4;
  merged_cloud += cloud5;
  sensor_msgs::msg::PointCloud2 merged_cloud_msg;
  pcl::toROSMsg(merged_cloud, merged_cloud_msg);
  merged_cloud_msg.header = cloud1_msg->header;
  fusion_frame_pub_->publish(merged_cloud_msg);
}

}  // namespace seyond
