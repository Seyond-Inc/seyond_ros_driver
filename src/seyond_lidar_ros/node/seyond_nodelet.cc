/**
 *  Copyright (C) 2024 - Seyond Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */
#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "src/driver/ros1_driver_adapter.hpp"
namespace seyond {
class SeyondDriver : public nodelet::Nodelet {
 public:
  SeyondDriver() = default;
  ~SeyondDriver() {
    ros_driver_ptr_.reset();
  }
 private:
  virtual void onInit() {
    ros_driver_ptr_ = std::make_shared<ROSNode>();
    nh_ = std::make_shared<ros::NodeHandle>(getNodeHandle());
    private_nh_ = std::make_shared<ros::NodeHandle>(getPrivateNodeHandle());
    ros_driver_ptr_->init(nh_, private_nh_);
    ros_driver_ptr_->start();
  }
 private:
  std::shared_ptr<ROSNode> ros_driver_ptr_;
  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<ros::NodeHandle> private_nh_;
};
}  // namespace seyond
// parameters: class type, base class type
PLUGINLIB_EXPORT_CLASS(seyond::SeyondDriver, nodelet::Nodelet);
