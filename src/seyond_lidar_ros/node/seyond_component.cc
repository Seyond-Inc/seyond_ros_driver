/**
 *  Copyright (C) 2024 - Seyond Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "src/driver/ros2_driver_adapter.hpp"

namespace seyond {

class SeyondDriver : public rclcpp::Node {
 public:
  explicit SeyondDriver(const rclcpp::NodeOptions& options) : Node("seyond_node", options) {
    this->declare_parameter<std::string>("config_path", "");
    ros_driver_ptr_ = std::make_shared<ROSNode>();
    std::thread(&SeyondDriver::Init, this).detach();
  }

  ~SeyondDriver() {
    ros_driver_ptr_.reset();
  }

  void Init() {
    // Wait for a short time to ensure the node is fully initialized
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ros_driver_ptr_->init(this->shared_from_this());
    ros_driver_ptr_->start();
  }

 private:
  std::shared_ptr<ROSNode> ros_driver_ptr_;
};

}  // namespace seyond

RCLCPP_COMPONENTS_REGISTER_NODE(seyond::SeyondDriver)
