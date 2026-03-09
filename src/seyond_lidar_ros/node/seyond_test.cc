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

#include <signal.h>
#include <memory>
#ifdef ROS_FOUND
#include <ros/ros.h>
#include <ros/package.h>
#include "src/test/ros1_test.hpp"
#elif ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include "src/test/ros2_test.hpp"
#endif

static void shutdown_callback(int sig) {
#ifdef ROS_FOUND
  ros::shutdown();
#elif ROS2_FOUND
  rclcpp::shutdown();
#endif
}


int main(int argc, char *argv[]) {
#ifdef ROS_FOUND
  ros::init(argc, argv, "test", ros::init_options::NoSigintHandler);
#elif ROS2_FOUND
  rclcpp::init(argc, argv);
#endif

  signal(SIGINT, shutdown_callback);

  std::shared_ptr<ROSDemo> ros_driver_ptr = std::make_shared<ROSDemo>();
  ros_driver_ptr->init();
  ros_driver_ptr->spin();
  return 0;
}
