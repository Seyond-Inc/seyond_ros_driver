#!/bin/bash

set -e

CURRENT_PATH=$(cd $(dirname $0); pwd)
CLIENT_SDK_PATH=${CURRENT_PATH}/src/seyond_lidar_ros/src/seyond_sdk/

echo -e "\n\033[1;32m-- (a). Clean the old build files...\033[0m"
rm -rf ${CURRENT_PATH}/build ${CURRENT_PATH}/install ${CURRENT_PATH}/devel ${CURRENT_PATH}/log
rm -rf ${CURRENT_PATH}/src/CMakeLists.txt

echo -e "\n\033[1;32m-- (b). Build the driver...\033[0m"
if [[ "$ROS_VERSION" -eq 1 ]]; then
  catkin_make install
elif [[ "$ROS_VERSION" -eq 2 ]]; then
  colcon build
else
    echo "Can't find ROS_VERSION or ROS2_VERSION"
fi
echo -e "\n\033[1;32m-- (c). Build Done!\033[0m"
