cmake_minimum_required(VERSION 3.5)
project(seyond)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Ofast)

set(SEYOND_SDK_PATH src/seyond_sdk/)

#-------------------------------------
# Point Type (PointXYZI, PointXYZIT)
#-------------------------------------
set(POINT_TYPE PointXYZIT)

if(${POINT_TYPE} STREQUAL "PointXYZI")
  add_definitions(-DENABLE_XYZI)
elseif(${POINT_TYPE} STREQUAL "PointXYZIT")
  add_definitions(-DENABLE_XYZIT)
endif()


if($ENV{ROS_VERSION} MATCHES "2")
  add_definitions(-DROS2_FOUND)

  if(($ENV{ROS_DISTRO} MATCHES "jazzy"))
    add_definitions(-DROS_JAZZY)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wno-format-extra-args -Wno-format)
  endif()

  # find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(PCL REQUIRED)
  find_package(message_filters REQUIRED)
  find_package(pcl_msgs REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(yaml-cpp REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/SeyondPacket.msg"
    "msg/SeyondScan.msg"
    DEPENDENCIES std_msgs
  )

  # add PCL include PATH
  list(APPEND PCL_INCLUDE_DIRS /opt/ros/$ENV{ROS_DISTRO}/include)

  include_directories(./)

  # seyond_node
  add_executable(seyond_node
    node/seyond_node.cc
    src/driver/driver_lidar.cc
  )

  rosidl_target_interfaces(seyond_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

  target_include_directories(seyond_node
    PRIVATE
    src/driver
    ${SEYOND_SDK_PATH}
    ${SEYOND_SDK_PATH}/apps
    ${SEYOND_SDK_PATH}/src
    ${PCL_INCLUDE_DIRS}
  )

  target_link_directories(seyond_node
    PRIVATE
    ${SEYOND_SDK_PATH}/lib
  )

  target_link_libraries(seyond_node
    innolidarsdkclient
    innolidarsdkcommon
    innolidarutils
    ${PCL_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
  )

  ament_target_dependencies(seyond_node
    rclcpp
    PCL
    std_msgs
    sensor_msgs
    message_filters
    pcl_msgs
    pcl_conversions
  )

  # seyond_test
  add_executable(seyond_test
    node/seyond_test.cc
  )

  rosidl_target_interfaces(seyond_test ${PROJECT_NAME} "rosidl_typesupport_cpp")

  ament_target_dependencies(seyond_test
    rclcpp
    PCL
    std_msgs
    sensor_msgs
  )

  set_target_properties(seyond_node PROPERTIES
    BUILD_WITH_INSTALL_RPATH FALSE
    LINK_FLAGS "-Wl,-rpath,$ORIGIN/")

  # install group
  install(TARGETS
    seyond_node
    seyond_test
    DESTINATION lib/${PROJECT_NAME}
  )
  install(FILES
    launch/start.py
    launch/test.py
    launch/start_with_config.py
    DESTINATION share/${PROJECT_NAME}
  )
  install(FILES
    rviz/rviz2.rviz
    DESTINATION share/${PROJECT_NAME}/rviz
  )
  install(FILES
    config/config.yaml
    DESTINATION share/${PROJECT_NAME}
  )
  install(DIRECTORY
    ${SEYOND_SDK_PATH}/lib/
    DESTINATION lib/${PROJECT_NAME}
  )

  ament_package()
endif($ENV{ROS_VERSION} MATCHES "2")


if($ENV{ROS_VERSION} MATCHES "1")
  add_definitions(-DROS_FOUND)

  set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    roslib
    sensor_msgs
    std_msgs
    message_generation
    message_filters
  )

  find_package(catkin REQUIRED COMPONENTS
    ${${PROJECT_NAME}_CATKIN_DEPS}
    pcl_conversions
  )

  find_package(yaml-cpp REQUIRED)

  add_message_files(
    FILES
    "SeyondPacket.msg"
    "SeyondScan.msg"
  )
  generate_messages(DEPENDENCIES std_msgs)

  SET(SEYOND_SDK_LIB_DIR ${SEYOND_SDK_PATH}/lib/)
  SET(SEYOND_SDK_INC_DIR ${SEYOND_SDK_PATH}/src/)

  include_directories(
    ./
    ${SEYOND_SDK_INC_DIR}
    ${catkin_INCLUDE_DIRS}
  )

  catkin_package(
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
    INCLUDE_DIRS node
  )

  link_directories(
    ${SEYOND_SDK_LIB_DIR}
  )

  add_executable(seyond_node
    node/seyond_node.cc
    src/driver/driver_lidar.cc
  )

  add_dependencies(seyond_node seyond_generate_messages_cpp)


  target_link_libraries(seyond_node
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    innolidarsdkclient
    innolidarsdkcommon
    innolidarutils
  )

  # seyond_test
  add_executable(seyond_test
    node/seyond_test.cc
  )

  add_dependencies(seyond_test seyond_generate_messages_cpp)

  target_link_libraries(seyond_test
    ${catkin_LIBRARIES}
  )


  install(TARGETS
    seyond_node
    seyond_test
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

  install(DIRECTORY
    ${SEYOND_SDK_PATH}/lib/
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

  install(FILES
    launch/start.launch
    launch/test.launch
    launch/start_with_config.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

  install(FILES
    rviz/rviz.rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)

  install(FILES
    config/config.yaml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

endif($ENV{ROS_VERSION} MATCHES "1")
