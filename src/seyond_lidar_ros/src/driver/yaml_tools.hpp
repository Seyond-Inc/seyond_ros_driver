/**
 *  Copyright (C) 2025 - Seyond Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */


#pragma once

#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>

#include "driver_lidar.h"

namespace seyond {

class YamlTools {
 public:
  static int32_t parseConfig(std::string& yaml_file, std::vector<LidarConfig>& lidar_configs,
                             CommonConfig& common_config) {
    YAML::Node config_;
    try {
        config_ = YAML::LoadFile(yaml_file);
    } catch (const YAML::BadFile& e) {
        std::cerr << "Failed to load YAML file: " << e.what() << std::endl;
    } catch (const YAML::ParserException& e) {
        std::cerr << "Failed to parse YAML file: " << e.what() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "An error occurred: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "An unknown error occurred." << std::endl;
    }

    if (config_.IsNull()) {
      return -1;
    }

    common_config.log_level = config_["common"]["log_level"].as<std::string>("info");
    common_config.fusion_enable = config_["common"]["fusion_enable"].as<bool>(false);
    common_config.fusion_topic = config_["common"]["fusion_topic"].as<std::string>("/iv_points_fusion");

    if (config_["lidars"]) {
      for (auto lidar_config : config_["lidars"]) {
        LidarConfig tmp_config;
        tmp_config.replay_rosbag = lidar_config["lidar"]["replay_rosbag"].as<bool>(false);
        tmp_config.packet_mode = lidar_config["lidar"]["packet_mode"].as<bool>(false);
        tmp_config.aggregate_num = lidar_config["lidar"]["aggregate_num"].as<uint32_t>(20);

        tmp_config.frame_id = lidar_config["lidar"]["frame_id"].as<std::string>("seyond");
        tmp_config.packet_topic = lidar_config["lidar"]["packet_topic"].as<std::string>("/iv_packets");
        tmp_config.frame_topic = lidar_config["lidar"]["frame_topic"].as<std::string>("/iv_points");

        tmp_config.lidar_name = lidar_config["lidar"]["lidar_name"].as<std::string>("seyond");
        tmp_config.lidar_ip = lidar_config["lidar"]["lidar_ip"].as<std::string>("172.168.1.10");
        tmp_config.port = lidar_config["lidar"]["port"].as<int32_t>(8010);
        tmp_config.udp_port = lidar_config["lidar"]["udp_port"].as<int32_t>(8010);
        tmp_config.reflectance_mode = lidar_config["lidar"]["reflectance_mode"].as<bool>(true);
        tmp_config.multiple_return = lidar_config["lidar"]["multiple_return"].as<int32_t>(1);
        tmp_config.enable_falcon_ring = lidar_config["lidar"]["enable_falcon_ring"].as<bool>(false);

        tmp_config.continue_live = lidar_config["lidar"]["continue_live"].as<bool>(false);

        tmp_config.pcap_file = lidar_config["lidar"]["pcap_file"].as<std::string>("");
        tmp_config.hv_table_file = lidar_config["lidar"]["hv_table_file"].as<std::string>("");
        tmp_config.packet_rate = lidar_config["lidar"]["packet_rate"].as<int32_t>(10000);
        tmp_config.file_rewind = lidar_config["lidar"]["file_rewind"].as<int32_t>(0);

        tmp_config.max_range = lidar_config["lidar"]["max_range"].as<double>(2000.0);
        tmp_config.min_range = lidar_config["lidar"]["min_range"].as<double>(0.4);
        tmp_config.name_value_pairs = lidar_config["lidar"]["name_value_pairs"].as<std::string>("");
        tmp_config.coordinate_mode = lidar_config["lidar"]["coordinate_mode"].as<int32_t>(3);

        tmp_config.transform_enable = lidar_config["lidar"]["transform_enable"].as<bool>(false);
        tmp_config.x = lidar_config["lidar"]["x"].as<double>(0.0);
        tmp_config.y = lidar_config["lidar"]["y"].as<double>(0.0);
        tmp_config.z = lidar_config["lidar"]["z"].as<double>(0.0);
        tmp_config.pitch = lidar_config["lidar"]["pitch"].as<double>(0.0);
        tmp_config.yaw = lidar_config["lidar"]["yaw"].as<double>(0.0);
        tmp_config.roll = lidar_config["lidar"]["roll"].as<double>(0.0);
        tmp_config.transform_matrix = lidar_config["lidar"]["transform_matrix"].as<std::string>("");

        lidar_configs.push_back(tmp_config);
      }
    } else {
      return -1;
    }
    return 0;
  }

  static void printConfig(std::vector<LidarConfig>& lidar_configs) {
    for (auto lidar_config : lidar_configs) {
      inno_log_info(
      "\n\treplay_rosbag: %d\n"
      "\tpacket_mode: %d\n"
      "\taggregate_num: %d\n"
      "\tframe_id: %s\n"
      "\tpacket_topic: %s\n"
      "\tframe_topic: %s\n"
      "\tlidar_name: %s\n"
      "\tlidar_ip: %s\n"
      "\tport: %d\n"
      "\tudp_port: %d\n"
      "\treflectance_mode: %d\n"
      "\tmultiple_return: %d\n"
      "\tenable_falcon_ring: %d\n"
      "\tcontinue_live: %d\n"
      "\tpcap_file: %s\n"
      "\thv_table_file: %s\n"
      "\tpacket_rate: %d\n"
      "\tfile_rewind: %d\n"
      "\tmax_range: %f\n"
      "\tmin_range: %f\n"
      "\tname_value_pairs: %s\n"
      "\tcoordinate_mode: %d\n"
      "\ttransform_enable: %d\n"
      "\tx: %f\n"
      "\ty: %f\n"
      "\tz: %f\n"
      "\tpitch: %f\n"
      "\tyaw: %f\n"
      "\troll: %f\n"
      "\ttransform_matrix: %s\n\n",
      lidar_config.replay_rosbag, lidar_config.packet_mode, lidar_config.aggregate_num,
      lidar_config.frame_id.c_str(), lidar_config.packet_topic.c_str(), lidar_config.frame_topic.c_str(),
      lidar_config.lidar_name.c_str(), lidar_config.lidar_ip.c_str(), lidar_config.port, lidar_config.udp_port,
      lidar_config.reflectance_mode, lidar_config.multiple_return,
      lidar_config.enable_falcon_ring, lidar_config.continue_live,
      lidar_config.pcap_file.c_str(), lidar_config.hv_table_file.c_str(),
      lidar_config.packet_rate, lidar_config.file_rewind, lidar_config.max_range,
      lidar_config.min_range, lidar_config.name_value_pairs.c_str(), lidar_config.coordinate_mode,
      lidar_config.transform_enable, lidar_config.x, lidar_config.y, lidar_config.z, lidar_config.pitch,
      lidar_config.yaw, lidar_config.roll, lidar_config.transform_matrix.c_str());
    }
  }

  static double RadianToDegree(double radian) {
    if (radian > (2.0 * M_PI) || radian < -(2.0 * M_PI)) {
      return radian;
    }
    double degree = radian * 180.0 / M_PI;
    degree = fmod(degree, 360.0);
    if (degree > 180.0) {
        degree -= 360.0;
    } else if (degree < -180.0) {
        degree += 360.0;
    }
    return degree;
  }
};

}  // namespace seyond
