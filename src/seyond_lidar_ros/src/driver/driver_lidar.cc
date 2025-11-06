/*
 *  Copyright (C) 2025 Seyond Inc.
 *
 *  License: Apache License
 *
 *  $Id$
 */

#include "driver_lidar.h"

#include <assert.h>

#include <thread>
#include <utility>

#include "sdk_common/inno_lidar_api.h"
#include "sdk_common/inno_lidar_other_api.h"
#include "sdk_common/inno_lidar_packet_utils.h"

constexpr uint64_t KBUF_SIZE = 1024 * 1024 * 10;
constexpr double us_in_second_c = 1000000.0;
constexpr double ten_us_in_second_c = 100000.0;

namespace seyond {

std::function<void(int, const char*, const char*)> DriverLidar::ros_log_cb_s_ = nullptr;

static void coordinate_transfer(SeyondPoint *point, int32_t coordinate_mode, float x, float y, float z) {
  switch (coordinate_mode) {
    case 0:
      point->x = x;  // up
      point->y = y;  // right
      point->z = z;  // forward
      break;
    case 1:
      point->x = y;  // right
      point->y = z;  // forward
      point->z = x;  // up
      break;
    case 2:
      point->x = y;  // right
      point->y = x;  // up
      point->z = z;  // forward
      break;
    case 3:
      point->x = z;   // forward
      point->y = -y;  // -right
      point->z = x;   // up
      break;
    case 4:
      point->x = z;  // forward
      point->y = x;  // up
      point->z = y;  // right
      break;
    default:
      // default
      point->x = x;  // up
      point->y = y;  // right
      point->z = z;  // forward
      break;
  }
}

static void coordinate_imu(std::vector<float>& imu_data, int32_t coordinate_mode) {
  if (imu_data.size() < 6) {
    return;
  }
  float tmp;
  switch (coordinate_mode) {
    case 0:
        break;
      case 1:
        tmp = imu_data[0];
        imu_data[0] = imu_data[1];
        imu_data[1] = imu_data[2];
        imu_data[2] = tmp;
        tmp = imu_data[3];
        imu_data[3] = imu_data[4];
        imu_data[4] = imu_data[5];
        imu_data[5] = tmp;
        break;
      case 2:
        tmp = imu_data[0];
        imu_data[0] = imu_data[1];
        imu_data[1] = tmp;
        tmp = imu_data[3];
        imu_data[3] = imu_data[4];
        imu_data[4] = tmp;
        break;
      case 3:
        tmp = imu_data[0];
        imu_data[0] = imu_data[2];
        imu_data[1] = -imu_data[1];
        imu_data[2] = tmp;
        tmp = imu_data[3];
        imu_data[3] = imu_data[5];
        imu_data[4] = -imu_data[4];
        imu_data[5] = tmp;
        break;
      case 4:
        tmp = imu_data[0];
        imu_data[0] = imu_data[2];
        imu_data[2] = imu_data[1];
        imu_data[1] = tmp;
        tmp = imu_data[3];
        imu_data[3] = imu_data[5];
        imu_data[5] = imu_data[4];
        imu_data[4] = tmp;
        break;
      default:
        break;
  }
}

DriverLidar::DriverLidar(const LidarConfig& lidar_config) {
  param_ = lidar_config;
  init_transform_matrix();
  input_parameter_check();

  if (param_.hv_table_file.size() > 0) {
    std::ifstream ifs(param_.hv_table_file, std::ios::binary);
    if (ifs.is_open()) {
      ifs.seekg(0, std::ios::end);
      std::streamsize table_size = ifs.tellg();
      ifs.seekg(0, std::ios::beg);
      anglehv_table_.resize(table_size);
      ifs.read(anglehv_table_.data(), table_size);
      ifs.close();
      if (anglehv_table_.size() > 0) {
        anglehv_table_init_ = true;
      } else {
        inno_log_error("hv_table_file is empty");
      }
    } else {
      inno_log_error("cannot open %s", param_.hv_table_file.c_str());
    }
  }

  data_buffer.resize(KBUF_SIZE);
  pcl_pc_ptr = pcl::PointCloud<SeyondPoint>::Ptr(new pcl::PointCloud<SeyondPoint>());
}

DriverLidar::~DriverLidar() {
  stop_lidar();  // make sure that lidar_handle_ has been closed
  data_buffer.clear();
}

void DriverLidar::init_transform_matrix() {
  if (param_.transform_matrix.empty()) {
    double tmp_yaw, tmp_pitch, tmp_roll;
    if (transform_degree_flag_ || param_.yaw > (2 * M_PI) || param_.yaw < -(2 * M_PI) || param_.pitch > (2 * M_PI) ||
        param_.pitch < -(2 * M_PI) || param_.roll > (2 * M_PI) || param_.roll < -(2 * M_PI)) {
      tmp_yaw = param_.yaw * M_PI / 180;
      tmp_pitch = param_.pitch * M_PI / 180;
      tmp_roll = param_.roll * M_PI / 180;
      inno_log_info("%s: transformation: yaw, pitch, roll in degree", param_.lidar_name.c_str());
    } else {
      tmp_yaw = param_.yaw;
      tmp_pitch = param_.pitch;
      tmp_roll = param_.roll;
    }
    inno_log_info("%s: transformation: x, y, z, yaw, pitch, roll: %.3f %.3f %.3f %.3f %.3f %.3f",
                  param_.lidar_name.c_str(), param_.x, param_.y, param_.z, param_.yaw, param_.pitch, param_.roll);

    Eigen::Vector3f euler_angle(tmp_yaw, tmp_pitch, tmp_roll);
    Eigen::AngleAxisf roll_AA(euler_angle(2), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitch_AA(euler_angle(1), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yaw_AA(euler_angle(0), Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f R = (yaw_AA * pitch_AA * roll_AA).toRotationMatrix();
    Eigen::Vector3f t(param_.x, param_.y, param_.z);
    T_2_0_.block<3, 3>(0, 0) = R;
    T_2_0_.block<3, 1>(0, 3) = t;
  } else {
    std::istringstream iss(param_.transform_matrix);
    std::string item;
    std::vector<float> m_arr(16);
    char comma;
    for (int i = 0; i < 16; i++) {
      iss >> m_arr[i] >> comma;
    }

    T_2_0_ << m_arr[0], m_arr[1], m_arr[2], m_arr[3],
              m_arr[4], m_arr[5], m_arr[6], m_arr[7],
              m_arr[8], m_arr[9], m_arr[10], m_arr[11],
              m_arr[12], m_arr[13], m_arr[14], m_arr[15];
    inno_log_info(
        "%s: transformation matrix:\n %.3f %.3f %.3f %.3f\n %.3f %.3f %.3f %.3f\n %.3f %.3f %.3f %.3f\n %.3f %.3f "
        "%.3f %.3f",
        param_.lidar_name.c_str(), m_arr[0], m_arr[1], m_arr[2], m_arr[3], m_arr[4], m_arr[5], m_arr[6], m_arr[7],
        m_arr[8], m_arr[9], m_arr[10], m_arr[11], m_arr[12], m_arr[13], m_arr[14], m_arr[15]);
  }
}

int32_t DriverLidar::lidar_data_callback_s(int32_t handle, void *ctx, const InnoDataPacket *pkt) {
  DriverLidar *context = reinterpret_cast<DriverLidar *>(ctx);
  assert(handle == context->lidar_handle_);
  return context->lidar_data_callback(pkt);
}

void DriverLidar::lidar_message_callback_s(int32_t handle, void *ctx, uint32_t from_remote, enum InnoMessageLevel level,
                                           enum InnoMessageCode code, const char *error_message) {
  DriverLidar *context = reinterpret_cast<DriverLidar *>(ctx);
  assert(handle == context->lidar_handle_);
  context->lidar_message_callback(from_remote, level, code, error_message);
}

int32_t DriverLidar::lidar_status_callback_s(int32_t handle, void *ctx, const InnoStatusPacket *pkt) {
  DriverLidar *context = reinterpret_cast<DriverLidar *>(ctx);
  assert(handle == context->lidar_handle_);
  return context->lidar_status_callback(pkt);
}

void DriverLidar::init_log_s(std::string &log_limit,
                             const std::function<void(int32_t, const char *, const char *)> &callback) {
  InnoLogLevel log_level_;
  DriverLidar::ros_log_cb_s_ = callback;
  if (log_limit.compare("info") == 0) {
    log_level_ = INNO_LOG_LEVEL_INFO;
  } else if (log_limit.compare("warn") == 0) {
    log_level_ = INNO_LOG_LEVEL_WARNING;
  } else if (log_limit.compare("error") == 0) {
    log_level_ = INNO_LOG_LEVEL_ERROR;
  } else {
    log_level_ = INNO_LOG_LEVEL_INFO;
  }
  inno_lidar_set_log_level(log_level_);
  inno_lidar_set_logs(-1, -1, NULL, 0, 0, lidar_log_callback_s, NULL, NULL, 0, 0, 1);
}

void DriverLidar::lidar_log_callback_s(void *ctx, enum InnoLogLevel level, const char *header1, const char *header2,
                                       const char *msg) {
  DriverLidar::ros_log_cb_s_(static_cast<int32_t>(level), header2, msg);
}

void DriverLidar::start_lidar() {
  bool ret = false;
  if (lidar_handle_ > 0) {
    inno_log_error("lidar_handle_ should <= 0");
    return;
  }

  if (param_.replay_rosbag) {
    // packet rosbag replay
    inno_log_info("waiting for rosbag to replay...");
    return;
  }

  if (!setup_lidar()) {
    inno_log_error("%s, setup_lidar failed!", param_.lidar_name.c_str());
    return;
  }

  is_running_ = true;
  start_check_datacallback_thread();
}

void DriverLidar::stop_lidar() {
  if (lidar_handle_ > 0) {
    (void)inno_lidar_stop(lidar_handle_);
    (void)inno_lidar_close(lidar_handle_);
  }
  current_frame_id_ = -1;
  lidar_handle_ = -1;
  is_running_ = false;
  running_cv_.notify_all();
  if (check_datacallback_thread_.joinable()) {
    check_datacallback_thread_.join();
  }
}

bool DriverLidar::setup_lidar() {
  if (param_.pcap_file.size() > 0) {
    // pcap replay
    if (param_.udp_port < 0) {
      inno_log_error("pcap playback mode, udp_port should be set!");
      return false;
    }

    if (pcap_playback_process() != 0) {
      return false;
    }
  } else {
    // live
    if (lidar_live_process() != 0) {
      return false;
    }
  }

  // set lidar parameters
  if (lidar_parameter_set() != 0) {
    return false;
  }

  return true;
}

int32_t DriverLidar::lidar_parameter_set() {
  int32_t ret = 0;
  ret = set_config_name_value();
  if (ret != 0) {
    inno_log_warning("%s, set config name failed", param_.lidar_name.c_str());
  }

  ret = inno_lidar_set_callbacks(lidar_handle_, lidar_message_callback_s, lidar_data_callback_s,
                                 lidar_status_callback_s, NULL, this);
  if (ret != 0) {
    inno_log_error("%s, inno_lidar_set_callbacks failed!, ret: %d", param_.lidar_name.c_str(), ret);
    return ret;
  }

  return 0;
}

int32_t DriverLidar::lidar_live_process() {
  enum InnoLidarProtocol protocol_;
  // setup read from live
  uint16_t tmp_udp_port = 0;
  if (param_.udp_port >= 0) {
    protocol_ = INNO_LIDAR_PROTOCOL_PCS_UDP;
    tmp_udp_port = param_.udp_port;
  } else {
    protocol_ = INNO_LIDAR_PROTOCOL_PCS_TCP;
  }

  lidar_handle_ =
      inno_lidar_open_live(param_.lidar_name.c_str(), param_.lidar_ip.c_str(), param_.port, protocol_, tmp_udp_port);
  if (lidar_handle_ < 0) {
    inno_log_error("FATAL: Lidar %s invalid handle", param_.lidar_name.c_str());
    return -1;
  }

  int32_t ret = 0;
  // ros always run externally, so we set timeout longer
  ret = inno_lidar_set_config_name_value(lidar_handle_, "LidarClient_Communication/get_conn_timeout_sec", "5.0");
  if (ret != 0) {
    inno_log_error("%s, inno_lidar_set_config_name_value 'get_conn_timeout_sec 5.0' failed %d",
                   param_.lidar_name.c_str(), ret);
  }

  // enable client sdk midorder_fix
  ret = inno_lidar_set_config_name_value(lidar_handle_, "LidarClient_StageClientRead/misorder_correct_enable", "1");
  if (ret != 0) {
    inno_log_warning("%s, inno_lidar_set_config_name_value 'misorder_correct_enable 1' failed %d",
                     param_.lidar_name.c_str(), ret);
  }

  // check lidar status
  char buf[20] = {'\0'};
  ret = inno_lidar_get_attribute_string(lidar_handle_, "enabled", buf, sizeof(buf));

  if (ret != 0) {
    inno_log_error("%s, cannot get lidar status, please check the network connection", param_.lidar_name.c_str());
  } else {
    double enabled = atof(buf);
    if (enabled == 0) {
      inno_log_error("%s, lidar internal server is off, please turn on the server", param_.lidar_name.c_str());
    }
  }

  enum InnoReflectanceMode m =
      param_.reflectance_mode ? INNO_REFLECTANCE_MODE_REFLECTIVITY : INNO_REFLECTANCE_MODE_INTENSITY;
  ret = inno_lidar_set_reflectance_mode(lidar_handle_, m);
  if (ret != 0) {
    inno_log_warning("%s, set_reflectance failed", param_.lidar_name.c_str());
  }

  ret = inno_lidar_set_return_mode(lidar_handle_, (InnoMultipleReturnMode)param_.multiple_return);
  if (ret != 0) {
    inno_log_warning("%s, set_return_mode failed", param_.lidar_name.c_str());
  }

  if (param_.enable_falcon_ring) {
    ret = ret = inno_lidar_set_attribute_string(lidar_handle_, "use_ring_id", "1");
    if (ret != 0) {
      inno_log_warning("%s, set use_ring_id failed", param_.lidar_name.c_str());
    }
    void *converter = inno_lidar_get_ring_id_converter(lidar_handle_);
    ring_id_converter_ = reinterpret_cast<RingIdConverterInterface *>(converter);
    if (ring_id_converter_ == nullptr) {
      inno_log_warning("%s, get ring id converter failed.", param_.lidar_name.c_str());
    }
  }

  return 0;
}

int32_t DriverLidar::pcap_playback_process() {
  InputParam param;
  param.pcap_param.source_type = SOURCE_PCAP;
  strncpy(param.pcap_param.filename, param_.pcap_file.c_str(), param_.pcap_file.length() + 1);
  strncpy(param.pcap_param.lidar_ip, param_.lidar_ip.c_str(), param_.lidar_ip.length() + 1);
  param.pcap_param.data_port = param_.udp_port;
  param.pcap_param.message_port = param_.udp_port;
  param.pcap_param.status_port = param_.udp_port;
  param.pcap_param.play_rate = param_.packet_rate;
  param.pcap_param.rewind = param_.file_rewind;
  inno_log_info("## pcap_file is %s, play_rate is %d, rewind id %d, udp port %d##", param_.pcap_file.c_str(),
                param_.packet_rate, param_.file_rewind, param_.udp_port);
  lidar_handle_ = inno_lidar_open_ctx(param_.lidar_name.c_str(), &param);
  if (lidar_handle_ < 0) {
    inno_log_error("FATAL: Lidar %s invalid handle", param_.lidar_name.c_str());
    return -1;
  }
  return 0;
}

int32_t DriverLidar::set_config_name_value() {
  if (param_.name_value_pairs.size() > 0) {
    char *rest = NULL;
    char *token;
    char *nv = strdup(param_.name_value_pairs.c_str());
    inno_log_info("Use name_value_pairs %s", param_.name_value_pairs.c_str());
    if (nv) {
      for (token = strtok_r(nv, ",", &rest); token != NULL; token = strtok_r(NULL, ",", &rest)) {
        char *eq = strchr(token, '=');
        if (eq) {
          *eq = 0;
          if (inno_lidar_set_config_name_value(lidar_handle_, token, eq + 1) != 0) {
            inno_log_warning("bad name_value pairs %s", nv);
            break;
          }
        } else {
          inno_log_warning("bad name_value pairs %s", nv);
          break;
        }
      }
      free(nv);
    }
  }
  return 0;
}

int32_t DriverLidar::lidar_data_callback(const InnoDataPacket *pkt) {
  is_receive_data_ = true;

  if (current_frame_id_ == -1) {
    current_frame_id_ = pkt->idx;
    inno_log_info("%s, get first frame id %lu", param_.lidar_name.c_str(), current_frame_id_);
    return 0;
  }

  if (CHECK_CO_SPHERE_POINTCLOUD_DATA(pkt->type) && (!anglehv_table_init_)) {
    anglehv_table_.resize(kInnoAngleHVTableMaxSize);
    int32_t ret = inno_lidar_get_anglehv_table(lidar_handle_, reinterpret_cast<InnoDataPacket*>(anglehv_table_.data()));
    if (ret == 0) {
      anglehv_table_init_ = true;
      inno_log_info("%s, Get Generic Compact Table", param_.lidar_name.c_str());
    } else {
      inno_log_error("%s, Get Generic Compact Table failed", param_.lidar_name.c_str());
    }
  }

  bool next_idx_flag = false;
  if (current_frame_id_ != pkt->idx) {
    next_idx_flag = true;
    current_frame_id_ = pkt->idx;
  }

  if (param_.packet_mode) {
    uint64_t pkt_len = sizeof(InnoDataPacket) + pkt->item_number * pkt->item_size;
    packet_publish_cb_(reinterpret_cast<const int8_t *>(pkt), pkt_len, pkt->common.ts_start_us, next_idx_flag);
  } else {
    if (next_idx_flag) {
      transform_pointcloud();
      frame_publish_cb_(*pcl_pc_ptr, pkt->common.ts_start_us);
      pcl_pc_ptr->clear();
    }
    convert_and_parse(pkt);
  }
  return 0;
}

void DriverLidar::transform_pointcloud() {
  if (param_.transform_enable) {
    pcl::transformPointCloud(*pcl_pc_ptr, *pcl_pc_ptr, T_2_0_);
  }
}

void DriverLidar::convert_and_parse(const int8_t *pkt) {
  convert_and_parse(reinterpret_cast<const InnoDataPacket *>(pkt));
}

void DriverLidar::convert_and_parse(const InnoDataPacket *pkt) {
  if (CHECK_SPHERE_POINTCLOUD_DATA(pkt->type)) {
    // convert sphere to xyz
    if (anglehv_table_init_) {
      inno_lidar_convert_to_xyz_pointcloud2(
          pkt, reinterpret_cast<InnoDataPacket *>(&data_buffer[0]), data_buffer.size(), false,
          reinterpret_cast<InnoDataPacket *>(anglehv_table_.data()));
    } else {
      InnoDataPacketUtils::convert_to_xyz_pointcloud(*pkt, reinterpret_cast<InnoDataPacket *>(&data_buffer[0]),
                                                     data_buffer.size(), false, ring_id_converter_, nullptr, false);
    }
    data_packet_parse(reinterpret_cast<InnoDataPacket *>(&data_buffer[0]));
  } else if (CHECK_XYZ_POINTCLOUD_DATA(pkt->type)) {
    data_packet_parse(pkt);
  } else {
    inno_log_error("%s, pkt type %d is not supported", param_.lidar_name.c_str(), pkt->type);
  }
}

void DriverLidar::data_packet_parse(const InnoDataPacket *pkt) {
  // calculate the point timestamp
  current_ts_start_ = pkt->common.ts_start_us / us_in_second_c;
  // adapt different data structures form different lidar
  if (CHECK_EN_XYZ_POINTCLOUD_DATA(pkt->type)) {
    const InnoEnXyzPoint *pt =
      reinterpret_cast<const InnoEnXyzPoint *>(reinterpret_cast<const char *>(pkt) + sizeof(InnoDataPacket));
    point_xyz_data_parse<const InnoEnXyzPoint *>(pkt->use_reflectance, pkt->item_number, pt);
  } else {
    const InnoXyzPoint *pt =
      reinterpret_cast<const InnoXyzPoint *>(reinterpret_cast<const char *>(pkt) + sizeof(InnoDataPacket));
    point_xyz_data_parse<const InnoXyzPoint *>(pkt->use_reflectance, pkt->item_number, pt);
  }
}

template <typename PointType>
void DriverLidar::point_xyz_data_parse(bool is_use_refl, uint32_t point_num, PointType point_ptr) {
  for (uint32_t i = 0; i < point_num; ++i, ++point_ptr) {
    SeyondPoint point;
    if (point_ptr->radius > param_.max_range || point_ptr->radius < param_.min_range) {
      continue;
    }

    if constexpr (std::is_same<PointType, const InnoEnXyzPoint *>::value) {
      point.intensity =
          is_use_refl ? static_cast<float>(point_ptr->reflectance) : static_cast<float>(point_ptr->intensity);
    } else if constexpr (std::is_same<PointType, const InnoXyzPoint *>::value) {
      point.intensity = static_cast<float>(point_ptr->refl);
    }
#ifdef ENABLE_XYZIT
    if constexpr (std::is_same<PointType, const InnoEnXyzPoint *>::value) {
      point.elongation = 0;
      point.scan_id = point_ptr->scan_id;
    } else if constexpr (std::is_same<PointType, const InnoXyzPoint *>::value) {
      point.elongation = point_ptr->elongation;
      if (param_.enable_falcon_ring) {
        point.scan_id = point_ptr->ring_id;
      } else {
        point.scan_id = point_ptr->scan_id;
      }
    }
    int32_t roi = point_ptr->in_roi == 3 ? (1 << 2) : 0;
    point.scan_idx = point_ptr->scan_idx;
    point.flags = point_ptr->channel | roi | (point_ptr->facet << 3);
    point.is_2nd_return = point_ptr->is_2nd_return;
    point.timestamp = point_ptr->ts_10us / ten_us_in_second_c + current_ts_start_;
#endif
    coordinate_transfer(&point, param_.coordinate_mode, point_ptr->x, point_ptr->y, point_ptr->z);
    pcl_pc_ptr->points.push_back(point);
    ++pcl_pc_ptr->width;
    pcl_pc_ptr->height = 1;
  }
}

void DriverLidar::lidar_message_callback(uint32_t from_remote, enum InnoMessageLevel level, enum InnoMessageCode code,
                                          const char *msg) {
  const char *remote = "";
  if (from_remote) {
    remote = "REMOTE-";
  }
  if (level == INNO_MESSAGE_LEVEL_WARNING) {
    inno_log_warning("%s%s level=%d, code=%d, message=%s", remote, inno_log_header_g[level], level, code, msg);
  } else if (level < INNO_MESSAGE_LEVEL_WARNING) {
    inno_log_error("%s%s level=%d, code=%d, message=%s", remote, inno_log_header_g[level], level, code, msg);
  }

  if ((level <= INNO_MESSAGE_LEVEL_CRITICAL && code != INNO_MESSAGE_CODE_LIB_VERSION_MISMATCH) ||
             (code == INNO_MESSAGE_CODE_CANNOT_READ)) {
    fatal_error_ = true;
  }
}

int32_t DriverLidar::lidar_status_callback(const InnoStatusPacket *pkt) {
  // sanity check
  if (!inno_lidar_check_status_packet(pkt, 0)) {
    inno_log_error("%s, corrupted pkt->idx = %" PRI_SIZEU, param_.lidar_name.c_str(), pkt->idx);
    return -1;
  }

  if (param_.enable_imu_msg) {
    InnoStatusPacket tmp_pkt = *pkt;
    inno_lidar_correct_imu_status(pkt, &tmp_pkt);
    std::vector<float> imu_data;
    imu_data.resize(6);

    // only for unit imu data
    imu_data[0] = static_cast<float>(tmp_pkt.sensor_readings.accel_unit_x) / 100000.0f;
    imu_data[1] = static_cast<float>(tmp_pkt.sensor_readings.accel_unit_y) / 100000.0f;
    imu_data[2] = static_cast<float>(tmp_pkt.sensor_readings.accel_unit_z) / 100000.0f;
    imu_data[3] = static_cast<float>(tmp_pkt.sensor_readings.gyro_unit_x) / 100000.0f;
    imu_data[4] = static_cast<float>(tmp_pkt.sensor_readings.gyro_unit_y) / 100000.0f;
    imu_data[5] = static_cast<float>(tmp_pkt.sensor_readings.gyro_unit_z) / 100000.0f;

    // imu_data[0] = static_cast<float>(static_cast<int16_t>(tmp_pkt.sensor_readings.accel_x)) / 16384.0f;
    // imu_data[1] = static_cast<float>(static_cast<int16_t>(tmp_pkt.sensor_readings.accel_y)) / 16384.0f;
    // imu_data[2] = static_cast<float>(static_cast<int16_t>(tmp_pkt.sensor_readings.accel_z)) / 16384.0f;
    // imu_data[3] = static_cast<float>(static_cast<int16_t>(tmp_pkt.sensor_readings.gyro_x)) / 131.0f;
    // imu_data[4] = static_cast<float>(static_cast<int16_t>(tmp_pkt.sensor_readings.gyro_y)) / 131.0f;
    // imu_data[5] = static_cast<float>(static_cast<int16_t>(tmp_pkt.sensor_readings.gyro_z)) / 131.0f;

    // ensure imu data in lidar coordinate before transform
    coordinate_imu(imu_data, param_.coordinate_mode);
    imu_data_publish_cb_(imu_data, pkt->common.ts_start_us);
  }

  static uint64_t cnt = 0;
  if (cnt++ % 100 == 1) {
    constexpr uint64_t buf_size = 2048;
    char buf[buf_size]{0};

    int32_t ret = inno_lidar_printf_status_packet(pkt, buf, buf_size);
    if (ret > 0) {
      inno_log_info("%s, Received status packet #%" PRI_SIZELU ": %s", param_.lidar_name.c_str(), cnt, buf);
    } else {
      inno_log_warning("%s, Received status packet #%" PRI_SIZELU ": errorno: %d", param_.lidar_name.c_str(), cnt, ret);
    }
  }
  return 0;
}

void DriverLidar::input_parameter_check() {
  if (param_.min_range >= param_.max_range) {
    inno_log_error("%s, The maximum range is less than The minimum range", param_.lidar_name.c_str());
  }

  if (param_.max_range < 2.0) {
    inno_log_error("%s, The maximum range is less than the blind spot", param_.lidar_name.c_str());
  }

  if (param_.min_range > 550.0) {
    inno_log_error("%s, The minimum range is greater than the lidar effective distance", param_.lidar_name.c_str());
  }

  if (!(param_.packet_mode) && param_.replay_rosbag) {
    inno_log_warning("%s, The replay_rosbag is only valid in packets mode, turn on packets mode",
                     param_.lidar_name.c_str());
    param_.packet_mode = true;
  }
}

void DriverLidar::start_check_datacallback_thread() {
  check_datacallback_thread_ = std::thread([&]() {
    // connect to lidar
    int32_t start_val = -1;
    do {
      start_val = inno_lidar_start(lidar_handle_);
      if (start_val != 0) {
        inno_log_error("%s, inno_lidar_start failed!", param_.lidar_name.c_str());
        {
          std::unique_lock<std::mutex> lock(running_mutex_);
          running_cv_.wait_for(lock, std::chrono::seconds(10));
        }
      }
    } while (start_val != 0 && param_.continue_live && is_running_);

    // check if the lidar is alive
    while (is_running_) {
      if (is_receive_data_.load()) {
        is_receive_data_ = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
      if (is_receive_data_.load()) {
        is_receive_data_ = false;
        inno_log_info("%s is alive", param_.lidar_name.c_str());
      }
      // fatal error, reconnect
      if (fatal_error_ && param_.continue_live) {
        fatal_error_ = false;
        stop_lidar();
        start_lidar();
        break;
      }
      {
        std::unique_lock<std::mutex> lock(running_mutex_);
        running_cv_.wait_for(lock, std::chrono::seconds(10));
      }
    }
    inno_log_info("%s is stopped", param_.lidar_name.c_str());
  });
}

}  // namespace seyond
