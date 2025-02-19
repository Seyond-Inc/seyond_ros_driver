/**
 *  Copyright (C) 2025 - Seyond Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#pragma once
#include <pcl/point_types.h>

namespace seyond {

struct EIGEN_ALIGN16 PointXYZIT {
  PCL_ADD_POINT4D;
  double timestamp;
  std::uint16_t intensity;
  std::uint8_t flags;
  std::uint8_t elongation;
  std::uint16_t scan_id;
  std::uint16_t scan_idx;
  std::uint8_t is_2nd_return;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace seyond

POINT_CLOUD_REGISTER_POINT_STRUCT(
    seyond::PointXYZIT,
    (float, x, x)(float, y, y)(float, z, z)
    (double, timestamp, timestamp)
    (std::uint16_t, intensity, intensity)
    (std::uint8_t, flags, flags)
    (std::uint8_t, elongation, elongation)
    (std::uint16_t, scan_id, scan_id)
    (std::uint16_t, scan_idx, scan_idx)
    (std::uint8_t, is_2nd_return, is_2nd_return))
