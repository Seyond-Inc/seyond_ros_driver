# 02_how_to_change_point_type

## 2.1 PointXYZI and PointXYZIT

The driver source code by default uses Seyond's custom point structure, defined as follows:

```c++
struct EIGEN_ALIGN16 PointXYZIT {
  PCL_ADD_POINT4D;
  double timestamp;
  float intensity;
  std::uint8_t flags;
  std::uint8_t elongation;
  std::uint16_t scan_id;
  std::uint16_t scan_idx;
  std::uint8_t is_2nd_return;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
```

In CMakeLists.txt of the project, change the variable POINT_TYPE to use the PointXYZI (pcl::PointXYZI)

Remember to rebuild the project after changing it.

```cmake
#-------------------------------------
# Point Type (PointXYZI, PointXYZIT)
#-------------------------------------
set(POINT_TYPE PointXYZIT)
```

## 2.2 Custom Point Type

Please refer to the definitions in point_types.h.

Focus on and modify the contents of the driver_wrapper.hpp in the src/driver/ dir.

```c++
#ifdef ENABLE_XYZIT
typedef seyond::PointXYZIT SeyondPoint;
#else
typedef pcl::PointXYZI SeyondPoint;
#endif
#ifdef ENABLE_XYZIT
    if constexpr (std::is_same<PointType, const InnoEnXyzPoint *>::value) {
      point.elongation = 0;
    } else if constexpr (std::is_same<PointType, const InnoXyzPoint *>::value) {
      point.elongation = point_ptr->elongation;
    }
    int32_t roi = point_ptr->in_roi == 3 ? (1 << 2) : 0;
    point.scan_id = point_ptr->scan_id;
    point.scan_idx = point_ptr->scan_idx;
    point.flags = point_ptr->channel | roi | (point_ptr->facet << 3);
    point.is_2nd_return = point_ptr->is_2nd_return;
    point.timestamp = point_ptr->ts_10us / ten_us_in_second_c + current_ts_start_;
#endif
```
