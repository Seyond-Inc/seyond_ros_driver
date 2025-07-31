# 02_how_to_change_point_type_cn

## 2.1 PointXYZI和PointXYZIT结构

驱动源码默认使用Seyond自定义的点结构，定义如下:

```c++
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
```

可在项目的CMakeLists.txt中修改变量Point_Type, 可使用PointXYZI结构(pcl::PointXYZI)

修改后，需要重新编译驱动

```cmake
#-------------------------------------
# Point Type (PointXYZI, PointXYZIT)
#-------------------------------------
set(POINT_TYPE PointXYZIT)
```

## 2.2 自定义结构

请参考point_types.h中的定义

关注并修改src/driver/下的 driver_wrapper.hpp文件内容

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
