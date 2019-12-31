
#include <cmath>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include "utils/ros_comman.h"
#include "utils/Twist.h"
#include "utils/math_utils.h"
#include "utils/TicToc.h"
#include "utils/geometry_utils.h"

#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointType>::ConstPtr PointCloudConstPtr;
typedef pcl::VoxelGrid<PointType> VoxelGrid;
typedef Twist<float> Transform;
typedef  Sophus::SO3f SO3;

inline float rad2deg(float radians)  //radin  to degree
{
  return radians * 180.0 / M_PI;
}

inline float deg2rad(float degrees)
{
  return degrees * M_PI / 180.0;
}
