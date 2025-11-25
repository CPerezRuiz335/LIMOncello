#pragma once

#include <functional>
#include <iostream>
#include <algorithm>

#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include "Utils/Config.hpp"


struct EIGEN_ALIGN16 PointT {
  PCL_ADD_POINT4D;
  float intensity;
  union {
    std::uint32_t t;   // (Ouster) time since beginning of scan in nanoseconds
    float time;        // (Velodyne) time since beginning of scan in seconds
    double timestamp;  // (Hesai) absolute timestamp in seconds
                       // (Livox) absolute timestamp in (seconds * 10e9)
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint32_t, t, t)
  (float, time, time)
  (double, timestamp, timestamp)
)

typedef pcl::PointCloud<PointT> PointCloudT;
typedef std::function<double(const PointT&, const double&)> PointTime;
typedef std::function<bool(const PointT&, const PointT&)> PointTimeComp;


PointTime point_time_func() {
  Config& cfg = Config::getInstance();

  if (cfg.sensors.lidar.type == 0) { // OUSTER
    return cfg.sensors.lidar.end_of_sweep
      ? [] (const PointT& p, const double& sweep_time) { return sweep_time - p.t * 1e-9; }
      : [] (const PointT& p, const double& sweep_time) { return sweep_time + p.t * 1e-9; };

  } else if (cfg.sensors.lidar.type == 1) { // VELODYNE
    return cfg.sensors.lidar.end_of_sweep
      ? [] (const PointT& p, const double& sweep_time) { return sweep_time - (double)p.time; }
      : [] (const PointT& p, const double& sweep_time) { return sweep_time + (double)p.time; };

  } else if (cfg.sensors.lidar.type == 2) { // HESAI
    return [] (const PointT& p, const double& sweep_time) { return p.timestamp; };

  } else if (cfg.sensors.lidar.type > 2) { // LIVOX
    return [] (const PointT& p, const double& sweep_time) { return p.timestamp * 1e-9; };
  } 

  return [](const PointT& p, const double& sweep_time) { return false; };
}

PointTimeComp get_point_time_comp() {
  Config& cfg = Config::getInstance();

  if (cfg.sensors.lidar.type == 0) {
    return cfg.sensors.lidar.end_of_sweep
      ? [](const PointT& p1, const PointT& p2) { return p1.t > p2.t; }
      : [](const PointT& p1, const PointT& p2) { return p1.t < p2.t; };

  } else if (cfg.sensors.lidar.type == 1) {
    return cfg.sensors.lidar.end_of_sweep
      ? [](const PointT& p1, const PointT& p2) { return p1.time > p2.time; }
      : [](const PointT& p1, const PointT& p2) { return p1.time < p2.time; };

  } else if (cfg.sensors.lidar.type > 1) {
    return [](const PointT& p1, const PointT& p2) { return p1.timestamp < p2.timestamp; };
  } 

  return [](const PointT& p1, const PointT& p2) { return false; };
}


void min_at_front_max_at_back(PointCloudT::Ptr& cloud) {
  cloud->is_dense = false;
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

  auto minmax = std::minmax_element(cloud->points.begin(),
                                    cloud->points.end(), 
                                    get_point_time_comp());

  if (minmax.first != cloud->points.begin())
    std::iter_swap(minmax.first, cloud->points.begin());

  if (minmax.second != cloud->points.end() - 1)
    std::iter_swap(minmax.second, cloud->points.end() - 1);
}

