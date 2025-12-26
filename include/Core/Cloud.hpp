#pragma once  

#include <vector>
#include <algorithm>
#include <execution>

#include <Eigen/Geometry>
#include <boost/circular_buffer.hpp>

#include "Core/State.hpp"
#include "Utils/PCL.hpp"
#include "Utils/Profiler.hpp"
#include "Utils/Config.hpp"


States filter_states(const States& states, const double& start, const double& end) {

  States out(1000); // Always initialize circular buffer !!

  for (const auto& state : states) {
    if (state.stamp >= end)
      continue;

    if (state.stamp >= start)
      out.push_front(state);
    
    if (state.stamp < start) {
      out.push_front(state);
      break;
    }
  }

  return out;
}


PointCloudT::Ptr deskew(const PointCloudT::Ptr& cloud,
                        const State& state,
                        const States& buffer,
                        const double& offset,
                        const double& sweep_time) {
  
PROFC_NODE("deskew")

  auto binary_search = [&](const double& t) {
    if (buffer.empty()) return -1;
    if (t <= buffer.front().stamp) return 0;
    if (t >= buffer.back().stamp)  return (int)(buffer.size() - 1);

    int l = 0, r = buffer.size() - 1;
    while (l < r) {
        int m = l + (r - l + 1) / 2;
        if (buffer[m].stamp <= t)
          l = m;
        else
          r = m - 1;
    }
    return l;
  };


  PointTime point_time = point_time_func();

  PointCloudT::Ptr out(new PointCloudT);
  out->points.resize(cloud->points.size());

  std::vector<int> indices(cloud->points.size());
  std::iota(indices.begin(), indices.end(), 0);

  std::for_each(
    std::execution::par_unseq,
    indices.begin(),
    indices.end(),
    [&](int k) {
      int i_f = binary_search(point_time(cloud->points[k], sweep_time) + offset);

      State X0 = buffer[i_f];
      X0.interpolate_to(point_time(cloud->points[k], sweep_time) + offset);

      Eigen::Isometry3f T0 = (X0.isometry() * X0.L2I_isometry()).cast<float>();
      Eigen::Isometry3f TN = (state.isometry() * state.L2I_isometry()).cast<float>();

      Eigen::Vector3f p = cloud->points[k].getVector3fMap();  
      p = TN.inverse() * T0 * p;

      PointT pt;
      pt.x = p.x();
      pt.y = p.y();
      pt.z = p.z();
      pt.intensity = cloud->points[k].intensity;

      out->points[k] = pt;
    }
  );

  return out;
}


PointCloudT::Ptr filter(const PointCloudT::Ptr& cloud, 
                        const Eigen::Isometry3d& lidar2baselink) {

PROFC_NODE("filter")

  Config& cfg = Config::getInstance();

  PointCloudT::Ptr out(new PointCloudT);

  int index = 0;
  std::copy_if(
    cloud->points.begin(), 
    cloud->points.end(), 
    std::back_inserter(out->points), 
    [&](const PointT& p) mutable {
        bool pass = true;
        Eigen::Vector3f p_ = lidar2baselink.cast<float>() * p.getVector3fMap();

        // Distance filter
        if (cfg.filters.min_distance.active) {
          if (p_.squaredNorm() 
              <= cfg.filters.min_distance.value*cfg.filters.min_distance.value)
              pass = false;
        }

        // Crop box
        if (pass and cfg.filters.crop_box.active) {
          Eigen::Vector3f mn = cfg.filters.crop_box.min.cast<float>();
          Eigen::Vector3f mx = cfg.filters.crop_box.max.cast<float>();
          if ((p_.cwiseMax(mn).cwiseMin(mx).array() == p_.array()).all())
            pass = false;
        }

        // Rate filter
        if (pass and cfg.filters.rate_sampling.active) {
          if (index % cfg.filters.rate_sampling.value != 0)
              pass = false;
        }

        // Field of view filter
        if (pass and cfg.filters.fov.active) {
          if (fabs(atan2(p_.y(), p_.x())) >= cfg.filters.fov.value)
              pass = false;
        }

        ++index; // Increment index

        return pass;
    }
  );

  return out;
}


PointCloudT::Ptr voxel_grid(const PointCloudT::Ptr& cloud) {

PROFC_NODE("downsample")

  Config& cfg = Config::getInstance();

  static pcl::VoxelGrid<PointT> filter;
  filter.setLeafSize(cfg.filters.voxel_grid.leaf_size.cast<float>());

  PointCloudT::Ptr out(new PointCloudT);
  filter.setInputCloud(cloud);
  filter.filter(*out);

  return out;
}
