#pragma once  

#include <vector>
#include <algorithm>
#include <execution>

#include <boost/circular_buffer.hpp>

#include "Core/State.hpp"
#include "Utils/PCL.hpp"
#include "Utils/Profiler.hpp"
#include "Utils/Config.hpp"


States filter_states(const States& states, const double& start, const double& end) {

  States out(250); // Always initialize circular buffer !!

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
    int l(0), r(buffer.size()-1);
    
    while (l < r) {
      int m = (l + r) / 2;
      if (buffer[m].stamp == t)
        return m;
      else if (t < buffer[m].stamp)
        r = m - 1;
      else
        l = m + 1;
    }

    return l-1 > 0 ? l-1 : l;
  };


  PointTime point_time = point_time_func();

  PointCloudT::Ptr out(boost::make_shared<PointCloudT>());
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
      X0.predict(point_time(cloud->points[k], sweep_time) + offset);

      Eigen::Affine3f T0 = X0.affine3f() * X0.I2L_affine3f();
      Eigen::Affine3f TN = state.affine3f() * state.I2L_affine3f();

      Eigen::Vector3f p;  
      p << cloud->points[k].x, cloud->points[k].y, cloud->points[k].z;

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


PointCloudT::Ptr process(const PointCloudT::Ptr& cloud) {

PROFC_NODE("filter")


  Config& cfg = Config::getInstance();

  PointCloudT::Ptr out(boost::make_shared<PointCloudT>());

  int index = 0;
  std::copy_if(
    cloud->points.begin(), 
    cloud->points.end(), 
    std::back_inserter(out->points), 
    [&](const PointT& p) mutable {
        bool pass = true;

        // Distance filter
        if (cfg.filters.min_distance.active) {
          if (Eigen::Vector3f(p.x, p.y, p.z).norm() <= cfg.filters.min_distance.value)
              pass = false;
        }

        // Rate filter
        if (cfg.filters.rate_sampling.active) {
          if (index % cfg.filters.rate_sampling.value != 0)
              pass = false;
        }

        // Field of view filter
        if (cfg.filters.fov.active) {
          if (fabs(atan2(p.y, p.x)) >= cfg.filters.fov.value)
              pass = false;
        }

        ++index; // Increment index

        return pass;
    }
  );

  return out;
}


PointCloudT::Ptr voxel_grid(const PointCloudT::Ptr& cloud, 
                            Eigen::Vector4d leaf_size) {

PROFC_NODE("downsample")

  leaf_size(3) = 0.;

  static pcl::VoxelGrid<PointT> filter;
  filter.setLeafSize(leaf_size.cast<float>());

  PointCloudT::Ptr out(boost::make_shared<PointCloudT>());
  filter.setInputCloud(cloud);
  filter.filter(*out);

  return out;
}