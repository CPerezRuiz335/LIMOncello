#pragma once

#include <algorithm>
#include <functional> 

#include <Eigen/Dense>

#include <ros/ros.h>

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>

#include "Core/Imu.hpp"
#include "Core/State.hpp"
#include "Utils/PCL.hpp"
#include "Utils/Config.hpp"


Imu fromROS(const sensor_msgs::Imu::ConstPtr& in) {
  Imu out;
  out.stamp = in->header.stamp.toSec();

  out.ang_vel(0) = in->angular_velocity.x;
  out.ang_vel(1) = in->angular_velocity.y;
  out.ang_vel(2) = in->angular_velocity.z;

  out.lin_accel(0) = in->linear_acceleration.x;
  out.lin_accel(1) = in->linear_acceleration.y;
  out.lin_accel(2) = in->linear_acceleration.z;

  tf2::fromMsg(in->orientation, out.q);

  return out;
}

void fromROS(const sensor_msgs::PointCloud2& msg, PointCloudT& raw) {

PROFC_NODE("PointCloud2 to pcl")

  Config& cfg = Config::getInstance();

  pcl::fromROSMsg(msg, raw);

  raw.is_dense = false;
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(raw, raw, indices);

  auto minmax = std::minmax_element(raw.points.begin(),
                                    raw.points.end(), 
                                    get_point_time_comp());

  if (minmax.first != raw.points.begin())
    std::iter_swap(minmax.first, raw.points.begin());

  if (minmax.second != raw.points.end() - 1)
    std::iter_swap(minmax.second, raw.points.end() - 1);
}

sensor_msgs::PointCloud2 toROS(const PointCloudT::Ptr& cloud) {
  
  sensor_msgs::PointCloud2 out;
  pcl::toROSMsg(*cloud, out);
  out.header.stamp = ros::Time::now();
  out.header.frame_id = Config::getInstance().topics.frame_id;

  return out;
}

nav_msgs::Odometry toROS(State& state) {

  Config& cfg = Config::getInstance();

  nav_msgs::Odometry out;

  // Pose/Attitude
  out.pose.pose.position    = tf2::toMsg(state.p());
  out.pose.pose.orientation = tf2::toMsg(state.quat());

  // Twist
  out.twist.twist.linear.x = state.v()(0);
  out.twist.twist.linear.y = state.v()(1);
  out.twist.twist.linear.z = state.v()(2);

  out.twist.twist.angular.x = state.w(0) - state.b_w()(0);
  out.twist.twist.angular.y = state.w(1) - state.b_w()(1);
  out.twist.twist.angular.z = state.w(2) - state.b_w()(2);

  out.header.frame_id = Config::getInstance().topics.frame_id;
  out.header.stamp = ros::Time::now();

  return out;
}


// Function to fill configuration using ROS NodeHandle
void fill_config(Config& cfg, ros::NodeHandle& nh) {

  nh.getParam("verbose", cfg.verbose);
  nh.getParam("debug",   cfg.debug);

  // TOPICS
  nh.getParam("topics/input/lidar",  cfg.topics.input.lidar);
  nh.getParam("topics/input/imu",    cfg.topics.input.imu);
  nh.getParam("topics/output/state", cfg.topics.output.state);
  nh.getParam("topics/output/frame", cfg.topics.output.frame);
  nh.getParam("topics/frame_id",     cfg.topics.frame_id);


  // SENSORS
  nh.getParam("sensors/lidar/type",          cfg.sensors.lidar.type);
  nh.getParam("sensors/lidar/end_of_sweep",  cfg.sensors.lidar.end_of_sweep);
  nh.getParam("sensors/imu/hz",              cfg.sensors.imu.hz);

  nh.getParam("sensors/calibration/gravity_align", cfg.sensors.calibration.gravity_align);
  nh.getParam("sensors/calibration/accel",         cfg.sensors.calibration.accel);
  nh.getParam("sensors/calibration/gyro",          cfg.sensors.calibration.gyro);
  nh.getParam("sensors/calibration/time",          cfg.sensors.calibration.time);

  nh.getParam("sensors/time_offset", cfg.sensors.time_offset);
  nh.getParam("sensors/TAI_offset",  cfg.sensors.TAI_offset);


  std::vector<double> tmp;
  nh.getParam("sensors/extrinsics/imu2baselink/t", tmp);

  cfg.sensors.extrinsics.imu2baselink_T.setIdentity();
  cfg.sensors.extrinsics.imu2baselink_T.translate(Eigen::Vector3d(tmp[0], tmp[1], tmp[2]));
  cfg.sensors.extrinsics.imu2baselink.t = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]);

  nh.getParam("sensors/extrinsics/imu2baselink/R", tmp);
  Eigen::Matrix3d R_imu = (
      Eigen::AngleAxisd(tmp[0] * M_PI/180., Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(tmp[1] * M_PI/180., Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(tmp[2] * M_PI/180., Eigen::Vector3d::UnitZ())
    ).toRotationMatrix();

  cfg.sensors.extrinsics.imu2baselink_T.rotate(R_imu);
  cfg.sensors.extrinsics.imu2baselink.roll  = tmp[0] * M_PI/180.;
  cfg.sensors.extrinsics.imu2baselink.pitch = tmp[1] * M_PI/180.;
  cfg.sensors.extrinsics.imu2baselink.yaw   = tmp[2] * M_PI/180.;

  nh.getParam("sensors/extrinsics/lidar2baselink/t", tmp);

  cfg.sensors.extrinsics.lidar2baselink_T.setIdentity();
  cfg.sensors.extrinsics.lidar2baselink_T.translate(Eigen::Vector3d(tmp[0], tmp[1], tmp[2]));
  cfg.sensors.extrinsics.lidar2baselink.t = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]);

  nh.getParam("sensors/extrinsics/lidar2baselink/R", tmp);
  Eigen::Matrix3d R_lidar = (
      Eigen::AngleAxisd(tmp[0] * M_PI/180., Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(tmp[1] * M_PI/180., Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(tmp[2] * M_PI/180., Eigen::Vector3d::UnitZ())
    ).toRotationMatrix();

  cfg.sensors.extrinsics.lidar2baselink_T.rotate(R_lidar);
  cfg.sensors.extrinsics.lidar2baselink.roll  = tmp[0] * M_PI/180.;
  cfg.sensors.extrinsics.lidar2baselink.pitch = tmp[1] * M_PI/180.;
  cfg.sensors.extrinsics.lidar2baselink.yaw   = tmp[2] * M_PI/180.;

  nh.getParam("sensors/extrinsics/gravity", cfg.sensors.extrinsics.gravity);

  nh.getParam("sensors/intrinsics/accel_bias", tmp);
  cfg.sensors.intrinsics.accel_bias = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]);

  nh.getParam("sensors/intrinsics/gyro_bias", tmp);
  cfg.sensors.intrinsics.gyro_bias = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]);

  nh.getParam("sensors/intrinsics/sm", tmp);
  cfg.sensors.intrinsics.sm << tmp[0], tmp[1], tmp[2],
                               tmp[3], tmp[4], tmp[5],
                               tmp[6], tmp[7], tmp[8];


  // FILTERS
  nh.getParam("filters/voxel_grid/active", cfg.filters.voxel_grid.active);
  nh.getParam("filters/voxel_grid/leaf_size", tmp);
  cfg.filters.voxel_grid.leaf_size = Eigen::Vector4d(tmp[0], tmp[1], tmp[2], 1.);

  nh.getParam("filters/min_distance/active", cfg.filters.min_distance.active);
  nh.getParam("filters/min_distance/value",  cfg.filters.min_distance.value);

  nh.getParam("filters/fov/active", cfg.filters.fov.active);
  nh.getParam("filters/fov/value",  cfg.filters.fov.value);
  cfg.filters.fov.value *= M_PI/360.;

  nh.getParam("filters/rate_sampling/active", cfg.filters.rate_sampling.active);
  nh.getParam("filters/rate_sampling/value",  cfg.filters.rate_sampling.value);


  // IKFoM
  nh.getParam("IKFoM/query_iters",         cfg.ikfom.query_iters);
  nh.getParam("IKFoM/max_iters",           cfg.ikfom.max_iters);
  nh.getParam("IKFoM/tolerance",           cfg.ikfom.tolerance);
  nh.getParam("IKFoM/estimate_extrinsics", cfg.ikfom.estimate_extrinsics);
  nh.getParam("IKFoM/lidar_noise",         cfg.ikfom.lidar_noise);

  
  nh.getParam("IKFoM/covariance/gyro",       cfg.ikfom.covariance.gyro);
  nh.getParam("IKFoM/covariance/accel",      cfg.ikfom.covariance.accel);
  nh.getParam("IKFoM/covariance/bias_gyro",  cfg.ikfom.covariance.bias_gyro);
  nh.getParam("IKFoM/covariance/bias_accel", cfg.ikfom.covariance.bias_accel);

  nh.getParam("IKFoM/plane/points",          cfg.ikfom.plane.points);
  nh.getParam("IKFoM/plane/max_sqrt_dist",   cfg.ikfom.plane.max_sqrt_dist);
  nh.getParam("IKFoM/plane/plane_threshold", cfg.ikfom.plane.plane_threshold);


  // iOctree
  nh.getParam("iOctree/order",       cfg.ioctree.order);
  nh.getParam("iOctree/min_extent",  cfg.ioctree.min_extent);
  nh.getParam("iOctree/bucket_size", cfg.ioctree.bucket_size);
  nh.getParam("iOctree/downsample",  cfg.ioctree.downsample);


  // Debug
  nh.getParam("debug/csv_path",  cfg.debug_.csv_path);

}