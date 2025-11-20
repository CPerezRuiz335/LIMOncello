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

  Eigen::Isometry3d T_M_B = state.isometry()
                            * cfg.sensors.extrinsics.imu2baselink.inverse();

  Eigen::Vector3d    p_B = T_M_B.translation();
  Eigen::Quaterniond q_B(T_M_B.linear());

  out.pose.pose.position.x = p_B.x();
  out.pose.pose.position.y = p_B.y();
  out.pose.pose.position.z = p_B.z();
  out.pose.pose.orientation = tf2::toMsg(q_B);

  Eigen::Vector3d v_B = T_M_B.linear().transpose() * state.v();
  out.twist.twist.linear.x = v_B.x();
  out.twist.twist.linear.y = v_B.y();
  out.twist.twist.linear.z = v_B.z();

  Eigen::Vector3d w_B = state.w - state.b_w();
  out.twist.twist.angular.x = w_B.x();
  out.twist.twist.angular.y = w_B.y();
  out.twist.twist.angular.z = w_B.z();

  out.header.frame_id = cfg.topics.frame_id;
  out.child_frame_id  = "base_link";
  out.header.stamp    = ros::Time::now();

  return out;
}


geometry_msgs::TransformStamped toTF(const Eigen::Isometry3d& T,
                                     const std::string& parent,
                                     const std::string& child,
                                     const ros::Time& stamp) {

    geometry_msgs::TransformStamped msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = parent;
    msg.child_frame_id  = child;

    Eigen::Vector3d    p = T.translation();
    Eigen::Quaterniond q(T.linear());

    msg.transform.translation.x = p.x();
    msg.transform.translation.y = p.y();
    msg.transform.translation.z = p.z();
    msg.transform.rotation      = tf2::toMsg(q);

    return msg;
}

void publishTFs(State& state, tf2_ros::TransformBroadcaster& br) {

    Config& cfg = Config::getInstance();
    ros::Time stamp = ros::Time::now();

    Eigen::Isometry3d T_B_I = cfg.sensors.extrinsics.imu2baselink;
    Eigen::Isometry3d T_I_B = T_B_I.inverse();
    Eigen::Isometry3d T_M_B = state.isometry() * T_I_B;
    Eigen::Isometry3d T_B_L = T_B_I * state.L2I_isometry();

    br.sendTransform(toTF(T_M_B, cfg.topics.frame_id, "base_link",  stamp));
    br.sendTransform(toTF(T_B_I, "base_link",         "imu_link",   stamp));
    br.sendTransform(toTF(T_B_L, "base_link",         "lidar_link", stamp));
}


// Function to fill configuration using ROS NodeHandle
void fill_config(Config& cfg, ros::NodeHandle& nh) {

  nh.getParam("verbose", cfg.verbose);
  nh.getParam("debug",   cfg.debug);

  // TOPICS
  nh.getParam("topics/input/lidar",               cfg.topics.input.lidar);
  nh.getParam("topics/input/imu",                 cfg.topics.input.imu);
  nh.getParam("topics/input/stop_ioctree_udate",  cfg.topics.input.stop_ioctree_udate);
  nh.getParam("topics/output/state",              cfg.topics.output.state);
  nh.getParam("topics/output/frame",              cfg.topics.output.frame);
  nh.getParam("topics/frame_id",                  cfg.topics.frame_id);


  // SENSORS
  nh.getParam("sensors/lidar/type",         cfg.sensors.lidar.type);
  nh.getParam("sensors/lidar/end_of_sweep", cfg.sensors.lidar.end_of_sweep);
  nh.getParam("sensors/imu/hz",             cfg.sensors.imu.hz);

  nh.getParam("sensors/calibration/gravity_align", cfg.sensors.calibration.gravity_align);
  nh.getParam("sensors/calibration/accel",         cfg.sensors.calibration.accel);
  nh.getParam("sensors/calibration/gyro",          cfg.sensors.calibration.gyro);
  nh.getParam("sensors/calibration/time",          cfg.sensors.calibration.time);

  nh.getParam("sensors/time_offset", cfg.sensors.time_offset);


  std::vector<double> tmp;
  nh.getParam("sensors/extrinsics/imu2baselink/t", tmp);

  cfg.sensors.extrinsics.imu2baselink.setIdentity();
  cfg.sensors.extrinsics.imu2baselink.translation() = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]);

  nh.getParam("sensors/extrinsics/imu2baselink/R", tmp);
  Eigen::Matrix3d R_imu2baselink = (
      Eigen::AngleAxisd(tmp[0] * M_PI/180., Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(tmp[1] * M_PI/180., Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(tmp[2] * M_PI/180., Eigen::Vector3d::UnitZ())
    ).toRotationMatrix();

  cfg.sensors.extrinsics.imu2baselink.linear() = R_imu2baselink;

  nh.getParam("sensors/extrinsics/lidar2baselink/t", tmp);

  cfg.sensors.extrinsics.lidar2baselink.setIdentity();
  cfg.sensors.extrinsics.lidar2baselink.translation() = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]);

  nh.getParam("sensors/extrinsics/lidar2baselink/R", tmp);
  Eigen::Matrix3d R_lidar = (
      Eigen::AngleAxisd(tmp[0] * M_PI/180., Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(tmp[1] * M_PI/180., Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(tmp[2] * M_PI/180., Eigen::Vector3d::UnitZ())
    ).toRotationMatrix();

  cfg.sensors.extrinsics.lidar2baselink.linear() = R_lidar;

  nh.getParam("sensors/intrinsics/accel_bias", tmp);
  cfg.sensors.intrinsics.accel_bias = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]);

  nh.getParam("sensors/intrinsics/gyro_bias", tmp);
  cfg.sensors.intrinsics.gyro_bias = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]);

  nh.getParam("sensors/intrinsics/sm", tmp);
  cfg.sensors.intrinsics.sm << tmp[0], tmp[1], tmp[2],
                               tmp[3], tmp[4], tmp[5],
                               tmp[6], tmp[7], tmp[8];


  // FILTERS
  nh.getParam("filters/voxel_grid/leaf_size", tmp);
  cfg.filters.voxel_grid.leaf_size = Eigen::Vector4d(tmp[0], tmp[1], tmp[2], 1.);

  nh.getParam("filters/min_distance/active", cfg.filters.min_distance.active);
  nh.getParam("filters/min_distance/value",  cfg.filters.min_distance.value);

  nh.getParam("filters/crop_box/active", cfg.filters.crop_box.active);
  nh.getParam("filters/crop_box/min", tmp);
  cfg.filters.crop_box.min = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]);
  nh.getParam("filters/crop_box/max", tmp);
  cfg.filters.crop_box.max = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]);

  nh.getParam("filters/fov/active", cfg.filters.fov.active);
  nh.getParam("filters/fov/value",  cfg.filters.fov.value);
  cfg.filters.fov.value *= M_PI/360.;

  nh.getParam("filters/rate_sampling/active", cfg.filters.rate_sampling.active);
  nh.getParam("filters/rate_sampling/value",  cfg.filters.rate_sampling.value);


  // IKFoM
  nh.getParam("IKFoM/max_iters",           cfg.ikfom.max_iters);
  nh.getParam("IKFoM/tolerance",           cfg.ikfom.tolerance);
  nh.getParam("IKFoM/lidar_noise",         cfg.ikfom.lidar_noise);
  nh.getParam("IKFoM/estimate_extrinsics", cfg.ikfom.estimate_extrinsics);

  
  nh.getParam("IKFoM/covariance/gyro",        cfg.ikfom.covariance.gyro);
  nh.getParam("IKFoM/covariance/accel",       cfg.ikfom.covariance.accel);
  nh.getParam("IKFoM/covariance/bias_gyro",   cfg.ikfom.covariance.bias_gyro);
  nh.getParam("IKFoM/covariance/bias_accel",  cfg.ikfom.covariance.bias_accel);
  nh.getParam("IKFoM/covariance/initial_cov", cfg.ikfom.covariance.initial_cov);

  nh.getParam("IKFoM/plane/points",          cfg.ikfom.plane.points);
  nh.getParam("IKFoM/plane/max_sqrt_dist",   cfg.ikfom.plane.max_sqrt_dist);
  nh.getParam("IKFoM/plane/plane_threshold", cfg.ikfom.plane.plane_threshold);


  // iOctree
  nh.getParam("iOctree/min_extent",  cfg.ioctree.min_extent);
  nh.getParam("iOctree/bucket_size", cfg.ioctree.bucket_size);
  nh.getParam("iOctree/downsample",  cfg.ioctree.downsample);
}