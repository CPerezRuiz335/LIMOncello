#pragma once

#include <algorithm>
#include <functional> 

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include "Core/Imu.hpp"
#include "Core/State.hpp"
#include "Utils/PCL.hpp"
#include "Utils/Config.hpp"


Imu fromROS(const sensor_msgs::msg::Imu::ConstSharedPtr& in) {
  Imu out;
  out.stamp = rclcpp::Time(in->header.stamp).seconds();

  out.ang_vel(0) = in->angular_velocity.x;
  out.ang_vel(1) = in->angular_velocity.y;
  out.ang_vel(2) = in->angular_velocity.z;

  out.lin_accel(0) = in->linear_acceleration.x;
  out.lin_accel(1) = in->linear_acceleration.y;
  out.lin_accel(2) = in->linear_acceleration.z;

  tf2::fromMsg(in->orientation, out.q);

  return out;
}

void fromROS(const sensor_msgs::msg::PointCloud2& msg, PointCloudT& raw) {

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

sensor_msgs::msg::PointCloud2 toROS(const PointCloudT::Ptr& cloud) {
  
  sensor_msgs::msg::PointCloud2 out;
  pcl::toROSMsg(*cloud, out);
  out.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  out.header.frame_id = Config::getInstance().frames.world;

  return out;
}

nav_msgs::msg::Odometry toROS(State& state) {

  Config& cfg = Config::getInstance();

  nav_msgs::msg::Odometry out;

  out.pose.pose.position    = tf2::toMsg(state.p());
  out.pose.pose.orientation = tf2::toMsg(state.quat());

  out.twist.twist.linear.x = state.v()(0);
  out.twist.twist.linear.y = state.v()(1);
  out.twist.twist.linear.z = state.v()(2);

  out.twist.twist.angular.x = state.w(0) - state.b_w()(0);
  out.twist.twist.angular.y = state.w(1) - state.b_w()(1);
  out.twist.twist.angular.z = state.w(2) - state.b_w()(2);

  out.header.frame_id = cfg.frames.world;
  out.child_frame_id = cfg.frames.body;
  out.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

  return out;
}


geometry_msgs::msg::TransformStamped toTF(State& state) {
  Config& cfg = Config::getInstance();

  geometry_msgs::msg::TransformStamped tf_msg;

  tf_msg.transform.translation.x = state.p().x();
  tf_msg.transform.translation.y = state.p().y();
  tf_msg.transform.translation.z = state.p().z();
  
  tf_msg.transform.rotation.x = state.quat().x();
  tf_msg.transform.rotation.y = state.quat().y();
  tf_msg.transform.rotation.z = state.quat().z();
  tf_msg.transform.rotation.w = state.quat().w();
 
  tf_msg.header.stamp = rclcpp::Time(state.stamp);
  tf_msg.header.frame_id = cfg.frames.world;
  tf_msg.child_frame_id = cfg.frames.body;

  return tf_msg;
}


void fill_config(Config& cfg, rclcpp::Node* n) {
  n->get_parameter("verbose", cfg.verbose);
  n->get_parameter("debug",   cfg.debug);
  
  // TOPICS
  n->get_parameter("topics.input.lidar",  cfg.topics.input.lidar);
  n->get_parameter("topics.input.imu",    cfg.topics.input.imu);
  n->get_parameter("topics.input.stop_ioctree_update", cfg.topics.input.stop_ioctree_update);
  n->get_parameter("topics.output.state", cfg.topics.output.state);
  n->get_parameter("topics.output.frame", cfg.topics.output.frame);
  
  // FRAMES
  n->get_parameter("frames.world", cfg.frames.world);
  n->get_parameter("frames.body", cfg.frames.body);
  n->get_parameter("frames.tf_pub", cfg.frames.tf_pub);

  // SENSORS
  n->get_parameter("sensors.lidar.type",         cfg.sensors.lidar.type);
  n->get_parameter("sensors.lidar.end_of_sweep", cfg.sensors.lidar.end_of_sweep);
  n->get_parameter("sensors.imu.hz",             cfg.sensors.imu.hz);
  n->get_parameter("sensors.time_offset", cfg.sensors.time_offset);
  n->get_parameter("sensors.TAI_offset",  cfg.sensors.TAI_offset);

  n->get_parameter("sensors.calibration.gravity", cfg.sensors.calibration.gravity);
  n->get_parameter("sensors.calibration.accel",         cfg.sensors.calibration.accel);
  n->get_parameter("sensors.calibration.gyro",          cfg.sensors.calibration.gyro);
  n->get_parameter("sensors.calibration.time",          cfg.sensors.calibration.time);

  // SENSORS - EXTRINSICS (imu2baselink)
  {
    std::vector<double> tmp;
    n->get_parameter("sensors.extrinsics.imu2baselink.t", tmp);
    cfg.sensors.extrinsics.imu2baselink_T.setIdentity();
    cfg.sensors.extrinsics.imu2baselink_T.translate(Eigen::Vector3d(tmp[0], tmp[1], tmp[2]));
  }

  {
    std::vector<double> tmp;
    n->get_parameter("sensors.extrinsics.imu2baselink.R", tmp);
    Eigen::Matrix3d R_imu = (
      Eigen::AngleAxisd(tmp[0] * M_PI / 180.0, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(tmp[1] * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(tmp[2] * M_PI / 180.0, Eigen::Vector3d::UnitZ())
    ).toRotationMatrix();
    cfg.sensors.extrinsics.imu2baselink_T.rotate(R_imu);
  }

  // SENSORS - EXTRINSICS (lidar2baselink)
  {
    std::vector<double> tmp;
    n->get_parameter("sensors.extrinsics.lidar2baselink.t", tmp);
    cfg.sensors.extrinsics.lidar2baselink_T.setIdentity();
    cfg.sensors.extrinsics.lidar2baselink_T.translate(Eigen::Vector3d(tmp[0], tmp[1], tmp[2]));
  }

  {
    std::vector<double> tmp;
    n->get_parameter("sensors.extrinsics.lidar2baselink.R", tmp);
    Eigen::Matrix3d R_lidar = (
      Eigen::AngleAxisd(tmp[0] * M_PI / 180.0, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(tmp[1] * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(tmp[2] * M_PI / 180.0, Eigen::Vector3d::UnitZ())
    ).toRotationMatrix();

    cfg.sensors.extrinsics.lidar2baselink_T.rotate(R_lidar);
  }

  n->get_parameter("sensors.extrinsics.gravity", cfg.sensors.extrinsics.gravity);

  // SENSORS - INTRINSICS
  {
    std::vector<double> tmp;
    n->get_parameter("sensors.intrinsics.accel_bias", tmp);
    cfg.sensors.intrinsics.accel_bias = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]);
  }

  {
    std::vector<double> tmp;
    n->get_parameter("sensors.intrinsics.gyro_bias", tmp);
    cfg.sensors.intrinsics.gyro_bias = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]);
  }

  {
    std::vector<double> tmp;
    n->get_parameter("sensors.intrinsics.sm", tmp);
    cfg.sensors.intrinsics.sm << tmp[0], tmp[1], tmp[2],
                                 tmp[3], tmp[4], tmp[5],
                                 tmp[6], tmp[7], tmp[8];
  }

  // FILTERS
  {
    std::vector<double> tmp;
    n->get_parameter("filters.voxel_grid.leaf_size", tmp);
    cfg.filters.voxel_grid.leaf_size = Eigen::Vector4d(tmp[0], tmp[1], tmp[2], 1.);

    n->get_parameter_or("filters.crop_box.active", cfg.filters.crop_box.active, false);
    std::vector<double> min_pt_tmp;
    n->get_parameter_or("filters.crop_box.min_pt", min_pt_tmp, {-1.0, -1.0, -1.0});
    cfg.filters.crop_box.min_pt = Eigen::Vector3f(min_pt_tmp[0], min_pt_tmp[1], min_pt_tmp[2]);
    std::vector<double> max_pt_tmp;
    n->get_parameter_or("filters.crop_box.max_pt", max_pt_tmp, {1.0, 1.0, 1.0});
    cfg.filters.crop_box.max_pt = Eigen::Vector3f(max_pt_tmp[0], max_pt_tmp[1], max_pt_tmp[2]);

    n->get_parameter("filters.min_distance.active", cfg.filters.min_distance.active);
  }
  n->get_parameter("filters.min_distance.value",  cfg.filters.min_distance.value);

  n->get_parameter("filters.fov.active", cfg.filters.fov.active);
  n->get_parameter("filters.fov.value",  cfg.filters.fov.value);
  cfg.filters.fov.value *= M_PI / 360.0;

  n->get_parameter("filters.rate_sampling.active", cfg.filters.rate_sampling.active);
  n->get_parameter("filters.rate_sampling.value",  cfg.filters.rate_sampling.value);

  // IKFoM
  n->get_parameter("IKFoM.query_iters",         cfg.ikfom.query_iters);
  n->get_parameter("IKFoM.max_iters",           cfg.ikfom.max_iters);
  n->get_parameter("IKFoM.tolerance",           cfg.ikfom.tolerance);
  n->get_parameter("IKFoM.lidar_noise",         cfg.ikfom.lidar_noise);
  n->get_parameter("IKFoM.covariance.gyro",       cfg.ikfom.covariance.gyro);
  n->get_parameter("IKFoM.covariance.accel",      cfg.ikfom.covariance.accel);
  n->get_parameter("IKFoM.covariance.bias_gyro",  cfg.ikfom.covariance.bias_gyro);
  n->get_parameter("IKFoM.covariance.bias_accel", cfg.ikfom.covariance.bias_accel);
  n->get_parameter("IKFoM.plane.points",          cfg.ikfom.plane.points);
  n->get_parameter("IKFoM.plane.max_sqrt_dist",   cfg.ikfom.plane.max_sqrt_dist);
  n->get_parameter("IKFoM.plane.plane_threshold", cfg.ikfom.plane.plane_threshold);

  // iOctree
  n->get_parameter("iOctree.downsample",  cfg.ioctree.downsample);
  n->get_parameter("iOctree.bucket_size", cfg.ioctree.bucket_size);
  n->get_parameter("iOctree.min_extent",  cfg.ioctree.min_extent);
}


