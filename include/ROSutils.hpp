#pragma once

#include <algorithm>
#include <functional> 
#include <execution>
#include <numeric>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include "livox_interfaces/msg/custom_msg.hpp"
#include "livox_ros_driver/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"

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

// Livox CustomMsg
template <typename MsgT>
static void fromROS_livox(const MsgT& msg, PointCloudT& raw) {
  raw.points.resize(msg.point_num);
  
  std::vector<int> idx(msg.point_num);
  std::iota(idx.begin(), idx.end(), 0);

  std::for_each(
    std::execution::par_unseq,
    idx.begin(), 
    idx.end(),
    [&](int i) {
      const auto& in = msg.points[i];
      PointT& p = raw.points[i];

      p.x = in.x;
      p.y = in.y;
      p.z = in.z;
      p.intensity = (float)in.reflectivity;
      double t_ns = (double)rclcpp::Time(msg.header.stamp).seconds()*1e9
                     + (double)in.offset_time;
      p.timestamp = t_ns;

      return p;
    }
  );
} 

void fromROS(const livox_ros_driver2::msg::CustomMsg& msg, PointCloudT& raw) { 
  fromROS_livox(msg, raw); }
void fromROS(const livox_ros_driver::msg::CustomMsg& msg,  PointCloudT& raw) { 
  fromROS_livox(msg, raw); }
void fromROS(const livox_interfaces::msg::CustomMsg& msg,  PointCloudT& raw) { 
  fromROS_livox(msg, raw); }



sensor_msgs::msg::PointCloud2 toROS(const PointCloudT::Ptr& cloud, 
                                    const double& sweep_time) {
  
  sensor_msgs::msg::PointCloud2 out;
  pcl::toROSMsg(*cloud, out);
  out.header.stamp = rclcpp::Time(sweep_time);
  out.header.frame_id = Config::getInstance().topics.frame_id;

  return out;
}

nav_msgs::msg::Odometry toROS(State& state, const double& stamp) {

  Config& cfg = Config::getInstance();
  nav_msgs::msg::Odometry out;

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
  out.header.stamp    = rclcpp::Time(stamp);

  return out;
}

geometry_msgs::msg::TransformStamped toTF(const Eigen::Isometry3d& T,
                                          const std::string& parent,
                                          const std::string& child,
                                          const rclcpp::Time& stamp) {

  geometry_msgs::msg::TransformStamped msg;
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

void publishTFs(State& state, tf2_ros::TransformBroadcaster& br, const double& time) {

  Config& cfg = Config::getInstance();
  rclcpp::Time stamp = rclcpp::Time(time);

  Eigen::Isometry3d T_B_I = cfg.sensors.extrinsics.imu2baselink;
  Eigen::Isometry3d T_I_B = T_B_I.inverse();
  Eigen::Isometry3d T_M_B = state.isometry() * T_I_B;
  Eigen::Isometry3d T_B_L = T_B_I * state.L2I_isometry();

  br.sendTransform(toTF(T_M_B, cfg.topics.frame_id, "base_link",  stamp));
  br.sendTransform(toTF(T_B_I, "base_link",         "imu_link",   stamp));
  br.sendTransform(toTF(T_B_L, "base_link",         "lidar_link", stamp));
}


void fill_config(Config& cfg, rclcpp::Node* n) {

  // --------------------------------------------------
  // BASIC FLAGS
  // --------------------------------------------------
  n->get_parameter("verbose", cfg.verbose);
  n->get_parameter("debug",   cfg.debug);

  // --------------------------------------------------
  // TOPICS
  // --------------------------------------------------
  n->get_parameter("topics.input.lidar",               cfg.topics.input.lidar);
  n->get_parameter("topics.input.imu",                 cfg.topics.input.imu);
  n->get_parameter("topics.input.stop_ioctree_update", cfg.topics.input.stop_ioctree_update);
  n->get_parameter("topics.output.state",              cfg.topics.output.state);
  n->get_parameter("topics.output.frame",              cfg.topics.output.frame);
  n->get_parameter("topics.frame_id",                  cfg.topics.frame_id);

  // --------------------------------------------------
  // SENSORS
  // --------------------------------------------------
  n->get_parameter("sensors.lidar.type",         cfg.sensors.lidar.type);
  n->get_parameter("sensors.lidar.end_of_sweep", cfg.sensors.lidar.end_of_sweep);
  n->get_parameter("sensors.imu.hz",             cfg.sensors.imu.hz);

  n->get_parameter("sensors.calibration.gravity_align", cfg.sensors.calibration.gravity_align);
  n->get_parameter("sensors.calibration.accel",         cfg.sensors.calibration.accel);
  n->get_parameter("sensors.calibration.gyro",          cfg.sensors.calibration.gyro);
  n->get_parameter("sensors.calibration.time",          cfg.sensors.calibration.time);

  n->get_parameter("sensors.time_offset", cfg.sensors.time_offset);

  // --------------------------------------------------
  // EXTRINSICS imu2baselink
  // --------------------------------------------------
  {
    std::vector<double> tmp;
    n->get_parameter("sensors.extrinsics.imu2baselink.t", tmp);

    cfg.sensors.extrinsics.imu2baselink.setIdentity();
    cfg.sensors.extrinsics.imu2baselink.translation() =
        Eigen::Vector3d(tmp[0], tmp[1], tmp[2]);
  }

  {
    std::vector<double> tmp;
    n->get_parameter("sensors.extrinsics.imu2baselink.R", tmp);

    Eigen::Matrix3d R = (
        Eigen::AngleAxisd(tmp[0] * M_PI / 180.0, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(tmp[1] * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(tmp[2] * M_PI / 180.0, Eigen::Vector3d::UnitZ())
    ).toRotationMatrix();

    cfg.sensors.extrinsics.imu2baselink.linear() = R;
  }

  // --------------------------------------------------
  // EXTRINSICS lidar2baselink
  // --------------------------------------------------
  {
    std::vector<double> tmp;
    n->get_parameter("sensors.extrinsics.lidar2baselink.t", tmp);

    cfg.sensors.extrinsics.lidar2baselink.setIdentity();
    cfg.sensors.extrinsics.lidar2baselink.translation() =
        Eigen::Vector3d(tmp[0], tmp[1], tmp[2]);
  }

  {
    std::vector<double> tmp;
    n->get_parameter("sensors.extrinsics.lidar2baselink.R", tmp);

    Eigen::Matrix3d R = (
        Eigen::AngleAxisd(tmp[0] * M_PI / 180.0, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(tmp[1] * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(tmp[2] * M_PI / 180.0, Eigen::Vector3d::UnitZ())
    ).toRotationMatrix();

    cfg.sensors.extrinsics.lidar2baselink.linear() = R;
  }

  // --------------------------------------------------
  // INTRINSICS
  // --------------------------------------------------
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
    cfg.sensors.intrinsics.sm <<
      tmp[0], tmp[1], tmp[2],
      tmp[3], tmp[4], tmp[5],
      tmp[6], tmp[7], tmp[8];
  }

  // --------------------------------------------------
  // FILTERS
  // --------------------------------------------------
  {
    std::vector<double> tmp;
    n->get_parameter("filters.voxel_grid.leaf_size", tmp);
    cfg.filters.voxel_grid.leaf_size = Eigen::Vector4d(tmp[0], tmp[1], tmp[2], 1.);
  }

  n->get_parameter("filters.min_distance.active", cfg.filters.min_distance.active);
  n->get_parameter("filters.min_distance.value",  cfg.filters.min_distance.value);

  n->get_parameter("filters.crop_box.active", cfg.filters.crop_box.active);
  {
    std::vector<double> tmp;
    n->get_parameter("filters.crop_box.min", tmp);
    cfg.filters.crop_box.min = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]);
  }
  {
    std::vector<double> tmp;
    n->get_parameter("filters.crop_box.max", tmp);
    cfg.filters.crop_box.max = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]);
  }

  n->get_parameter("filters.fov.active", cfg.filters.fov.active);
  n->get_parameter("filters.fov.value",  cfg.filters.fov.value);
  cfg.filters.fov.value *= M_PI / 360.0;

  n->get_parameter("filters.rate_sampling.active", cfg.filters.rate_sampling.active);
  n->get_parameter("filters.rate_sampling.value",  cfg.filters.rate_sampling.value);

  // --------------------------------------------------
  // IKFoM
  // --------------------------------------------------
  n->get_parameter("IKFoM.max_iters",           cfg.ikfom.max_iters);
  n->get_parameter("IKFoM.tolerance",           cfg.ikfom.tolerance);
  n->get_parameter("IKFoM.lidar_noise",         cfg.ikfom.lidar_noise);
  n->get_parameter("IKFoM.estimate_extrinsics", cfg.ikfom.estimate_extrinsics);

  n->get_parameter("IKFoM.covariance.gyro",        cfg.ikfom.covariance.gyro);
  n->get_parameter("IKFoM.covariance.accel",       cfg.ikfom.covariance.accel);
  n->get_parameter("IKFoM.covariance.bias_gyro",   cfg.ikfom.covariance.bias_gyro);
  n->get_parameter("IKFoM.covariance.bias_accel",  cfg.ikfom.covariance.bias_accel);
  n->get_parameter("IKFoM.covariance.initial_cov", cfg.ikfom.covariance.initial_cov);

  n->get_parameter("IKFoM.plane.points",          cfg.ikfom.plane.points);
  n->get_parameter("IKFoM.plane.max_sqrt_dist",   cfg.ikfom.plane.max_sqrt_dist);
  n->get_parameter("IKFoM.plane.plane_threshold", cfg.ikfom.plane.plane_threshold);

  // --------------------------------------------------
  // IOCTREE
  // --------------------------------------------------
  n->get_parameter("iOctree.min_extent",  cfg.ioctree.min_extent);
  n->get_parameter("iOctree.bucket_size", cfg.ioctree.bucket_size);
  n->get_parameter("iOctree.downsample",  cfg.ioctree.downsample);
}
