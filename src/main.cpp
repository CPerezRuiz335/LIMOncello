#include <mutex>
#include <condition_variable>

#include <Eigen/Dense>

#include <ros/ros.h>

#include <tf2/convert.h>

#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include "Core/Octree.hpp"
#include "Core/State.hpp"
#include "Core/Cloud.hpp"
#include "Core/Imu.hpp"

#include "Utils/Config.hpp"
#include "ROSutils.hpp"


ros::Publisher pub_state, pub_frame, pub_raw, 
               pub_deskewed, pub_downsampled, pub_to_match;


class Manager {
  State state_;
  States state_buffer_;
  
  Imu prev_imu_;
  double first_imu_stamp_;

  bool imu_calibrated_;

  std::mutex mtx_state_;
  std::mutex mtx_buffer_;

  std::condition_variable cv_prop_stamp_;

  ros::NodeHandle nh_;

  thuni::Octree ioctree_;

  
public:
  Manager() : first_imu_stamp_(-1.0), state_buffer_(1000), ioctree_() {
    Config& cfg = Config::getInstance();

    imu_calibrated_ = not (cfg.sensors.calibration.gravity_align 
                           or cfg.sensors.calibration.accel
                           or cfg.sensors.calibration.gyro); 

    ioctree_.set_bucket_size(cfg.ioctree.bucket_size);
    ioctree_.set_down_size(cfg.ioctree.downsample);
    ioctree_.set_min_extent(cfg.ioctree.min_extent);
    ioctree_.set_order(cfg.ioctree.order);
  };
  
  ~Manager() = default;

  void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {

    Config& cfg = Config::getInstance();

    Imu imu = fromROS(msg);

    if (first_imu_stamp_ < 0.)
      first_imu_stamp_ = imu.stamp;
    
    if (not imu_calibrated_) {
      static int N(0);
      static Eigen::Vector3d gyro_avg(0., 0., 0.);
      static Eigen::Vector3d accel_avg(0., 0., 0.);
      static Eigen::Vector3d grav_vec(0., 0., cfg.sensors.extrinsics.gravity);

      if ((imu.stamp - first_imu_stamp_) < cfg.sensors.calibration.time) {
        gyro_avg  += imu.ang_vel;
        accel_avg += imu.lin_accel; 
        N++;

      } else {
        gyro_avg /= N;
        accel_avg /= N;

        if (cfg.sensors.calibration.gravity_align) {
          grav_vec = accel_avg.normalized() * abs(cfg.sensors.extrinsics.gravity);
          Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(
                                  grav_vec, 
                                  Eigen::Vector3d(0., 0., cfg.sensors.extrinsics.gravity));
          state_.quat(q);
          state_.g(-grav_vec);
        }
        
        if (cfg.sensors.calibration.gyro)
          state_.b_w(gyro_avg);

        if (cfg.sensors.calibration.accel)
          state_.b_a(accel_avg - grav_vec);

        imu_calibrated_ = true;
      }

    } else {
      double dt = imu.stamp - prev_imu_.stamp;
      dt = (dt < 0 or dt > 0.1) ? 1./cfg.sensors.imu.hz : dt;

      imu = imu2baselink(imu, dt);

      // Correct acceleration
      imu.lin_accel = cfg.sensors.intrinsics.sm * imu.lin_accel;
      prev_imu_ = imu;

      mtx_state_.lock();
        state_.predict(imu, dt);
      mtx_state_.unlock();

      mtx_buffer_.lock();
        state_buffer_.push_front(state_);
      mtx_buffer_.unlock();

      cv_prop_stamp_.notify_one();

      pub_state.publish(toROS(state_));
    }

  }

  void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

PROFC_NODE("LiDAR Callback")

    Config& cfg = Config::getInstance();

    PointCloudT::Ptr raw(boost::make_shared<PointCloudT>());
    fromROS(*msg, *raw);

    if (raw->points.empty()) {
      ROS_ERROR("[LIMONCELLO] Raw PointCloud is empty!");
      return;
    }

    if (not imu_calibrated_)
      return;

    if (state_buffer_.empty()) {
      ROS_ERROR("[LIMONCELLO] No IMUs received");
      return;
    }

    PointTime point_time = point_time_func();
    double sweep_time = msg->header.stamp.toSec() + cfg.sensors.TAI_offset;
    
    double offset = 0.0;
    if (cfg.sensors.time_offset) { // automatic sync (not precise!)
      offset = state_.stamp - point_time(raw->points.back(), sweep_time) - 1.e-4; 
      if (offset > 0.0) offset = 0.0; // don't jump into future
    }

    // Wait for state buffer
    double start_stamp = point_time(raw->points.front(), sweep_time) + offset;
    double end_stamp = point_time(raw->points.back(), sweep_time) + offset;

    if (state_buffer_.front().stamp < end_stamp) {
      std::cout << std::setprecision(20);
      std::cout <<
        "PROPAGATE WAITING... \n" <<
        "     - buffer time: " << state_buffer_.front().stamp << "\n"
        "     - end scan time: " << end_stamp << std::endl;

      std::unique_lock<decltype(mtx_buffer_)> lock(mtx_buffer_);
      cv_prop_stamp_.wait(lock, [this, &end_stamp] { 
          return state_buffer_.front().stamp >= end_stamp;
      });
    } 


  mtx_buffer_.lock();
    States interpolated = filter_states(state_buffer_,
                                        start_stamp,
                                        end_stamp);
  mtx_buffer_.unlock();

    if (start_stamp < interpolated.front().stamp or interpolated.size() == 0) {
      // every points needs to have a state associated not in the past
      ROS_WARN("Not enough interpolated states for deskewing pointcloud \n");
      return;
    }

  mtx_state_.lock();

    PointCloudT::Ptr deskewed = deskew(raw, state_, interpolated, offset, sweep_time);

    PointCloudT::Ptr downsampled(boost::make_shared<PointCloudT>());
    *downsampled = *deskewed;

    if (cfg.filters.voxel_grid.active)
      downsampled = voxel_grid(deskewed);
    
    PointCloudT::Ptr processed = process(downsampled);

    if (processed->points.empty()) {
      ROS_ERROR("[LIMONCELLO] Processed & downsampled cloud is empty!");
      return;
    }

    state_.update(processed, ioctree_);
    Eigen::Affine3f T = state_.affine3f() * state_.I2L_affine3f();

  mtx_state_.unlock();

    PointCloudT::Ptr global(boost::make_shared<PointCloudT>());
    pcl::transformPointCloud(*deskewed, *global, T);
    pcl::transformPointCloud(*processed, *processed, T);

    // Publish
    pub_state.publish(toROS(state_));
    pub_frame.publish(toROS(global));

    if (cfg.debug) {
      pub_raw.publish(toROS(raw));
      pub_deskewed.publish(toROS(deskewed));
      pub_downsampled.publish(toROS(downsampled));
      pub_to_match.publish(toROS(processed));
    }

    // Update map
    ioctree_.update(processed->points);

    if (cfg.verbose)
      PROFC_PRINT()
  }
};


int main(int argc, char** argv) {

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
  
  ros::init(argc, argv, "limoncello");
  ros::NodeHandle nh("~");
  

  // Setup config parameters.
  Config& cfg = Config::getInstance();
  fill_config(cfg, nh); 

  // Initialize manager (reads from config)
  Manager manager = Manager();

  // Publishers
  pub_state = nh.advertise<nav_msgs::Odometry>(cfg.topics.output.state, 10);
  pub_frame = nh.advertise<sensor_msgs::PointCloud2>(cfg.topics.output.frame, 10);

  // Debug only
  pub_raw         = nh.advertise<sensor_msgs::PointCloud2>("debug/raw",         10);
  pub_deskewed    = nh.advertise<sensor_msgs::PointCloud2>("debug/deskewed",    10);
  pub_downsampled = nh.advertise<sensor_msgs::PointCloud2>("debug/downsampled", 10);
  pub_to_match    = nh.advertise<sensor_msgs::PointCloud2>("debug/to_match",    10);


  // Subscribers
  ros::Subscriber lidar_sub = nh.subscribe(cfg.topics.input.lidar,
                                           1,
                                           &Manager::lidar_callback,
                                           &manager,
                                           ros::TransportHints().tcpNoDelay());

  ros::Subscriber imu_sub = nh.subscribe(cfg.topics.input.imu,
                                         1000,
                                         &Manager::imu_callback,
                                         &manager,
                                         ros::TransportHints().tcpNoDelay());

  ros::AsyncSpinner spinner(0);
  spinner.start();
  
  ros::waitForShutdown();

  return 0;
}
