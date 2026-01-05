#include <mutex>
#include <condition_variable>

#include <Eigen/Dense>

#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>


#include "Core/Octree.hpp"
#include "Core/State.hpp"
#include "Core/Cloud.hpp"
#include "Core/Imu.hpp"

#include "Utils/Config.hpp"
#include "ROSutils.hpp"


class Manager {
  State  state_;
  States state_buffer_;
  
  Imu prev_imu_;
  double first_imu_stamp_;

  bool imu_calibrated_;

  std::mutex mtx_state_;
  std::mutex mtx_buffer_;

  std::condition_variable cv_prop_stamp_;

  charlie::Octree ioctree_;
  bool stop_ioctree_update_;

  ros::Publisher pub_state_, 
                 pub_frame_,
                 pub_raw_, 
                 pub_deskewed_, 
                 pub_downsampled_, 
                 pub_filtered_;

  tf2_ros::TransformBroadcaster br;
  
public:
  Manager(ros::NodeHandle& nh) : first_imu_stamp_(-1.0), 
                                 state_buffer_(1000), 
                                 stop_ioctree_update_(false),
                                 ioctree_() {

    Config& cfg = Config::getInstance();

    imu_calibrated_ = not (cfg.sensors.calibration.gravity_align
                           or cfg.sensors.calibration.accel
                           or cfg.sensors.calibration.gyro)
                      or cfg.sensors.calibration.time <= 0.; 

    ioctree_.setBucketSize(cfg.ioctree.bucket_size);
    ioctree_.setDownsample(cfg.ioctree.downsample);
    ioctree_.setMinExtent(cfg.ioctree.min_extent);

    // Publishers
    pub_state_ = nh.advertise<nav_msgs::Odometry>(cfg.topics.output.state, 10);
    pub_frame_ = nh.advertise<sensor_msgs::PointCloud2>(cfg.topics.output.frame, 10);

    // Debug only
    pub_raw_         = nh.advertise<sensor_msgs::PointCloud2>("debug/raw",         10);
    pub_deskewed_    = nh.advertise<sensor_msgs::PointCloud2>("debug/deskewed",    10);
    pub_downsampled_ = nh.advertise<sensor_msgs::PointCloud2>("debug/downsampled", 10);
    pub_filtered_    = nh.advertise<sensor_msgs::PointCloud2>("debug/filtered",    10);
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

      if ((imu.stamp - first_imu_stamp_) < cfg.sensors.calibration.time) {
        gyro_avg  += imu.ang_vel;
        accel_avg += imu.lin_accel; 
        N++;

      } else {
        gyro_avg /= N;
        accel_avg /= N;

        if (cfg.sensors.calibration.gravity_align) {
          Eigen::Vector3d g_m = (accel_avg - state_.b_a()).normalized(); 
                          g_m *= cfg.sensors.extrinsics.gravity;
          
          Eigen::Vector3d g_b = state_.quat().conjugate() * state_.g();
          Eigen::Quaterniond dq = Eigen::Quaterniond::FromTwoVectors(g_b, g_m);

          state_.quat((state_.quat() * dq).normalized());
        }
        
        if (cfg.sensors.calibration.gyro)
          state_.b_w(gyro_avg);

        if (cfg.sensors.calibration.accel)
          state_.b_a(accel_avg - state_.R().transpose()*state_.g());

        imu_calibrated_ = true;
      }

    } else {
      double dt = imu.stamp - prev_imu_.stamp;

      if (dt < 0)
        ROS_ERROR("IMU timestamps not correct");

      dt = (dt < 0 or dt >= imu.stamp) ? 1./cfg.sensors.imu.hz : dt;

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

      pub_state_.publish(toROS(state_, imu.stamp));
      publishTFs(state_, br, imu.stamp);
    }
  }


  void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    Config& cfg = Config::getInstance();

    if (not imu_calibrated_)
      return;
    
    if (state_buffer_.empty()) {
      ROS_ERROR("[LIMONCELLO] No IMUs received");
      return;
    }
    
    PointCloudT::Ptr raw(boost::make_shared<PointCloudT>());
    fromROS(*msg, *raw);

    if (raw->points.empty()) {
      ROS_ERROR("[LIMONCELLO] Raw PointCloud is empty!");
      return;
    }

    PointTime point_time = point_time_func();
    double sweep_time = msg->header.stamp.toSec();
    
    double offset = 0.0;
    if (cfg.sensors.time_offset) { // automatic sync (not precise!)
      offset = state_.stamp - point_time(raw->points.back(), sweep_time) - 1.e-4; 
      if (offset > 0.0) offset = 0.0; // don't jump into future
    }

    // Wait for state buffer
    double start_stamp = point_time(raw->points.front(), sweep_time) + offset;
    double end_stamp   = point_time(raw->points.back(), sweep_time) + offset;

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
    States interpolated = filter_states(state_buffer_, start_stamp, end_stamp);
  mtx_buffer_.unlock();

    if (start_stamp < interpolated.front().stamp or interpolated.size() == 0) {
      // every point needs to have a state associated not in the past
      ROS_WARN("Not enough interpolated states for deskewing pointcloud \n");
      return;
    }

  mtx_state_.lock();

    PointCloudT::Ptr deskewed    = deskew(raw, state_, interpolated, offset, sweep_time);
    PointCloudT::Ptr downsampled = voxel_grid(deskewed);
    PointCloudT::Ptr filtered    = filter(downsampled, state_.isometry() * state_.L2I_isometry());

    if (filtered->points.empty()) {
      ROS_ERROR("[LIMONCELLO] Filtered & downsampled cloud is empty!");
      mtx_state_.unlock();
      return;
    }

    state_.update(filtered, ioctree_);

    Eigen::Isometry3f T = (state_.isometry() * state_.L2I_isometry()).cast<float>();
  mtx_state_.unlock();

    PointCloudT::Ptr global(boost::make_shared<PointCloudT>());
    pcl::transformPointCloud(*deskewed, *global, T);

    PointCloudT::Ptr to_save(boost::make_shared<PointCloudT>());
    pcl::transformPointCloud(*filtered, *to_save, T);

    // Publish
    pub_state_.publish(toROS(state_, sweep_time));
    pub_frame_.publish(toROS(global, sweep_time));

    if (cfg.debug) {
      pub_raw_.publish(toROS(raw, sweep_time));
      pub_deskewed_.publish(toROS(deskewed, sweep_time));
      pub_downsampled_.publish(toROS(downsampled, sweep_time));
      pub_filtered_.publish(toROS(to_save, sweep_time));
    }

    // Update map
    if (not stop_ioctree_update_)
      ioctree_.update(to_save->points);

    if (cfg.verbose)
      PROFC_PRINT()
  }


  void stop_update_callback(const std_msgs::Bool::ConstPtr& msg) {
    if (not stop_ioctree_update_ and msg->data) {
      stop_ioctree_update_ = msg->data;
      ROS_INFO("Stopping ioctree updates from now onwards");
    }
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
  Manager manager = Manager(nh);

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

  ros::Subscriber stop_sub = nh.subscribe(cfg.topics.input.stop_ioctree_update,
                                          10,
                                          &Manager::stop_update_callback,
                                          &manager);


  ros::AsyncSpinner spinner(0);
  spinner.start();
  
  ros::waitForShutdown();

  return 0;
}

