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

#include <csvcpp.h>

ros::Publisher pub_state, pub_frame, pub_raw, 
               pub_deskewed, pub_downsampled, pub_to_match;

csv::CsvFile myCsv;

void save_data(const State& state, const int& lidar /* 0 or 1*/) {
  myCsv.push_back({state.stamp,
                   lidar,
                   state.p().x(), state.P(0,0),
                   state.p().y(), state.P(1,1),
                   state.p().z(), state.P(2,2),
                   state.angular().x(), state.P(3,3),
                   state.angular().y(), state.P(4,4),
                   state.angular().z(), state.P(5,5),
                   state.v().x(), state.P(12,12),
                   state.v().y(), state.P(13,13),
                   state.v().z(), state.P(14,14),
                   state.b_a().x(), state.P(18,18),
                   state.b_a().y(), state.P(19,19),
                   state.b_a().z(), state.P(20,20),
                   state.b_a().x(), state.P(15,15),
                   state.b_a().y(), state.P(16,16),
                   state.b_a().z(), state.P(17,17),
                   state.g().x(), state.P(21,21),
                   state.g().y(), state.P(22,22),
                   state.g().z(), state.P(23,23)});
}

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
  }

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
      save_data(state_, 0);
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

    PointCloudT::Ptr processed = process(deskewed);

    PointCloudT::Ptr to_match(boost::make_shared<PointCloudT>());
    PointCloudT::Ptr to_save(boost::make_shared<PointCloudT>());

    *to_match = *to_save = *processed;

    if (cfg.filters.voxel_grid.active) {
      to_save = voxel_grid(processed, cfg.filters.voxel_grid.leaf_size * .5);
      to_match = voxel_grid(processed, cfg.filters.voxel_grid.leaf_size * 1.5);
    }

    if (to_match->points.empty()) {
      ROS_ERROR("[LIMONCELLO] Processed & downsampled cloud is empty!");
      return;
    }

    state_.update(to_match, ioctree_);
    Eigen::Affine3f T = state_.affine3f() * state_.I2L_affine3f();

    save_data(state_, 1);
  mtx_state_.unlock();

    PointCloudT::Ptr global(boost::make_shared<PointCloudT>());
    pcl::transformPointCloud(*deskewed, *global, T);
    pcl::transformPointCloud(*to_save, *to_save, T);

    // Publish
    pub_state.publish(toROS(state_));
    pub_frame.publish(toROS(global));

    if (cfg.debug) {
      pub_raw.publish(toROS(raw));
      pub_deskewed.publish(toROS(deskewed));
      pub_downsampled.publish(toROS(processed));
      pub_to_match.publish(toROS(to_match));
    }

    // Update map
    ioctree_.update(to_save->points, false);

    if (cfg.verbose)
      PROFC_PRINT()
  }
};


int main(int argc, char** argv) {
  std::cout << std::setprecision(100);
  myCsv.setSeparator(',');
  myCsv.push_back({"stamp",
                   "lidar_update",
                   "x", "x_var",
                   "y", "y_var",
                   "z", "z_var",
                   "x_ang", "x_ang_var",
                   "y_ang", "y_ang_var",
                   "z_ang", "z_ang_var",
                   "vx", "vx_var",
                   "vy", "vy_var",
                   "vz", "vz_var",
                   "ba_x", "ba_x_var",
                   "ba_y", "ba_y_var",
                   "ba_z", "ba_z_var",
                   "bw_x", "bw_x_var",
                   "bw_y", "bw_y_var",
                   "bw_z", "bw_z_var",
                   "g_x", "g_x_var",
                   "g_y", "g_y_var",
                   "g_z", "g_z_var"});

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

  std::cout << cfg.debug_.csv_path << std::endl;
  myCsv.save(cfg.debug_.csv_path);

  return 0;
}

