#include <mutex>
#include <condition_variable>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>

#include "Core/Octree.hpp"
#include "Core/State.hpp"
#include "Core/Cloud.hpp"
#include "Core/Imu.hpp"

#include "Utils/Config.hpp"
#include "ROSutils.hpp"


class Manager : public rclcpp::Node {

  State state_;
  States state_buffer_;
  
  Imu prev_imu_;
  double first_imu_stamp_;

  bool imu_calibrated_;

  std::mutex mtx_state_;
  std::mutex mtx_buffer_;

  std::condition_variable cv_prop_stamp_;

  charlie::Octree ioctree_;
  bool stop_ioctree_update_;


  // ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr           stop_sub_;


  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_state;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_frame;

  // TF Broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Debug
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_raw;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_deskewed;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_downsampled;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_to_match;


public:
  Manager() : Node("limoncello", 
                   rclcpp::NodeOptions()
                      .allow_undeclared_parameters(true)
                      .automatically_declare_parameters_from_overrides(true)),
              first_imu_stamp_(-1.0), 
              state_buffer_(1000), 
              ioctree_(),
              stop_ioctree_update_(false) {

    Config& cfg = Config::getInstance();
    fill_config(cfg, this);

    state_.init();

    imu_calibrated_ = not (cfg.sensors.calibration.gravity
                           or cfg.sensors.calibration.accel
                           or cfg.sensors.calibration.gyro); 

    ioctree_.setBucketSize(cfg.ioctree.bucket_size);
    ioctree_.setDownsample(cfg.ioctree.downsample);
    ioctree_.setMinExtent(cfg.ioctree.min_extent);

    // Set callbacks and publishers
    rclcpp::SubscriptionOptions lidar_opt, imu_opt, stop_opt;
    lidar_opt.callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    imu_opt.callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    stop_opt.callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    cfg.topics.input.lidar, 
                    1, 
                    std::bind(&Manager::lidar_callback, this, std::placeholders::_1), 
                    lidar_opt);

    imu_sub_   = this->create_subscription<sensor_msgs::msg::Imu>(
                    cfg.topics.input.imu, 
                    3000, 
                    std::bind(&Manager::imu_callback, this, std::placeholders::_1), 
                    imu_opt);

    stop_sub_  = this->create_subscription<std_msgs::msg::Bool>(
                    cfg.topics.input.stop_ioctree_update,
                    10,
                    std::bind(&Manager::stop_update_callback, this, std::placeholders::_1),
                    stop_opt);

    pub_state       = this->create_publisher<nav_msgs::msg::Odometry>(cfg.topics.output.state, 10);
    pub_frame       = this->create_publisher<sensor_msgs::msg::PointCloud2>(cfg.topics.output.frame, 10);

    pub_raw         = this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/raw", 10);
    pub_deskewed    = this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/deskewed", 10);
    pub_downsampled = this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/downsampled", 10);
    pub_to_match    = this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/to_match", 10);

    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    auto param_names = this->list_parameters({}, 100).names;
    auto params = this->get_parameters(param_names);
    for (size_t i = 0; i < param_names.size(); i++) {
        RCLCPP_INFO(this->get_logger(), "Parameter: %s = %s",
                    param_names[i].c_str(),
                    params[i].value_to_string().c_str());
    }
  }
  

  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {

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

        if (cfg.sensors.calibration.gravity) {
          grav_vec = accel_avg.normalized() * abs(cfg.sensors.extrinsics.gravity);
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

      if (dt < 0) {
        RCLCPP_ERROR(get_logger(), "IMU timestamps not correct");
      }

      dt = (dt < 0 or dt >= imu.stamp) ? 1./cfg.sensors.imu.hz : dt;

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

      pub_state->publish(toROS(state_));
      
      if (cfg.frames.tf_pub) {
        tf_broadcaster_->sendTransform(toTF(state_));
      }
    }
  }


  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
PROFC_NODE("LiDAR Callback")

    Config& cfg = Config::getInstance();

    PointCloudT::Ptr raw(new PointCloudT);
    fromROS(*msg, *raw);

    if (raw->points.empty()) {
      RCLCPP_ERROR(get_logger(), "[LIMONCELLO] Raw PointCloud is empty!");
      return;
    }

    if (not imu_calibrated_)
      return;

    if (state_buffer_.empty()) {
      RCLCPP_ERROR(get_logger(), "[LIMONCELLO] No IMUs received");
      return;
    }

    PointTime point_time = point_time_func();
    double sweep_time = rclcpp::Time(msg->header.stamp).seconds() 
                        + cfg.sensors.TAI_offset;

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
      // every points needs to have a state associated not in the past
      RCLCPP_WARN(get_logger(), "Not enough interpolated states for deskewing pointcloud \n");
      return;
    }

  mtx_state_.lock();

    PointCloudT::Ptr deskewed    = deskew(raw, state_, interpolated, offset, sweep_time);
    PointCloudT::Ptr downsampled = voxel_grid(deskewed);
    PointCloudT::Ptr processed   = process(downsampled);

    if (processed->points.empty()) {
      RCLCPP_ERROR(get_logger(), "[LIMONCELLO] Processed & downsampled cloud is empty!");
      return;
    }

    state_.update(processed, ioctree_);
    Eigen::Affine3f T = (state_.affine3d() * state_.I2L_affine3d()).cast<float>();

  mtx_state_.unlock();

    PointCloudT::Ptr global(new PointCloudT);
    deskewed->width  = static_cast<uint32_t>(deskewed->points.size());
    deskewed->height = 1;                     
    pcl::transformPointCloud(*deskewed, *global, T);

    processed->height = 1;                     
    processed->width  = static_cast<uint32_t>(processed->points.size());
    pcl::transformPointCloud(*processed, *processed, T);

    // Publish
    pub_state->publish(toROS(state_));
    pub_frame->publish(toROS(global));

    if (cfg.debug) {
      pub_raw->publish(toROS(raw));
      pub_deskewed->publish(toROS(deskewed));
      pub_downsampled->publish(toROS(downsampled));
      pub_to_match->publish(toROS(processed));
    }

    // Update map
    ioctree_.update(processed->points);

    if (cfg.verbose)
      PROFC_PRINT()
  }

  void stop_update_callback(const std_msgs::msg::Bool::ConstSharedPtr msg) {
    if (not stop_ioctree_update_ and msg->data) {
      stop_ioctree_update_ = msg->data;
      RCLCPP_INFO(this->get_logger(), "Stopping ioctree updates from now onwards");
    }
  }
};


int main(int argc, char** argv) {

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
  
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr manager = std::make_shared<Manager>();

  rclcpp::executors::MultiThreadedExecutor executor; // by default using all available cores
  executor.add_node(manager);
  executor.spin();

  rclcpp::shutdown();


  return 0;
}

