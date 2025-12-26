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
#include "Utils/PCL.hpp"

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

  // Subscribers
  rclcpp::SubscriptionBase::SharedPtr                    lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   stop_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_state_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_frame_;

  // TF Broadcaster
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // Debug
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_raw_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_deskewed_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_downsampled_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_;


public:
  Manager() : Node("limoncello", 
                   rclcpp::NodeOptions()
                      .allow_undeclared_parameters(true)
                      .automatically_declare_parameters_from_overrides(true)),
              first_imu_stamp_(-1.0), 
              state_buffer_(1000), 
              ioctree_(),
              stop_ioctree_update_(false),
              tf_broadcaster_(*this)  {

    Config& cfg = Config::getInstance();
    fill_config(cfg, this);

    state_.init();

    imu_calibrated_ = not (cfg.sensors.calibration.gravity_align
                           or cfg.sensors.calibration.accel
                           or cfg.sensors.calibration.gyro)
                      or cfg.sensors.calibration.time <= 0.; 

    ioctree_.setBucketSize(cfg.ioctree.bucket_size);
    ioctree_.setDownsample(cfg.ioctree.downsample);
    ioctree_.setMinExtent(cfg.ioctree.min_extent);

    // Set callbacks and publishers
    rclcpp::SubscriptionOptions lidar_opt, imu_opt, stop_opt;
    lidar_opt.callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    imu_opt.callback_group   = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    stop_opt.callback_group  = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        cfg.topics.input.imu, 3000, 
        std::bind(&Manager::imu_callback, this, std::placeholders::_1), imu_opt);

    stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        cfg.topics.input.stop_ioctree_update, 10,
        std::bind(&Manager::stop_update_callback, this, std::placeholders::_1), stop_opt);

    switch (cfg.sensors.lidar.type) {
      case 0:
      case 1:
      case 2:
      case 3:
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cfg.topics.input.lidar, 5,
            std::bind(&Manager::PointCloud2_callback, this, std::placeholders::_1), lidar_opt);
        break;

      case 4:
        lidar_sub_ = this->create_subscription<livox_interfaces::msg::CustomMsg>(
            cfg.topics.input.lidar, 5,
            std::bind(&Manager::livox_interfaces_callback, this, std::placeholders::_1), lidar_opt);
        break;

      case 5:
        lidar_sub_ = this->create_subscription<livox_ros_driver::msg::CustomMsg>(
            cfg.topics.input.lidar, 5,
            std::bind(&Manager::livox_ros_driver_callback, this, std::placeholders::_1), lidar_opt);
        break;

      case 6:
        lidar_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            cfg.topics.input.lidar, 5,
            std::bind(&Manager::livox_ros_driver2_callback, this, std::placeholders::_1), lidar_opt);
        break;

      default:
        RCLCPP_ERROR(this->get_logger(),
          "Unknown lidar type %d in config. Cannot create subscriber.", cfg.sensors.lidar.type);
        throw std::runtime_error("Invalid lidar type");
    }

    pub_state_       = this->create_publisher<nav_msgs::msg::Odometry>(cfg.topics.output.state, 10);
    pub_frame_       = this->create_publisher<sensor_msgs::msg::PointCloud2>(cfg.topics.output.frame, 10);

    pub_raw_         = this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/raw",         10);
    pub_deskewed_    = this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/deskewed",    10);
    pub_downsampled_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/downsampled", 10);
    pub_filtered_    = this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/filtered",    10);
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
        RCLCPP_ERROR(get_logger(), "IMU timestamps not correct");

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

      pub_state_->publish(toROS(state_, imu.stamp));
      publishTFs(state_, tf_broadcaster_, imu.stamp);
    }
  }

  void PointCloud2_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    process_cloud([&]() {
      PointCloudT::Ptr raw(new PointCloudT);
      pcl::fromROSMsg(*msg, *raw);
      return raw;
    }, rclcpp::Time(msg->header.stamp).seconds());
  }

  void livox_ros_driver_callback(const livox_ros_driver::msg::CustomMsg::ConstSharedPtr& msg) {
    process_cloud([&]() {
      PointCloudT::Ptr raw(new PointCloudT);
      fromROS(*msg, *raw);
      return raw;
    }, rclcpp::Time(msg->header.stamp).seconds());
  }

  void livox_ros_driver2_callback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg) {
    process_cloud([&]() {
      PointCloudT::Ptr raw(new PointCloudT);
      fromROS(*msg, *raw);
      return raw;
    }, rclcpp::Time(msg->header.stamp).seconds());
  }

  void livox_interfaces_callback(const livox_interfaces::msg::CustomMsg::ConstSharedPtr& msg) {
    process_cloud([&]() {
      PointCloudT::Ptr raw(new PointCloudT);
      fromROS(*msg, *raw);
      return raw;
    }, rclcpp::Time(msg->header.stamp).seconds());
  }

  template<typename F>
  void process_cloud(F&& producer, const double& sweep_time) {
    Config& cfg = Config::getInstance();

    if (not imu_calibrated_)
      return;
    
    if (state_buffer_.empty()) {
      RCLCPP_ERROR(get_logger(), "[LIMONCELLO] No IMUs received");
      return;
    }
    
    PointCloudT::Ptr raw = producer();

    if (raw->points.empty()) {
      RCLCPP_ERROR(get_logger(), "[LIMONCELLO] Raw PointCloud is empty!");
      return;
    }

    min_at_front_max_at_back(raw); // oldest point to front and newest to back
    PointTime point_time = point_time_func();
    
    double offset = 0.0;
    if (cfg.sensors.time_offset) { // automatic sync (not precise!)
      offset = state_.stamp - point_time(raw->points.back(), sweep_time) - 1.e-4; 
      if (offset > 0.0) offset = 0.0; // don't jump into future
    }

    // Wait for state buffer
    double start_stamp = point_time(raw->points.front(), sweep_time) + offset;
    double end_stamp   = point_time(raw->points.back(),  sweep_time) + offset;

    if (state_buffer_.front().stamp < end_stamp) {
      std::unique_lock<decltype(mtx_buffer_)> lock(mtx_buffer_);

      std::cout << std::setprecision(20);
      std::cout <<
        "PROPAGATE WAITING... \n" <<
        "     - buffer time: " << state_buffer_.front().stamp << "\n"
        "     - end scan time: " << end_stamp << std::endl;
      
      cv_prop_stamp_.wait(lock, [this, &end_stamp] { 
        return state_buffer_.front().stamp >= end_stamp;
      });
    } 

  mtx_buffer_.lock();
    States interpolated = filter_states(state_buffer_, start_stamp, end_stamp);
  mtx_buffer_.unlock();

    if (start_stamp < interpolated.front().stamp or interpolated.size() == 0) {
      // every point needs to have a state associated not in the past
      RCLCPP_WARN(get_logger(), "Not enough interpolated states for deskewing pointcloud \n");
      return;
    }

  mtx_state_.lock();

    PointCloudT::Ptr deskewed    = deskew(raw, state_, interpolated, offset, sweep_time);
    PointCloudT::Ptr downsampled = voxel_grid(deskewed);
    PointCloudT::Ptr filtered    = filter(downsampled, state_.isometry() * state_.L2I_isometry());

    if (filtered->points.empty()) {
      RCLCPP_ERROR(get_logger(), "Filtered cloud is empty!");
      mtx_state_.unlock();
      return;
    }

    state_.update(filtered, ioctree_);
    Eigen::Isometry3f T = (state_.isometry() * state_.L2I_isometry()).cast<float>();
  
  mtx_buffer_.lock();
    state_buffer_[0] = state_;
  mtx_buffer_.unlock();

  mtx_state_.unlock();

    PointCloudT::Ptr global(new PointCloudT);
    deskewed->height = 1;                     
    deskewed->width  = static_cast<uint32_t>(deskewed->points.size());
    pcl::transformPointCloud(*deskewed, *global, T);
    
    PointCloudT::Ptr to_save(new PointCloudT);
    filtered->height = 1;                     
    filtered->width  = static_cast<uint32_t>(filtered->points.size());
    pcl::transformPointCloud(*filtered, *to_save, T);

    // Publish
    pub_state_->publish(toROS(state_, sweep_time));
    pub_frame_->publish(toROS(global, sweep_time));

    if (cfg.debug) {
      pub_raw_->publish(toROS(raw, sweep_time));
      pub_deskewed_->publish(toROS(deskewed, sweep_time));
      pub_downsampled_->publish(toROS(downsampled, sweep_time));
      pub_filtered_->publish(toROS(to_save, sweep_time));
    }

    // Update map
    if (not stop_ioctree_update_)
      ioctree_.update(to_save->points);

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

