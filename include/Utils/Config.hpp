#pragma once

#include <ros/ros.h>

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <iomanip> 
#include <type_traits>


struct Config {

	bool verbose;
	bool debug;

  struct Topics {
  	struct {
  		std::string lidar;
  		std::string imu;
		std::string stop_ioctree_update;
  	} input;

  	struct {
  		std::string state;
  		std::string frame;
  	} output;
  	
  	std::string frame_id;
  } topics;


  struct Sensors {
  	struct { 
  		int type; 
  		bool end_of_sweep;
  	} lidar;

  	struct {
  		int hz;
  	} imu;

  	struct {
  		bool gravity_align;
  		bool accel;
  		bool gyro;
  		float time;
  	} calibration;

  	bool time_offset;

  	struct {
  		Eigen::Isometry3d imu2baselink;
  		Eigen::Isometry3d lidar2baselink;
  		float gravity = 9.80665;
  	} extrinsics;

  	struct {
  		Eigen::Vector3d accel_bias;
  		Eigen::Vector3d gyro_bias;
  		Eigen::Matrix3d sm;
  	} intrinsics;

  } sensors;

  struct Filters {
    struct {
      Eigen::Vector4d leaf_size;
    } voxel_grid;

    struct {
      bool active;
      float value;
    } min_distance;

    struct {
      bool active;
      Eigen::Vector3d min;
      Eigen::Vector3d max;
    } crop_box;

    struct {
      bool active;
      float value;
    } fov;

    struct {
        bool active;
        int value;
    } rate_sampling;

  } filters;

  struct IKFoM {
  	int max_iters;
  	float tolerance;
  	float lidar_noise;
	bool estimate_extrinsics;

  	struct {
  		float gyro;
  		float accel;
  		float bias_gyro;
  		float bias_accel;
		float initial_cov;
  	} covariance;

  	struct {
  		int points;
  		float max_sqrt_dist;
  		float plane_threshold;
  	} plane;
  } ikfom;

  struct iOctree {
	float min_extent;
	int bucket_size;
	bool downsample;
  } ioctree;

  static Config& getInstance() {
	static Config* config = new Config();
	return *config;
  }

 private:
  // Singleton pattern
  Config() = default;

  // Delete copy/move so extra instances can't be created/moved.
  Config(const Config&) = delete;
  Config& operator=(const Config&) = delete;
  Config(Config&&) = delete;
  Config& operator=(Config&&) = delete;
};

