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
    float TAI_offset;

  	struct {
  		Eigen::Affine3d imu2baselink_T;
  		Eigen::Affine3d lidar2baselink_T;
  		float gravity;

      struct {
        double roll, pitch, yaw;
        Eigen::Vector3d t;
      } lidar2baselink;

      struct {
        double roll, pitch, yaw;
        Eigen::Vector3d t;
      } imu2baselink;

  	} extrinsics;

  	struct {
  		Eigen::Vector3d accel_bias;
  		Eigen::Vector3d gyro_bias;
  		Eigen::Matrix3d sm;
  	} intrinsics;

  } sensors;

  struct Filters {
    struct {
    	bool active;
    	Eigen::Vector4d leaf_size;
    } voxel_grid;

    struct {
    	bool active;
    	float value;
    } min_distance;

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
  	int query_iters;
  	int max_iters;
  	float tolerance;
  	bool estimate_extrinsics;
  	float lidar_noise;

  	struct {
  		float gyro;
  		float accel;
  		float bias_gyro;
  		float bias_accel;
  	} covariance;

  	struct {
  		int points;
  		float max_sqrt_dist;
  		float plane_threshold;
  	} plane;
  } ikfom;

  struct iOctree {
    bool order;
    float min_extent;
    int bucket_size;
    bool downsample;
  } ioctree;

  struct Debug {
    std::string csv_path;
  } debug_;

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

