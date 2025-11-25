#pragma once

#include <Eigen/Dense>

#include "Utils/Config.hpp"

struct Imu {
  double stamp;
  Eigen::Vector3d ang_vel;
  Eigen::Vector3d lin_accel;
  Eigen::Quaterniond q;

  Imu() : stamp(0.),
          ang_vel(Eigen::Vector3d::Zero()),
          lin_accel(Eigen::Vector3d::Zero()),
          q(Eigen::Quaterniond::Identity()) {}
};