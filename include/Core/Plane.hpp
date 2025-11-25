#pragma once

#include <vector>

#include <Eigen/Dense>

#include "Utils/PCL.hpp"


inline bool estimate_plane(Eigen::Vector4d& pabcd,
                           const std::vector<pcl::PointXYZ>& pts,
                           const double& thresh) {

  int N = pts.size();
  if (N < 3)
    return false;

  Eigen::Matrix<double, Eigen::Dynamic, 3> neighbors(N, 3);
  for (size_t i = 0; i < N; i++) {
    neighbors.row(i) = pts[i].getVector3fMap().cast<double>();
  }

  Eigen::Vector3d centroid = neighbors.colwise().mean(); 
  neighbors.rowwise() -= centroid.transpose();

  Eigen::Matrix3d cov = (neighbors.transpose() * neighbors) / N;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(cov);
  if (eigensolver.info() != Eigen::Success)
    return false;

  Eigen::Vector3d normal = eigensolver.eigenvectors().col(0);
  double d = -normal.dot(centroid);

  pabcd.head<3>() = normal;
  pabcd(3) = d;

  for (auto& p : pts) {
    double distance = normal.dot(p.getVector3fMap().cast<double>()) + d;
    if (std::abs(distance) > thresh)
      return false;
  }

  return true;
}


struct Plane {
  Eigen::Vector3d p;
  Eigen::Vector4d n; // world normal vector

  Plane() = default;
  Plane(Eigen::Vector3d& p_, Eigen::Vector4d& n_) : p(p_), n(n_) {};
};

inline double dist2plane(const Eigen::Vector4d& normal,
                         const Eigen::Vector3d& point) {
  return normal.head<3>().dot(point) + normal(3);
}


typedef std::vector<Plane> Planes;