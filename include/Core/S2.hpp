#pragma once

#include <cmath>
#include <optional>
#include <cassert>
#include <functional>
#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <manif/manif.h>
#include <manif/SO3.h>

#include "Utils/Config.hpp"

// Based on He-2021, [https://arxiv.org/abs/2102.03804] Eq. (64-70)

namespace S2 {

  using Vector2d = Eigen::Vector2d;
  using Vector3d = Eigen::Vector3d;
  using Matrix3d = Eigen::Matrix3d; 
  using Matrix2d = Eigen::Matrix2d; 
  using Matrix2x3d = Eigen::Matrix<double, 2, 3>;
  using Matrix3x2d = Eigen::Matrix<double, 3, 2>; 

  template <typename MatT>
  using Opt = std::optional<Eigen::Ref<MatT>>;

  inline Matrix3d R(const Vector3d& x) {
    return manif::SO3d::Tangent(x).exp().rotation();
  }

  Vector3d compose(const Vector3d& x, 
                   const Vector3d& y,
                   Opt<Matrix3d> J_x = std::nullopt,
                   Opt<Matrix3d> J_y = std::nullopt) {
    
    Matrix3d Ry = R(y);
    Vector3d out = Ry * x;

    if (J_x) *J_x = Ry;
    if (J_y) *J_y = -Ry * manif::skew(x) * manif::SO3d::Tangent(y).rjac() ;

    return out;
  }


  inline Matrix3x2d B(const Vector3d& x) {
    // Seems that IKFoM's S2 formalization is for negative vectors
    // as gravity is alwasy positive in State.hpp, B(x) is flipped
    Vector3d   e_i  = Vector3d::UnitZ() * -1.;
    Matrix3x2d E_jk = Matrix3d::Identity().leftCols<2>() * -1.;

    Matrix3x2d out;

    // Handle near-zero x: rotation ~ Identity
    if (x.norm() < 1e-12) {
        out = E_jk;
    } else {
        Vector3d cross      = e_i.cross(x);
        double   cross_norm = cross.norm();
        double   dot        = e_i.dot(x);                    

        if (cross_norm < 1e-12) {
            // x parallel to e_i: angle is 0 (c>=0) or pi (c<0), axis arbitrary ⟂ e_i
            out = dot >= 0. ? E_jk : R(Vector3d::UnitX() * M_PI) * E_jk;
        } else {
            double   theta = std::atan2(cross_norm, dot);
            Vector3d axis  = cross / cross_norm;                  // unit axis
            out = R(axis*theta) * E_jk;
        }
    }

    return out/x.norm();
  }


  inline Vector3d oplus(const Vector3d& x, 
                        const Vector2d& u,
                        Opt<Matrix3d>   J_x = std::nullopt,
                        Opt<Matrix3x2d> J_u = std::nullopt) {

    bool tinyu = u.norm() < 1e-12;
    
    Matrix3d RBu = R(B(x)*u);
    Vector3d out = tinyu ? x : (RBu*x).eval();

    if (J_x) { assert(tinyu); *J_x = Matrix3d::Identity(); }
    if (J_u) {
      if (tinyu)
        *J_u = -manif::skew(x)*B(x);
      else
        *J_u = -RBu * manif::skew(x) * manif::SO3d::Tangent(B(x)*u).rjac() * B(x);
    }

    return out; 
  }


  inline Vector2d ominus(const Vector3d& y, 
                         const Vector3d& x,
                         Opt<Matrix2x3d> J_y = std::nullopt) {

    Vector3d cross      = x.cross(y);
    double   cross_norm = cross.norm();
    double   dot        = x.dot(y);

    Vector2d out;
    if (cross_norm >= 1e-12) {
      Vector3d axis  = cross / cross_norm; 
      double   theta = std::atan2(cross_norm, dot);
      out = B(x).transpose() * (axis*theta);

      if (J_y) {
        double   cross_normSq = cross.squaredNorm();
        double   y_normSq     = y.squaredNorm();
        Matrix3d cross_outer  = cross*cross.transpose();

        *J_y = B(x).transpose() 
               * (1/cross_norm * (Matrix3d::Identity() - cross_outer/cross_normSq) * manif::skew(x) * theta 
               + axis * (dot*y.transpose() - y_normSq*x.transpose())/(cross_norm*y_normSq));
      }
    } else {
      // Collinear case: s ~ 0
      // y in same direction as x -> zero rotation in the tangent or either
      // Antipodal: angle = pi, axis is not unique (any unit vector ⟂ x).
      // Pick a consistent tangent direction; e.g. first basis vector of B(x).
      // This makes the result finite (norm = pi) but not unique by geometry.
      out = dot >= 0. ? Vector2d::Zero() : (M_PI*Vector2d::UnitX()).eval();

      if (J_y) { 
        if (dot < 0.)
          throw std::domain_error("S2::ominus derivative not defined (non-unique)");

        *J_y = B(x).transpose() * manif::skew(x); // no need to multiply by 1 / ||x||^2
                                                  // B(x) already normalizes and Fx if 
                                                  // left multiplied by B(x) again
      }
    }
    return out;
  }
}