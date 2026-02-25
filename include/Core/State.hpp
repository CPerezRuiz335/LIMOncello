#pragma once

#include <execution>
#include <numeric>
#include <algorithm>
#include <boost/circular_buffer.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "Core/Imu.hpp"
#include "Core/Plane.hpp"
#include "Core/Octree.hpp"
#include <Core/S2.hpp>

#include "Utils/Config.hpp"
#include "Utils/PCL.hpp"

#include <manif/manif.h>
#include <manif/SO3.h>
#include <manif/SE3.h>
#include <manif/Bundle.h>
#include <manif/Rn.h>


struct State {

  using BundleT = manif::Bundle<double,
      manif::SO3,    // rotation
      manif::R3,     // position
      manif::SE3,    // extrinsics
      manif::R3,     // velocity
      manif::R3,     // angular bias
      manif::R3,     // acceleartion bias
      manif::R3      // gravity
  >;

  using Tangent = typename BundleT::Tangent; 
  
  template<int R = Eigen::Dynamic, int C = R>
  using Mat = Eigen::Matrix<double, R, C>;

  template<int N = Eigen::Dynamic>
  using Vec = Eigen::Matrix<double, N, 1>;


  static constexpr int DoF = BundleT::DoF;  // DoF whole state
  static constexpr int DoFS2 = DoF-1;       // DoF g as S2
  static constexpr int DoFNoise = 4*3;      // b_w, b_a, n_{b_w}, n_{b_a}
  static constexpr int DoFObs = manif::SO3d::DoF 
                                + manif::R3d::DoF 
                                + manif::SE3d::DoF;   // DoF obsevation equation

  BundleT X;
  Mat<DoFS2> P;
  Mat<DoFNoise> Q;

  Vec<3> w; // angular velocity (IMU input)
  Vec<3> a; // linear acceleration (IMU input)

  double stamp;

  State() : stamp(-1.0) {};

  void init() {
  
    Config& cfg = Config::getInstance();
    
    // Set initial state
    auto extrinsics = cfg.sensors.extrinsics;
    auto lidar2imu  = extrinsics.imu2baselink.inverse() * extrinsics.lidar2baselink;
                                                                  //                Tangent (idx)
    X = BundleT(manif::SO3d(Eigen::AngleAxisd(extrinsics.imu2baselink.linear())), //           0
                manif::R3d(extrinsics.imu2baselink.translation()),//                           3
                manif::SE3d(lidar2imu),                           // isometry                  6
                manif::R3d(Vec<3>::Zero()),                       // velocity                 12
                manif::R3d(cfg.sensors.intrinsics.gyro_bias),     // b_w                      15
                manif::R3d(cfg.sensors.intrinsics.accel_bias),    // b_a                      18
                manif::R3d(Vec<3>::UnitZ()                        // g                        21
                           * extrinsics.gravity));

    P.setIdentity();
    P *= cfg.ikfom.covariance.initial_cov;

    w.setZero();
    a.setZero();

    // Control signal noise covariance (never changes)
    Q.setZero();
 
    Q.block<3, 3>(0, 0) = cfg.ikfom.covariance.gyro       * Eigen::Matrix3d::Identity(); // n_w
    Q.block<3, 3>(3, 3) = cfg.ikfom.covariance.accel      * Eigen::Matrix3d::Identity(); // n_a
    Q.block<3, 3>(6, 6) = cfg.ikfom.covariance.bias_gyro  * Eigen::Matrix3d::Identity(); // n_{b_w}
    Q.block<3, 3>(9, 9) = cfg.ikfom.covariance.bias_accel * Eigen::Matrix3d::Identity(); // n_{b_a}
  } 

  void predict(const Imu& imu, const double& dt) {
PROFC_NODE("predict")

    Mat<DoF> Adj, Jr; // Adjoint_X(u)^{-1}, J_r(u)  Sola-18, [https://arxiv.org/abs/1812.01537]
    BundleT X_tmp = X.plus(f(imu.lin_accel, imu.ang_vel, dt) * dt, Adj, Jr);
    
    // S2 particular cases. No increment for g
      Mat<3> AdjS2, JrS2;
      S2::boxplus(g(), {0., 0., 0.}, AdjS2, JrS2);

      Adj.template bottomRightCorner<3, 3>() = AdjS2;
      Jr.template bottomRightCorner<3, 3>() = JrS2;

      // Leftmost Jacobian
      Mat<2, 3> Jx;
      S2::ominus(g(), g(), Jx);

      Mat<DoFS2, DoF> left = Mat<DoFS2, DoF>::Identity();
      left.template bottomRightCorner<2, 3>() = Jx;
      
      // Rightmost Jacobian
      Mat<3, 2> Ju;
      S2::oplus(g(), {0., 0.}, {}, Ju);

      Mat<DoF, DoFS2> right = Mat<DoF, DoFS2>::Identity();
      right.template bottomRightCorner<3, 2>() = Ju;

    Mat<DoFS2>           Fx = left * (Adj + Jr * df_dx(imu, dt) * dt) * right; // Pérez-Ruiz-2026 [https://arxiv.org/abs/2512.19567] Eq. (8a)
    Mat<DoFS2, DoFNoise> Fw = left * Jr * df_dw(dt) * dt;                      // Pérez-Ruiz-2026 [https://arxiv.org/abs/2512.19567] Eq. (8b)

    P = Fx * P * Fx.transpose() + Fw * Q * Fw.transpose(); 

    X = X_tmp;

    // Save info
    a = imu.lin_accel;
    w = imu.ang_vel;

    stamp = imu.stamp;
  }


  void interpolate_to(const double& t) {
    double dt = t - this->stamp;
    assert(dt >= 0);

    X = X.plus(f(a, w, dt) * dt);
  }


  Tangent f(const Vec<3>& lin_acc, const Vec<3>& ang_vel, const double& dt) {

    Tangent u = Tangent::Zero();
    u.element<0>().coeffs() = ang_vel - b_w() /* -n_w */;
    u.element<1>().coeffs() = v() + 0.5 * (R()*(lin_acc - b_a() /* -n_a */) - g()) * dt;
    u.element<3>().coeffs() = R()*(lin_acc - b_a() /* -n_a */) - g();
    // u.element<4>().coeffs() = n_{b_w} 
    // u.element<5>().coeffs() = n_{b_a}
    
    return u;
  }

  Mat<DoF> df_dx(const Imu& imu, const double& dt) {
    Mat<DoF> out = Mat<DoF>::Zero();

    // rotation  (eq., idx)
    out.block<3, 3>(0, 15) = -Mat<3>::Identity(); // w.r.t b_w
    
    // position
    out.block<3, 3>(3, 12) = Mat<3>::Identity(); // w.r.t v
    out.block<3, 3>(3,  0) = 0.5*dt * -R()*manif::skew(imu.lin_accel - b_a()); // w.r.t R
    out.block<3, 3>(3, 18) = -0.5*dt * R(); // w.r.t b_a
    out.block<3, 3>(3, 21) = 0.5*dt * -Mat<3>::Identity(); // w.r.t g
    
    // velocity
    out.block<3, 3>(12, 0)  = -R()*manif::skew(imu.lin_accel - b_a()); // w.r.t R
    out.block<3, 3>(12, 18) = -R(); // w.r.t b_a
    out.block<3, 3>(12, 21) = -Mat<3>::Identity(); // w.r.t g

    return out;
  }

  Mat<DoF, DoFNoise> df_dw(const double& dt) {
    // w = (n_w, n_a, n_{b_w}, n_{b_a})
    Mat<DoF, DoFNoise> out = Mat<DoF, DoFNoise>::Zero();

    // rotation update
    out.block<3, 3>(0, 0) = -Mat<3>::Identity(); // w.r.t n_w
    // position update
    out.block<3, 3>(3, 3) = -0.5*dt * R(); // w.r.t n_a
    // velocity update
    out.block<3, 3>(12, 3) = -R(); // w.r.t n_a
    // b_w & b_a update
    out.block<3, 3>(15, 6) = Mat<3>::Identity(); // w.r.t n_{b_w}
    out.block<3, 3>(18, 9) = Mat<3>::Identity(); // w.r.t n_{b_a}
    
    return out;
  }

  void update(PointCloudT::Ptr& cloud, charlie::Octree& map) {
PROFC_NODE("update")

    Config& cfg = Config::getInstance();

// OBSERVATION MODEL

    auto h_model = [&](const State& s,
                       Mat<Eigen::Dynamic, DoFObs>& H,
                       Mat<Eigen::Dynamic, 1>&      z) {

      int N = cloud->size();

      std::vector<bool> chosen(N, false);
      Planes planes(N);

      std::vector<int> indices(N);
      std::iota(indices.begin(), indices.end(), 0);
      
      std::for_each(
        std::execution::par_unseq,
        indices.begin(),
        indices.end(),
        [&](int i) {
          PointT pt = cloud->points[i];
          Vec<3> p = pt.getVector3fMap().cast<double>();
          Vec<3> g = s.isometry() * s.L2I_isometry() * p; // global coords 

          std::vector<pcl::PointXYZ> neighbors;
          std::vector<float> pointSearchSqDis;
          map.knn(pcl::PointXYZ(g(0), g(1), g(2)),
                  cfg.ikfom.plane.points,
                  neighbors,
                  pointSearchSqDis);
          
          if (neighbors.size() < cfg.ikfom.plane.points 
              or pointSearchSqDis.back() > cfg.ikfom.plane.max_sqrt_dist)
                return;
          
          Eigen::Vector4d p_abcd = Eigen::Vector4d::Zero();
          if (not estimate_plane(p_abcd, neighbors, cfg.ikfom.plane.plane_threshold))
            return;


          chosen[i] = true;
          planes[i] = Plane(p, p_abcd);
        }
      ); // end for_each

      Planes valid_planes;

      for (int i = 0; i < N; i++) {
        if (chosen[i])
          valid_planes.push_back(planes[i]);        
      }

      H = Mat<>::Zero(valid_planes.size(), DoFObs);
      z = Mat<>::Zero(valid_planes.size(), 1);

      indices.resize(valid_planes.size());
      std::iota(indices.begin(), indices.end(), 0);

      // For each plane, calculate its derivative and distance
      std::for_each(
        std::execution::par_unseq,
        indices.begin(),
        indices.end(),
        [&](int i) {
          Plane m = valid_planes[i];
          
          Vec<3> p_lidar = m.p;
          Vec<3> p_imu = s.L2I_isometry() * p_lidar;

          Vec<3> C = s.R().transpose() * m.n.head(3);
          Vec<3> B = p_lidar.cross(s.L2I_isometry().linear().transpose() * C);
          Vec<3> A = p_imu.cross(C);

          H.block<1, 6>(i, 0) << A.transpose(), m.n.head(3).transpose();

          // Differentiate w.r.t. SE3
          if (cfg.ikfom.estimate_extrinsics)
            H.block<1, manif::SE3d::DoF>(i, 6) << B.transpose(), C.transpose();

          z(i) = -dist2plane(m.n, s.isometry() * p_imu);
        }
      );

    }; // end h_model

// IESEKF UPDATE

    BundleT    X_predicted = X;
    Mat<DoFS2> P_predicted = P;

    Mat<Eigen::Dynamic, DoFObs> H;
    Mat<Eigen::Dynamic, 1>      z;
    Mat<DoFS2> KH;

    double R = cfg.ikfom.lidar_noise;

    Vec<3> g_pred = g();

    int i(0);

    do {
      h_model(*this, H, z); // Update H,z and set K to zeros

      // project P to homemorphic space
        Mat<DoF> J_;
        Vec<DoFS2> dx = X.minus(X_predicted, J_).coeffs().head(DoFS2);
        dx.tail(2) = S2::ominus(g(), g_pred);

        // d/db ((g oplus b) ominus g_pred) | b = 0
        Mat<DoFS2> J_inv = J_.topLeftCorner(DoFS2, DoFS2).inverse();
        P = J_inv * P * J_inv.transpose(); // !! projection

      // Build K from blocks (numerical stability)
        Mat<DoFObs> HTH = H.transpose() * H / R;
        
        Mat<DoFS2>  P_inv = P.inverse();
        P_inv.template topLeftCorner<DoFObs, DoFObs>() += HTH;
        P_inv = P_inv.inverse();

        Vec<DoFS2> Kz = P_inv.template topLeftCorner<DoFS2, DoFObs>() * H.transpose() * z / R;

        KH.setZero();
        KH.template topLeftCorner<DoFS2, DoFObs>() = P_inv.template topLeftCorner<DoFS2, DoFObs>() * HTH;

      dx = Kz + (KH - Mat<DoFS2>::Identity()) * J_inv * dx; 
      
      // Update manif Bundle, left g unmodified
      Tangent tau = Tangent::Zero();
      tau.coeffs().head(DoF-3) = dx.head(DoF-3);

      // Update
      X = X.plus(tau);
      g(S2::oplus(g(), dx.tail(2)));

      if ((dx.array().abs() <= cfg.ikfom.tolerance).all())
        break;

    } while (i++ < cfg.ikfom.max_iters);

    P = (Mat<DoFS2>::Identity() - KH) * P;
    X = X;
  }


// Getters
  inline Vec<3>                p() const { return X.element<1>().coeffs();             }
  inline Mat<3>                R() const { return X.element<0>().quat().toRotationMatrix(); }
  inline Eigen::Quaterniond quat() const { return X.element<0>().quat();                    }
  inline Vec<3>                v() const { return X.element<3>().coeffs();          }
  inline Vec<3>              b_w() const { return X.element<4>().coeffs();                  }
  inline Vec<3>              b_a() const { return X.element<5>().coeffs();                  }
  inline Vec<3>                g() const { return X.element<6>().coeffs();                  }

  inline Eigen::Isometry3d isometry() const {
    Eigen::Isometry3d T;
    T.linear() = R();
    T.translation() = p();
    return T;
  }

  inline Eigen::Isometry3d L2I_isometry() const {
    return X.element<2>().isometry();
  }

// Setters
  void quat(const Eigen::Quaterniond& in) { X.element<0>() = manif::SO3d(in);                  } 
  void b_w (const Vec<3>& in)             { X.element<4>() = manif::R3d(in);                   }
  void b_a (const Vec<3>& in)             { X.element<5>() = manif::R3d(in);                   }
  void g   (const Vec<3>& in)             { X.element<6>() = manif::R3d(in);                   }

};

typedef boost::circular_buffer<State> States;
