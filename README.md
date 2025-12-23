# LIMOncello

[arXiv](https://arxiv.org/abs/2512.19567)


Another tightly coupled LiDAR‑Inertial SLAM algorithm? Yep, based on the existing algorithms
[FAST-LIO2](https://github.com/hku-mars/FAST_LIO), [LIMO-Velo](https://github.com/Huguet57/LIMO-Velo),
[Fast LIMO](https://github.com/fetty31/fast_LIMO) and
[DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry), but cleaner, faster, and
without [IKFoM](https://github.com/hku-mars/IKFoM)/[ikd-Tree](https://github.com/hku-mars/ikd-Tree)
dependencies.

LIMOncello is essentially FAST-LIO 2 with the Fast LIMO (and thus DLIO) implementation, but
without any dependency on the [IKFoM](https://github.com/hku-mars/IKFoM) or
[ikd-Tree](https://github.com/hku-mars/ikd-Tree) C++ libraries. It implements an Iterated Error
State Extended Kalman Filter (ISEKF) as described in IKFoM and FAST-LIO 2, reimplemented using
the [manif](https://github.com/artivis/manif) library and a refactored version of the original
[iOctree](https://github.com/zhujun3753/i-octree) data structure.

This is the result of my efforts to truly understand LIMO‑Velo and, consequently, FAST-LIO 2.
Those insights led to an improvement in performance and accuracy: LIMOncello is the first known
LIO‑SLAM system to incorporate the SGal(3) manifold in its state representation, and it uses
iOctree, which is much faster and more efficient than iKd-Tree. On top of that, the code is
portable, not as Fast LIMO, but as simplified as possible (the entire IKFoM library is
synthesized into `State.hpp` thanks to [manif](https://github.com/artivis/manif), for example)
so it’s accessible to anyone who wants to modify it.

<div align="center">
  <img src="misc/CAT16x.gif" alt="CAT16X's performance"  width="800"/>
  <small>
  <p> Formula Student race car CAT16X. Velocity in straights (~15m/s) and really tight turns (~120deg/s) 
  (<a href="https://youtu.be/mk9U0lRWr-0?si=j4mM6e5dzihfCLJM">CAT15X</a>'s video for reference) </p>
  </small>
</div>
<br/>

## Dependencies

LIMOncello is header-only and the core depends only on:
- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [oneTBB](https://github.com/uxlfoundation/oneTBB)
- [manif](https://github.com/artivis/manif)

All Livox driver dependencies can be bypassed by compiling  
[RESPLE](https://github.com/ASIG-X/RESPLE) or by solely using its custom message  
definitions.

## Approach

To truly understand the concepts, here are the papers that greatly influenced this work, apart
from Fast LIMO:
- [IKFoM](https://arxiv.org/abs/2102.03804)
- [FAST-LIO](https://arxiv.org/abs/2010.08196) and [FAST-LIO2](https://arxiv.org/abs/2107.06829)
- [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/abs/1711.02508)
- [A micro Lie theory for state estimation in robotics](https://arxiv.org/abs/1812.01537)
- [Absolute humanoid localization and mapping based on IMU Lie group and fiducial markers](https://digital.csic.es/handle/10261/206456)
- [iOctree](https://arxiv.org/pdf/2309.08315)


I should also mention [S-FAST_LIO](https://github.com/zlwang7/S-FAST_LIO), which helped me grasp
the initial intuition behind the math of IKFoM, and [manif](https://github.com/artivis/manif),
which provides accessible, elegant Lie algebra tools for nearly any manifold used in robotics.


## How To

Most parameters are already commented in each configuration file. However,  
special attention should be paid to those related to `sensors.calibration`,  
`sensors.time_offset`, `sensors.extrinsics`, and `sensors.intrinsics`.

First, note that LIMOncello assumes the **NEU** convention for the world  
reference frame of any robot. The `sensors.extrinsics` are defined with respect  
to `base_link`, which is an arbitrarily chosen location on the robot. For  
example, in wheeled vehicles this is often the projection of the center of  
gravity (CoG) onto the x–y plane, such that the LiDAR map places the floor on  
the x–y plane. Most IMUs follow the NEU convention, but they are often rotated  
180 degrees about the x-axis, resulting in the z-axis pointing downward.

`sensors.intrinsics` can be manually provided and are typically obtained from a  
static IMU calibration procedure, during which Allan noise parameters are also  
estimated. These values are used to configure `IKFoM.covariance`. Although they  
can be manually set, if the IMU is not calibrated it is recommended to use  
reasonably large values (but not exceeding 1). If poor initialization is  
observed, it is most likely due to incorrect initial bias estimation. In such  
cases, you may also reduce the initial covariance value  
`IKFoM.covariance.initial_cov` to help stabilize the filter startup (this  
should be done cautiously).

This is where `sensors.calibration` becomes useful. Specifically,  
`sensors.calibration.gravity_align` attempts to refine the initial orientation  
defined in `sensors.extrinsics.imu2baselink`. The IMU reference frame is  
considered the robot reference frame, and the initial pose (position and  
orientation) is defined by `sensors.extrinsics.imu2baselink`; therefore,  
`base_link` is assumed to be at the origin at startup. The gravity alignment  
works by exploiting the fact that gravity always points downward and by  
detecting discrepancies between the expected gravity direction and the  
accelerometer measurements in a static condition.

Although the LiDAR and IMU are physically misaligned,  
`sensors.calibration.gravity_align` applies the same rotation and translation  
correction to both sensors, assuming they are rigidly mounted to the same  
body. The relative extrinsics between the IMU and the LiDAR are therefore kept  
fixed and are not re-estimated. As a consequence, the IMU (and thus the robot) may 
not appear level with respect to the ground.  

It is strongly discouraged to enable `IKFoM.estimate_extrinsics`. In most  
practical scenarios, online extrinsic estimation does not converge reliably  
and often leads to unstable or incorrect results. Providing accurate extrinsic  
parameters offline is the recommended approach.

`sensors.time_offset` does not need to be set when the LiDAR and IMU  
are not hardware-synchronized. As long as messages are timestamped within the  
same UTC time domain, LIMOncello will handle the alignment internally. This  
parameter is mainly useful when the sensors are not properly synchronized at  
the software level.

Finally, note that the estimated velocity is published with respect to  
`base_link` and not the IMU frame. This frame conversion can be modified in  
`include/ROSUtils.hpp` if a different convention is required.


## Configuration

Here, the configuration file for `LIMOncello` is explained.

| Parameter                        | Units    | Summary                                                                               |
|----------------------------------|----------|---------------------------------------------------------------------------------------|
| topics/input/lidar               | –        | ROS topic for incoming LiDAR point cloud                                              |
| topics/input/imu                 | –        | ROS topic for incoming IMU data                                                       |
| topics/input/stop_ioctree_update | –        | ROS topic to trigger stopping of the iOctree updates (std_msgs/Bool)                  |
| topics/output/state              | –        | ROS topic for published state estimates                                               |
| topics/output/frame              | –        | ROS topic for published full point cloud frame                                        |
| frame_id                         | –        | Coordinate frame ID used for all published data                                       |
| verbose                          | –        | Display execution time board                                                          |
| debug                            | –        | Publish intermediate point‑clouds (deskewed, processed, …) for visualization          |
| sensors/lidar/type               | –        | LiDAR model type (see config files)                                                   |
| sensors/lidar/end_of_sweep       | –        | Whether sweep timestamp refers to end (true) or start (false) of scan                 |
| sensors/imu/hz                   | Hz       | IMU data rate                                                                         |
| calibration/gravity_align        | –        | If true, estimate gravity vector while robot is stationary                            |
| calibration/accel                | –        | If true, estimate linear accelerometer bias while robot is stationary                 |
| calibration/gyro                 | –        | If true, estimate gyroscope bias while robot is stationary                            |
| calibration/time                 | s        | Duration for which robot must remain stationary to perform above calibrations         |
| time_offset                      | –        | Whether to account for sync offset between IMU and LiDAR                              |
| extrinsics/imu2baselink/t        | m        | Translation of IMU relative to base_link                                              |
| extrinsics/imu2baselink/R        | deg      | Rotation (roll, pitch, yaw) of IMU relative to base_link                              |
| extrinsics/lidar2baselink/t      | m        | Translation of LiDAR relative to base_link                                            |
| extrinsics/lidar2baselink/R      | deg      | Rotation (roll, pitch, yaw) of LiDAR relative to base_link                            |
| intrinsics/accel_bias            | m/s²     | Default accelerometer bias vector                                                     |
| intrinsics/gyro_bias             | rad/s    | Default gyroscope bias vector                                                         |
| intrinsics/sm                    | –        | Sensor‑to‑standard‑axis mapping matrix                                                |
| filters/voxel_grid/leaf_size     | m        | Voxel‑grid leaf size                                                                  |
| filters/min_distance/active      | –        | Enable minimum‑distance (sphere) crop                                                 |
| filters/min_distance/value       | m        | Radius for min‑distance crop                                                          |
| filters/crop_box/active          | -        | Enable crop box filter (centered at base_link)                                        |
| filters/crop_box/min             | m        | Minimum point of the box                                                              |
| filters/crop_box/max             | m        | Value of maximum point of the box                                                     |
| filters/fov/active               | –        | Enable field‑of‑view crop                                                             |
| filters/fov/value                | deg      | Field‑of‑view angle                                                                   |
| filters/rate_sampling/active     | –        | Enable simple rate‑based downsampling                                                 |
| filters/rate_sampling/value      | –        | Take one out of every *value* points                                                  |
| IKFoM/query_iters                | –        | Number ISEKF updates                                                                  |
| IKFoM/tolerance                  | –        | Convergence tolerance for ISEKF                                                       |
| IKFoM/lidar_noise                | –        | LiDAR measurement noise                                                               |
| IKFoM/covariance/gyro            | rad²     | Gyroscope measurement covariance                                                      |
| IKFoM/covariance/accel           | m²/s²    | Accelerometer measurement covariance                                                  |
| IKFoM/covariance/bias_gyro       | rad/s·√s | Gyroscope bias covariance                                                             |
| IKFoM/covariance/bias_accel      | m²/s²·√s | Accelerometer bias covariance                                                         |
| IKFoM/plane/points               | –        | Number of points used to fit each plane feature                                       |
| IKFoM/plane/max_sqrt_dist        | m        | Maximum distance from query to any point in the plane (if exceeded, plane is invalid) |
| IKFoM/plane/plane_threshold      | m        | Maximum distance from any point to its plane to be considered valid                   |
| iOctree/min_extent               | m        | Minimum cell size in octree                                                           |
| iOctree/bucket_size              | –        | Maximum points per octree leaf                                                        |
| iOctree/downsample               | –        | Whether to downsample when inserting into the octree                                  |
