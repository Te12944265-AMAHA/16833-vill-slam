#ifndef LIDAR_FACTOR_H
#define LIDAR_FACTOR_H

#include <ceres/ceres.h>
#include "../parameters.h"

struct LidarFactor
{
  static double sqrt_info;
  Eigen::Quaterniond relative_q;

  LidarFactor(double *_relative_q)
  {
    relative_q.x() = _relative_q[0];
    relative_q.y() = _relative_q[1];
    relative_q.z() = _relative_q[2];
    relative_q.w() = _relative_q[3];  
  }

  static ceres::CostFunction * Create(double _relative_dist)
  {
    return (new ceres::AutoDiffCostFunction<LidarFactor, 1, 7, 7>(
        new LidarFactor(_relative_dist)));
  }

  template <typename T>
  bool operator()(const T* const pose1, const T* const pose2, T *residuals) const
  {
    Eigen::Quaterniond q01(pose1[3], pose1[4], pose1[5], pose1[6]);
    Eigen::Quaterniond q02(pose2[3], pose2[4], pose2[5], pose2[6]);
    Eigen::Quaterniond q12 = q01.inverse() * q02;
    Eigen::AngleAxisd d_aa = relative_q - q12;
    residuals[0] = T(sqrt_info) * T(d_aa.angle());
    //printf("displacement norm: %.6f, meas %.6f, residual %.3f\n",
    //       getDouble(d_position), relative_dist, getDouble(residuals[0]));

    return true;
  }
};

#endif //LIDAR_FACTOR_H