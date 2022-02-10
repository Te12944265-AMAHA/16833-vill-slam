#ifndef LIDAR_FACTOR_H
#define LIDAR_FACTOR_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include "../parameters.h"


struct LidarFactor
{
    static double sqrt_info;
    
    Eigen::Quaterniond relative_q;

    const Eigen::Vector3d p_dst;
    const Eigen::Vector3d p_src;
/*
    LidarFactor(double *_relative_q)
    {
        relative_q.x() = _relative_q[0];
        relative_q.y() = _relative_q[1];
        relative_q.z() = _relative_q[2];
        relative_q.w() = _relative_q[3];
    }
*/
    LidarFactor(const Eigen::Vector3d &dst, const Eigen::Vector3d &src) : p_dst(dst), p_src(src) {}

    template <typename T>
    bool operator()(const T *const pose1, const T *const pose2, T* residuals) const {
        
        // Make sure the Eigen::Vector world point is using the ceres::Jet type as it's Scalar type
        Eigen::Matrix<T,3,1> src; src << T(p_src[0]), T(p_src[1]), T(p_src[2]);
        Eigen::Matrix<T,3,1> dst; dst << T(p_dst[0]), T(p_dst[1]), T(p_dst[2]);

        // Map T* to Eigen Vector3 with correct Scalar type
        Eigen::Matrix<T,3,1> t = Eigen::Map<const Eigen::Matrix<T,3,1> >(pose1);
        Eigen::Matrix<T,3,1> tDst = Eigen::Map<const Eigen::Matrix<T,3,1> >(pose2);

        // Map the T* array to an Eigen Quaternion object (with appropriate Scalar type)
        Eigen::Quaternion<T> q = Eigen::Map<const Eigen::Quaternion<T> >(pose1+3);
        Eigen::Quaternion<T> qDst = Eigen::Map<const Eigen::Quaternion<T> >(pose2+3);

        // Rotate the point using Eigen rotations
        Eigen::Matrix<T,3,1> p = q * src;
        p += t;
        Eigen::Matrix<T,3,1> p2 = qDst * dst;
        p2 += tDst;

        // The error is the difference between the predicted and observed position.
        residuals[0] = (p - p2).dot(p - p2);
        residuals[0] *= T(sqrt_info);

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& dst, const Eigen::Vector3d& src) {
        return (new ceres::AutoDiffCostFunction<LidarFactor, 1, 7, 7>(new LidarFactor(dst, src)));
    }


/*
    static ceres::CostFunction *Create(double _relative_dist)
    {
        return (new ceres::AutoDiffCostFunction<LidarFactor, 1, 7, 7>(
            new LidarFactor(_relative_dist)));
    }

    template <typename T>
    bool operator()(const T *const pose1, const T *const pose2, T *residuals) const
    {
        Eigen::Quaterniond q01(pose1[3], pose1[4], pose1[5], pose1[6]);
        Eigen::Quaterniond q02(pose2[3], pose2[4], pose2[5], pose2[6]);
        Eigen::Quaterniond q12 = q01.inverse() * q02;
        Eigen::AngleAxisd d_aa = relative_q - q12;
        residuals[0] = T(sqrt_info) * T(d_aa.angle());
        // printf("displacement norm: %.6f, meas %.6f, residual %.3f\n",
        //        getDouble(d_position), relative_dist, getDouble(residuals[0]));

        return true;
    }
*/

};

#endif // LIDAR_FACTOR_H