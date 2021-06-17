//
// Created by dcheng on 4/24/20.
//

#ifndef VINS_ESTIMATOR_LASER_2D_FACTOR_H
#define VINS_ESTIMATOR_LASER_2D_FACTOR_H


#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../parameters.h"

class Laser2DFactor : public ceres::SizedCostFunction<1, 1>
{
public:
  Laser2DFactor(const double laser_dep, int _feature_idx);
  virtual inline bool Evaluate(double const *const *parameters, double *residuals,
      double **jacobians) const;

  double laser_dep_;
  Eigen::Matrix<double, 2, 3> tangent_base;
  static double sqrt_info;
  static double sum_t;
  int feature_idx_;
};


#endif //VINS_ESTIMATOR_LASER_2D_FACTOR_H
