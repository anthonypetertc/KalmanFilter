// kalman_filter.hpp
#ifndef INCLUDE_KALMAN_FILTER_KALMAN_STEP_HPP_  // Include guard
#define INCLUDE_KALMAN_FILTER_KALMAN_STEP_HPP_

#include <Eigen/Dense>  // dynamic size matrices and good performance
#include <iostream>

class KalmanStep {
  // TODO(Tony): Need to implement usage of parameters.
  // int dx;       // size of the vector representing the dependent
  // int dy;       // size of the vector representing the
  // independent variables.
  // int dparams;  // size of vector of parameters.

  Eigen::VectorXd x;  // dependent variables.
  Eigen::VectorXd y;  // independent variables (measured).
  Eigen::MatrixXd R;  // Measurement noise, should be of size (dy x dy)
  Eigen::MatrixXd Q;  // System noise, should be of size (dparams x dparams)

 public:
  // constructor
  KalmanStep(const Eigen::VectorXd& x, const Eigen::VectorXd& y,
             const Eigen::MatrixXd& R, const Eigen::MatrixXd& Q, int dx, int dy,
             int dparams);

  // get attributes
  Eigen::VectorXd getx() const;
  Eigen::VectorXd gety() const;
  Eigen::MatrixXd getR() const;
  Eigen::MatrixXd getQ() const;
  double getdx() const;
  double getdy() const;
  double getdparams() const;
};

#endif  // INCLUDE_KALMAN_FILTER_KALMAN_STEP_HPP_
