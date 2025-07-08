// kalman_filter.hpp
#ifndef INCLUDE_KALMAN_FILTER_KALMAN_STEP_HPP_  // Include guard
#define INCLUDE_KALMAN_FILTER_KALMAN_STEP_HPP_

#include <Eigen/Dense>
#include <iostream>

class KalmanStep {
  const int dx;  // size of the vector representing the state.
  const int dy;  // size of the vector representing the measurement.
  const int du;  // size of the control vector.

  Eigen::VectorXd y;  // measurement outcome.
  Eigen::VectorXd u;  // control vector.
  Eigen::MatrixXd A;  // Linearized dynamics (dependence on previous state).
  Eigen::MatrixXd B;  // Linearized dynamics (dependence on control vector).
  Eigen::MatrixXd R;  // Measurement noise, should be of size (dy x dy)
  Eigen::MatrixXd Q;  // System noise, should be of size (dx x dx)
  Eigen::MatrixXd H;  // Relates the measurement to the state (dy x dx)

 public:
  // constructor
  KalmanStep(const Eigen::VectorXd& y, const Eigen::VectorXd& u,
             const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
             const Eigen::MatrixXd& R, const Eigen::MatrixXd& Q,
             const Eigen::MatrixXd& H);

  // get attributes
  Eigen::VectorXd gety() const;
  Eigen::VectorXd getu() const;
  Eigen::MatrixXd getA() const;
  Eigen::MatrixXd getB() const;
  Eigen::MatrixXd getR() const;
  Eigen::MatrixXd getQ() const;
  Eigen::MatrixXd getH() const;
  int getdx() const;
  int getdy() const;
  int getdu() const;
};

#endif  // INCLUDE_KALMAN_FILTER_KALMAN_STEP_HPP_
