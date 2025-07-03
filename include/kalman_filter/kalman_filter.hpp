// kalman_filter.hpp
#ifndef INCLUDE_KALMAN_FILTER_KALMAN_FILTER_HPP_  // Include guard
#define INCLUDE_KALMAN_FILTER_KALMAN_FILTER_HPP_

#include <Eigen/Dense>  // NOLINT
#include <iostream>
#include <kalman_filter/kalman_step.hpp>

class KalmanFilter {
  const int dx;  // size of the state vector.
  const int dy;  // size of the measurement vector.
  const int du;  // size of the control vector.

  Eigen::VectorXd
      estimate;        // current estimate of the parameters. Vector of size dx.
  Eigen::MatrixXd P;   // current uncertainty of estimate, represented by a
                       // covariance matrix of size (dx x dx).
  Eigen::MatrixXd Kg;  // current Kalman Gain, a matrix of size (dx x dy).

 public:
  // constructor: initialises the KalmanFilter object with an initial guess at
  // the parameters, and a covariance matrix representing the uncertainty around
  // that guess.
  KalmanFilter(const Eigen::VectorXd& init_estimate,
               const Eigen::MatrixXd& init_P, const int du, const int dy);

  // update the object with a new measurement.
  void update(const KalmanStep& next_step);

  // get attributes
  Eigen::VectorXd getEstimate() const;
  Eigen::MatrixXd getP() const;
  Eigen::MatrixXd getKg() const;
  int getdx() const;
  int getdy() const;
  int getdu() const;

 private:
  // private method to verify next_step is compatible with this
  // Kalman filter.
  void verify_step(const KalmanStep& next_step);

  // private method to predict the state.S
  void predict_state(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                     const Eigen::VectorXd& u);

  void predict_P(const Eigen::MatrixXd& A, const Eigen::MatrixXd& Q);

  // private method to update Kalman Gain.
  void update_kalman_gain(const Eigen::MatrixXd& H, const Eigen::MatrixXd& R);

  // private method to update estimate.
  void update_estimate(const Eigen::MatrixXd& H, const Eigen::VectorXd& y);

  // private method to update the uncertainty.
  void update_P(const Eigen::MatrixXd& H);
};

#endif  // INCLUDE_KALMAN_FILTER_KALMAN_FILTER_HPP_
