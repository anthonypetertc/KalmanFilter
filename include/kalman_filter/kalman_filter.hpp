// kalman_filter.hpp
#ifndef INCLUDE_KALMAN_FILTER_KALMAN_FILTER_HPP_  // Include guard
#define INCLUDE_KALMAN_FILTER_KALMAN_FILTER_HPP_

#include <Eigen/Dense>  // NOLINT
#include <iostream>
#include <kalman_filter/kalman_step.hpp>

class KalmanFilter {
  // TODO(Tony): Need to implement usage of dparams, dx, dy.
  // int dparams;  // size of the vector of parameters being estimated.
  // int dx;       // size of the vector representing the independent
  // int dy;       // size of the vector representing the
  // dependent (measured)
  //  variables.

  Eigen::VectorXd
      estimate;  // current estimate of the parameters. Vector of size dparams.
  Eigen::MatrixXd P;  // current uncertainty of estimate, represented by a
                      // covariance matrix of size (dparams x dparams).
  Eigen::MatrixXd K;  // current Kalman Gain, a matrix of size (dparams x dy).

 public:
  // constructor: initialises the KalmanFilter object with an initial guess at
  // the parameters, and a covariance matrix representing the uncertainty around
  // that guess.
  KalmanFilter(const Eigen::VectorXd& init_estimate,
               const Eigen::MatrixXd& init_P, double dx, double dy,
               double dparams);

  // predict
  Eigen::VectorXd predict();  // TODO(tony): what does this do?

  // update the object with a new measurement.
  void update(KalmanStep KS, const Eigen::MatrixXd& H,
              const Eigen::MatrixXd& L);

  // get attributes
  Eigen::VectorXd getEstimate() const;
  Eigen::VectorXd getP() const;
  Eigen::MatrixXd getK() const;
  double getdx() const;
  double getdy() const;
  double getdparams() const;

 private:
  // private method to obtain P_n|n-1.
  void initial_update_covariance(KalmanStep KS, const Eigen::MatrixXd& L);

  // private method to obtain alpha_n|n-1
  // TODO(Tony): Best way to incorporate the dynamics?
  void initial_update_estimate(KalmanStep KS);

  // private method to compute Kalman Gain.
  void compute_kalman_gain(KalmanStep KS, const Eigen::MatrixXd& H);

  // private method to to obtain P_n|n.
  void update_covariance(const Eigen::MatrixXd& H);

  // private method to compute estimate
  // TODO(Tony): Best way to incorporate the dynamics?
  void update_estimate(KalmanStep KS);
};

#endif  // INCLUDE_KALMAN_FILTER_KALMAN_FILTER_HPP_
