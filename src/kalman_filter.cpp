#include "kalman_filter/kalman_filter.hpp"

#include <Eigen/Dense>
#include <cassert>

#include "kalman_filter/kalman_step.hpp"

KalmanFilter::KalmanFilter(const Eigen::VectorXd& init_estimate,
                           const Eigen::MatrixXd& init_P, const int du,
                           const int dy)
    : estimate(init_estimate),
      P(init_P),
      du(du),
      dy(dy),
      dx(init_estimate.size()),
      Kg(Eigen::MatrixXd::Zero(dx, dy)) {}

int KalmanFilter::getdx() const { return dx; }
int KalmanFilter::getdy() const { return dy; }
int KalmanFilter::getdu() const { return du; }

Eigen::VectorXd KalmanFilter::getEstimate() const { return estimate; }
Eigen::MatrixXd KalmanFilter::getP() const { return P; }
Eigen::MatrixXd KalmanFilter::getKg() const { return Kg; }
