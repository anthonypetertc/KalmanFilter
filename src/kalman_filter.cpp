#include "kalman_filter/kalman_filter.hpp"

#include <Eigen/Dense>
#include <cassert>
#include <stdexcept>

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

void KalmanFilter::update(const KalmanStep& next_step) {
  verify_step(next_step);
}

void KalmanFilter::verify_step(const KalmanStep& next_step) {
  if (dx != next_step.getdx() || (dy != next_step.getdy()) ||
      (du != next_step.getdu()))
    throw(std::invalid_argument(
        "Dimension mismatch between step object and kalman filter"));
}
