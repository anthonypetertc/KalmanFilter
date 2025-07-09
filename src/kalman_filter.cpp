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
  // Verify that the new KalmanStep object is compatible with the KalmanFilter.
  verify_step(next_step);

  // Prediction of both the state and the uncertainty, based only on dynamics
  // and not on the measurement.
  predict_state(next_step.getA(), next_step.getB(), next_step.getu());
  predict_P(next_step.getA(), next_step.getQ());

  // Update of the kalman gain.
  update_kalman_gain(next_step.getH(), next_step.getR());

  // Update of the estimate and uncertainty based on the measurement, and the
  // kalman gain.
  update_P(next_step.getH());
  update_estimate(next_step.getH(), next_step.gety());
}

void KalmanFilter::verify_step(const KalmanStep& next_step) {
  if (dx != next_step.getdx() || (dy != next_step.getdy()) ||
      (du != next_step.getdu()))
    throw(std::invalid_argument(
        "Dimension mismatch between step object and kalman filter"));
}

void KalmanFilter::predict_state(const Eigen::MatrixXd& A,
                                 const Eigen::MatrixXd& B,
                                 const Eigen::VectorXd& u) {
  estimate = A * estimate + B * u;
}

void KalmanFilter::predict_P(const Eigen::MatrixXd& A,
                             const Eigen::MatrixXd& Q) {
  P = A * P * A.transpose() + Q;
}

void KalmanFilter::update_kalman_gain(const Eigen::MatrixXd& H,
                                      const Eigen::MatrixXd& R) {
  Kg = P * H.transpose() * (R + H * P * H.transpose()).inverse();
}

void KalmanFilter::update_P(const Eigen::MatrixXd& H) {
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dx, dx);
  P = (I - Kg * H) * P;
}

void KalmanFilter::update_estimate(const Eigen::MatrixXd& H,
                                   const Eigen::VectorXd& y) {
  estimate = estimate + Kg * (y - H * estimate);
}
