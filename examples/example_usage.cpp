#include <matplot/matplot.h>

#include <Eigen/Dense>
#include <cmath>
#include <random>
#include <vector>

#include "kalman_filter/kalman_filter.hpp"
#include "kalman_filter/kalman_step.hpp"
using Eigen::VectorXd, Eigen::MatrixXd;
using matplot::figure, matplot::plot, matplot::iota, matplot::hold, matplot::on,
    matplot::xlabel, matplot::ylabel, matplot::grid, matplot::show;

int main() {
  // Initial set-up of problem.
  double dt = 0.1;                                  // discrete timestep.
  std::vector<double> times = iota(0.0, dt, 50.0);  // timesteps.
  double v0 = 12;   // constant part of the initial velocity.
  double a = -5;    // constant part of the initial acceleration.
  double k = 1000;  // amplitude of oscillatory component of dynamics.
  double w = 1;     // frequency of oscillatory component of dynamics.

  std::vector<double> true_x;  // stores the exact dynamics.
  true_x.reserve(times.size());
  std::transform(
      times.begin(), times.end(), std::back_inserter(true_x), [&](double t) {
        return v0 * t + 0.5 * a * t * t - (k / (w * w)) * std::sin(w * t);
      });

  // Now, build the initial KalmanFilter object.
  VectorXd init_estimate(2);
  init_estimate << 0, v0 - k / w;

  MatrixXd init_P = MatrixXd::Identity(2, 2);
  int du = 1;
  int dy = 1;

  KalmanFilter kalman_filter(init_estimate, init_P, du, dy);

  // Now update the KalmanFilter with new measurements, using the KalmanStep
  // class.
  MatrixXd A(2, 2);
  A << 1, dt, 0, 1;
  MatrixXd B(2, 1);
  B << dt * dt / 2, dt;

  MatrixXd Q = MatrixXd::Zero(2, 2);  // No Process Noise.
  MatrixXd H(1, 2);  // Only measuring position, and not velocity.
  H << 1, 0;

  MatrixXd R(1, 1);  // Measurement noise covariance.
  double sigma = 400;
  R << sigma * sigma;

  // Normal distribution to generate measurement noise.
  static thread_local std::mt19937_64 rng{std::random_device{}()};
  std::normal_distribution<double> N(0.0, sigma);

  std::vector<double> measured_x, u_vector, estimates;
  measured_x.reserve(times.size() - 1);
  u_vector.reserve(times.size() - 1);
  estimates.reserve(times.size() - 1);

  // Loop to generate the noisy measurements and update the Kalman Filter.
  for (std::size_t i = 1; i < times.size(); ++i) {
    double t = times[i];
    double mx = true_x[i] + N(rng);      // noisy measurement
    double u = a + k * std::sin(w * t);  // control (acceleration)

    measured_x.push_back(mx);
    u_vector.push_back(u);

    Eigen::VectorXd y(1);
    y << mx;
    Eigen::VectorXd uu(1);
    uu << u;

    KalmanStep step(y, uu, A, B, R, Q, H);
    kalman_filter.update(step);

    estimates.push_back(kalman_filter.getEstimate()(0));  // x position
  }

  // Plotting the results.
  std::vector<double> meas_times(times.begin() + 1, times.end());

  auto f = figure();
  hold(on);

  auto exact = plot(times, true_x);
  exact->display_name("Exact trajectory");
  exact->line_width(3.5);

  auto meas = plot(meas_times, measured_x, "^");
  meas->display_name("Measurements");
  meas->marker_size(6);
  meas->marker_color({0.0, 0.30, 0.0});
  // meas->marker_face_color({0.1, 0.0, 0.0});

  auto est = plot(meas_times, estimates);
  est->display_name("Kalman Filter: Estimates");
  est->line_width(3.5);
  est->line_style("--");
  est->color({0.545, 0.0, 0.0});

  xlabel("t [s]");
  ylabel("x(t) [m]");

  grid(on);
  matplot::legend(
      {"Exact trajectory", "Measurement", "Kalman Filter: Estimates"});
  show();
}
