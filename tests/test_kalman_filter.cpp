#define BOOST_TEST_MODULE KalmanFilterTests
#include <Eigen/Dense>
#include <boost/test/included/unit_test.hpp>
#include <stdexcept>
#include <vector>

#include "kalman_filter/kalman_filter.hpp"
#include "kalman_filter/kalman_step.hpp"

using Eigen::VectorXd, Eigen::MatrixXd;

BOOST_AUTO_TEST_CASE(TestKalmanFilter) {
  int du = 1;
  VectorXd u(du);
  u << 0;

  int dy = 1;
  VectorXd y(dy);
  y << 0.1;

  int dx = 1;

  MatrixXd init_P(dx, dx);
  init_P << 1.;

  VectorXd init_estimate(dx);
  init_estimate << 1;

  KalmanFilter kfilter(init_estimate, init_P, du, dy);

  BOOST_CHECK_EQUAL(kfilter.getdy(), dy);
  BOOST_CHECK_EQUAL(kfilter.getdu(), du);
  BOOST_CHECK_EQUAL(kfilter.getdx(), dx);
  BOOST_CHECK_EQUAL(kfilter.getP(), init_P);
  BOOST_CHECK_EQUAL(kfilter.getEstimate(), init_estimate);
  BOOST_TEST(kfilter.getKg().isZero());
}

BOOST_AUTO_TEST_CASE(TestVerifyStep) {
  VectorXd x0 = VectorXd::Zero(2);
  MatrixXd P0 = MatrixXd::Zero(2, 2);
  KalmanFilter kfilter(x0, P0, 1, 1);

  MatrixXd A = MatrixXd::Identity(3, 3);
  MatrixXd B = MatrixXd::Zero(3, 1);
  VectorXd u = VectorXd::Zero(1);
  MatrixXd H = MatrixXd::Identity(3, 3);
  MatrixXd Q = 0.1 * MatrixXd::Identity(3, 3);
  MatrixXd R = 0.01 * MatrixXd::Identity(3, 3);
  VectorXd y = VectorXd::Zero(3);

  KalmanStep step(y, u, A, B, R, Q, H);

  BOOST_CHECK_THROW(kfilter.update(step), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(TestUpdate) {
  Eigen::VectorXd init_estimate(1);
  init_estimate << 0.1;

  Eigen::MatrixXd init_P(1, 1);
  init_P << 1;

  int du = 1;
  int dy = 1;

  KalmanFilter kalmanfilter(init_estimate, init_P, du, dy);

  Eigen::VectorXd y(1);
  y << -0.1;

  Eigen::VectorXd u(1);
  u << 0;

  Eigen::MatrixXd A(1, 1);
  A << 1;

  Eigen::MatrixXd B(1, 1);
  B << 0;

  Eigen::MatrixXd R(1, 1);
  R << 1;

  Eigen::MatrixXd Q(1, 1);
  Q << 0;

  Eigen::MatrixXd H(1, 1);
  H << 1;

  KalmanStep next_step(y, u, A, B, R, Q, H);

  kalmanfilter.update(next_step);

  BOOST_CHECK_CLOSE(kalmanfilter.getKg()(0, 0), 0.5, 1e-6);
  BOOST_CHECK_CLOSE(kalmanfilter.getP()(0, 0), 0.5, 1e-6);
  BOOST_CHECK_SMALL(kalmanfilter.getEstimate()(0), 1e-6);
}

BOOST_AUTO_TEST_CASE(TestKGDecreasesWithNoise) {
  Eigen::VectorXd init_estimate(1);
  init_estimate << 0;

  Eigen::MatrixXd init_P(1, 1);
  init_P << 1;

  int du = 1;
  int dy = 1;
  KalmanFilter Kf(init_estimate, init_P, du, dy);

  Eigen::VectorXd y(1);
  y << -0.1;

  Eigen::VectorXd u(1);
  u << 0;

  Eigen::MatrixXd A(1, 1);
  A << 1;

  Eigen::MatrixXd B(1, 1);
  B << 0;

  Eigen::MatrixXd Q(1, 1);
  Q << 0;

  Eigen::MatrixXd H(1, 1);
  H << 1;

  std::vector<double> v1{0.001f, 1000.0f};
  std::vector<double> kgs;
  kgs.reserve(2);

  for (std::size_t k = 0; k < 2; ++k) {
    Eigen::MatrixXd R(1, 1);
    R << v1[k];

    KalmanStep next_step(y, u, A, B, R, Q, H);
    Kf.update(next_step);
    kgs.emplace_back(Kf.getKg()(0, 0));
  }

  BOOST_TEST(kgs[0] > kgs[1]);
}
