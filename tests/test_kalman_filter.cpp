#define BOOST_TEST_MODULE KalmanFilterTests
#include <Eigen/Dense>
#include <boost/test/included/unit_test.hpp>
#include <stdexcept>

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
