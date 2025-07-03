#define BOOST_TEST_MODULE KalmanFilterTests
#include <Eigen/Dense>
#include <boost/test/included/unit_test.hpp>

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
