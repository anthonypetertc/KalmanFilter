#define BOOST_TEST_MODULE KalmanStepTests
#include <Eigen/Dense>
#include <boost/test/included/unit_test.hpp>

#include "kalman_filter/kalman_step.hpp"

using Eigen::VectorXd, Eigen::MatrixXd;

BOOST_AUTO_TEST_CASE(TestKalmanStep) {
  int du = 1;
  VectorXd u(du);
  u << 0;

  int dy = 1;
  VectorXd y(dy);
  y << 0.1;

  int dx = 1;

  MatrixXd R(dy, dy);
  R << 1;

  MatrixXd Q(dx, dx);
  Q << 0;

  MatrixXd A(dx, dx);
  A << 0;

  MatrixXd B(dx, du);
  B << 0;

  MatrixXd H(dy, dx);
  H << 1;

  KalmanStep kstep(y, u, A, B, R, Q, H);

  BOOST_CHECK_EQUAL(kstep.gety(), y);
  BOOST_CHECK_EQUAL(kstep.getu(), u);
  BOOST_CHECK_EQUAL(kstep.getA(), A);
  BOOST_CHECK_EQUAL(kstep.getB(), B);
  BOOST_CHECK_EQUAL(kstep.getR(), R);
  BOOST_CHECK_EQUAL(kstep.getQ(), Q);
  BOOST_CHECK_EQUAL(kstep.getH(), H);
  BOOST_CHECK_EQUAL(kstep.getdx(), dx);
  BOOST_CHECK_EQUAL(kstep.getdy(), dy);
  BOOST_CHECK_EQUAL(kstep.getdu(), du);
}
