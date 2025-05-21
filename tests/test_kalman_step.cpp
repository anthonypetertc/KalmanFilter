#define BOOST_TEST_MODULE KalmanStepTests
#include <Eigen/Dense>
#include <boost/test/included/unit_test.hpp>

#include "kalman_filter/kalman_step.hpp"

using Eigen::VectorXd, Eigen::MatrixXd;

BOOST_AUTO_TEST_CASE(TestKalmanStep) {
  int dx = 1;
  VectorXd x(dx);
  x << 0;

  int dy = 1;
  VectorXd y(dy);
  y << 0.1;

  MatrixXd R(dx, dy);
  R << 1;

  int dparams = 1;
  MatrixXd Q(dparams, dparams);
  Q << 0;

  KalmanStep kstep(x, y, R, Q, dx, dy, dparams);

  BOOST_CHECK_EQUAL(kstep.getx(), x);
  BOOST_CHECK_EQUAL(kstep.gety(), y);
  BOOST_CHECK_EQUAL(kstep.getR(), R);
  BOOST_CHECK_EQUAL(kstep.getQ(), Q);
  BOOST_CHECK_EQUAL(kstep.getdx(), dx);
  BOOST_CHECK_EQUAL(kstep.getdy(), dy);
  BOOST_CHECK_EQUAL(kstep.getdparams(), dparams);
}
