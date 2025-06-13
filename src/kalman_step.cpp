#include "kalman_filter/kalman_step.hpp"

#include <Eigen/Dense>
#include <cassert>

KalmanStep::KalmanStep(const Eigen::VectorXd& y, const Eigen::VectorXd& u,
                       const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                       const Eigen::MatrixXd& R, const Eigen::MatrixXd& Q,
                       const Eigen::MatrixXd& H)
    : y(y),
      u(u),
      A(A),
      B(B),
      R(R),
      Q(Q),
      H(H),
      dy(y.size()),
      du(u.size()),
      dx(Q.rows()) {
  assert(dy == R.rows() && dy == R.cols());
  assert(du == B.cols());
  assert(dx == Q.cols() && dx == A.cols() && dx == A.rows());
  assert(dy == H.rows() && dx == H.cols());
}

int KalmanStep::getdx() const { return dx; }

int KalmanStep::getdy() const { return dy; }

int KalmanStep::getdu() const { return du; }

Eigen::VectorXd KalmanStep::gety() const { return y; }
Eigen::VectorXd KalmanStep::getu() const { return u; }
Eigen::VectorXd KalmanStep::getA() const { return A; }
Eigen::VectorXd KalmanStep::getB() const { return B; }
Eigen::MatrixXd KalmanStep::getR() const { return R; }
Eigen::MatrixXd KalmanStep::getQ() const { return Q; }
Eigen::MatrixXd KalmanStep::getH() const { return H; }
