#include <iostream>
#include <Eigen/Dense> // dynamic size matrices and good performance
#include "kalman_filter/kf.h" // include the header file where KalmanFilter is defined

using namespace Eigen;
using namespace std;

KalmanFilter::KalmanFilter(double dt, double var_x, double var_v, double cov_xv, Vector2d& u) 
    : dt(dt), var_x(var_x), var_v(var_v), cov_xv(cov_xv), u(u) {
    
    // Initialize state vector
    x << 0, 0; // zero position, zero velocity
    
    // Initialize system matrices
    A << 1, dt,
         0, 1;
    
    B << dt*dt/2, 0,
         0, dt;
    
    H << 1, 0,
         0, 1;
    
    // Initialize covariance matrices
    P << var_x, cov_xv,
         cov_xv, var_v;
    
    // Process noise covariance matrix
    Q << 0, 0,
        0, 0;
    
    // Measurement noise covariance matrix
    R << 0.1, 0,
        0, 0.1;
    
    // identity matrix
    I << Matrix2d::Identity();
}

// predictions method
Vector2d KalmanFilter::predict() {
    
    // Predict the state
    x = A * x + B * u;
    
    // Predict the covariance
    P = A * P * A.transpose() + Q;
    
    return x;
}

// calculate Kalman gain
Matrix2d KalmanFilter::kalmanGain() {
    Matrix2d K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    
    return K;
}

// update method
Vector2d KalmanFilter::update(const Vector2d& z) {
    // extract the Kalman gain matrix
    Matrix2d K = kalmanGain();
    
    // update the state with the measurement
    x = x + K * (z - H * x);
    
    // update the covariance
    P = (I - K * H) * P;
    return x;
}
