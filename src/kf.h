#include <Eigen/Dense> // dynamic size matrices and good performance
#include <iostream>


using namespace std;
using namespace Eigen;

class KalmanFilter {

    public:
        // constructor
        KalmanFilter(double dt, double var_x, double var_v, double cov_xv, Vector2d& u);

        // predict, Kalman gain and update methods
        Vector2d predict();
        Matrix2d kalmanGain();
        Vector2d update();

    private:
        // member variables of the class
        double dt, var_x, var_v, cov_xv;
        
        // state vector
        Vector2d x;
        
        // control vector
        Vector2d u;
        
        // system matrices
        Matrix2d A, B, H;
        
        // covariance matrices
        Matrix2d P, Q, R;
        
        // identity matrix
        Matrix2d I;

    }