#include <iostream>
#include <string>
#include <Eigen/Dense> // dynamic size matrices and good performance

using namespace std;
using namespace Eigen;

class KalmanFilter {

    public:
        // declarations
        double dt, var_x, var_v, cov_xv; // time step, variance of x, variance of v, covariance of x and v 
        Vector2d x; // state vector
        Vector2d u; // control vector
        Matrix2d A; // state transition matrix
        Matrix2d B; // control matrix
        Matrix2d P; // state covariance matrix
        Matrix2d Q; // process noise covariance matrix
        Matrix2d R; // measurement noise covariance matrix
        Matrix2d H; // measurement matrix
        Matrix2d I; // identity matrix
        
        
        KalmanFilter(
            double dt,

        )

        // methods

        void predictions() {
            cout << "State to be predicted is " << x << endl;
        }

        void updates() {
            cout << "State to be updated is " << x << endl;
        }

        private:
            // time step counter
            int k; 

            // define matrices that can be dynamically sized
            MatrixXd A, B, Q, R, H, I;

            // initial state covariance matrix
            MatrixXd P0;

            // initial state vector, dynamically sized
            VectorXd x0;

            // control vector, dynamically sized
            VectorXd u;

            // matrix of states to store the predicted state at each timestep
            MatrixXd predictedx;

            // matrix of matrices to store the predicted state at each timestep
            MatrixXd predictedP;

            // matrix of states to store the updated state at each timestep
            MatrixXd updatedx;

            // matrix of matrices to store the updated state at each timestep
            MatrixXd updatedP;

            // matrix of states to store the kalman gain at each timestep
            MatrixXd KalmanMatrixes;

            // matrix to store error in prediction at each timestep
            MatrixXd errors;
};