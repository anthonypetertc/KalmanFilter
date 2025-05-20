#include <Eigen/Dense> // dynamic size matrices and good performance
#include <iostream>
#include <kalman_filter/kalman_step.hpp>


using namespace std;
using namespace Eigen;

class KalmanFilter {
    double dparams; // size of the vector of parameters being estimated.
    double dx; // size of the vector representing the independent variables.
    double dy; // size of the vector representing the dependent (measured) variables.

    VectorXd estimate; // current estimate of the parameters. Vector of size dparams.
    MatrixXd P; // current uncertainty of estimate, represented by a covariance matrix of size (dparams x dparams).
    MatrixXd K; // current Kalman Gain, a matrix of size (dparams x dy).

    public:
        //constructor: initialises the KalmanFilter object with an initial guess at the parameters,
        //and a covariance matrix representing the uncertainty around that guess.
        KalmanFilter(
            const VectorXd& init_estimate,
            const MatrixXd& init_P,
            double dx,
            double dy,
            double dparams
        );

        //predict
        VectorXd predict(); //TODO: what does this do?

        // update the object with a new measurement.
        void update(
            KalmanStep KS,
            MatrixXd& H, 
            MatrixXd& L);

    private:
        //private method to obtain P_n|n-1.
        void initial_update_covariance(
            KalmanStep KS,
            MatrixXd& L
        );

        //private method to obtain alpha_n|n-1
        //TODO: Best way to incorporate the dynamics?
        void initial_update_estimate(KalmanStep KS);

        //private method to compute Kalman Gain.
        void compute_kalman_gain(
            KalmanStep KS,
            MatrixXd H
        );

        //private method to to obtain P_n|n.
        void update_covariance(MatrixXd H);

        //private method to compute estimate
        //TODO: Best way to incorporate the dynamics?
        void update_estimate(KalmanStep KS);
};

/*
class KalmanFilter {

    public:
        // constructor
        KalmanFilter(double dt, double var_x, double var_v, double cov_xv, Vector2d& u);

        // predict, Kalman gain and update methods
        Vector2d predict();
        Matrix2d kalmanGain();

        // update method gets the measurement vector z passed to it
        Vector2d update(const Vector2d& z);

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

}; */