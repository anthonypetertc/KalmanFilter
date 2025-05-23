#include <Eigen/Dense> // dynamic size matrices and good performance
#include <iostream>


using namespace std;
using namespace Eigen;

class KalmanStep {
    double dx; // size of the vector representing the dependent variables.
    double dy; // size of the vector representing the independent variables.
    double dparams; // size of vector of parameters.

    VectorXd x; // dependent variables.
    VectorXd y; // independent variables (measured).
    MatrixXd R; // Measurement noise, should be of size (dy x dy)
    MatrixXd Q; // System noise, should be of size (dparams x dparams)


};