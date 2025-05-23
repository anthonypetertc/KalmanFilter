
#include <iostream>
#include <Eigen/Dense> // for matrix and vector operations

#include "kalman_filter/kalman_filter.hpp" // include the header file where KalmanFilter is defined

// first we need the constructor, which is analogous to the init method in Python

KalmanFilter::KalmanFilter(
        const VectorXd& init_estimate,
        const MatrixXd& init_P,
        double dx,
        double dy,
        double dparams
    ): 

    // member initialisation list 
    estimate(init_estimate), // initialise the estimate
    P(init_P),  // initialise the covariance matrix
    dx(dx), // initialise the size of the vector of independent variables
    dy(dy), // initialise the size of the vector of dependent (measured) variables
    dparams(dparams) // initialise the size of the vector of parameters being estimated
    
    //constructor body 
    // since K depends on dparams and dy, we need to set it here rather than in the member initialisation list
    {K = MatrixXd::Zero(dparams, dy); // sets K to a zero matrix of size (dparams x dy)
        }

