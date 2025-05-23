#include <iostream>
#include <fstream>   // For saving files
#include <vector>    // For std::vector (lists)
#include <random>   // for random number generation
#include <Eigen/Dense> // for matrix and vector operations


#include "kalman_filter/example.h"
#include "kalman_filter/kf1D.h" // include the header file where KalmanFilter is defined

using namespace std;
using namespace Eigen;


int main() {
    // std::cout << "Result: " << add(2, 3) << std::endl;
    // return 0;

    std::cout << "Starting the filter" << std::endl;

    // Initialize physical parameters
    double dt = 0.1; // time step
    double a = -9.81; // control vector (1D acceleration)
    double x0 = 10.0; // initial position
    double v0 = 25.0; // initial velocity
    double t_final = 7.5; // final time

    // print the physical parameters
    std::cout << "dt = " << dt <<  std::endl;
    std::cout << "a = " << a <<  std::endl;
    std::cout << "x0 = " << x0 <<  std::endl;
    std::cout << "v0 = " << v0 <<  std::endl;

    // create a list of times
    vector<double> times;
    for (double t = 0.0; t < t_final; t += dt) {
        times.push_back(t);
    }

    // print the final time in the time list
    std::cout << "Final time: " << times.back() << std::endl;

    // create list of trajector truth values
    vector<double> true_x;
    for (double t: times) {
        true_x.push_back(x0 + v0 * t + 0.5 * a * t * t);
    }
    // Initialize Kalman Filter parameters
    double var_x = 0.01; // variance of position
    double var_v = 0.01; // variance of velocity
    double cov_xv = 0.01; // covariance between position and velocity

    // print the Kalman Filter parameters
    std::cout << "var_x = " << var_x <<  std::endl;
    std::cout << "var_v = " << var_v <<  std::endl;
    std::cout << "cov_xv = " << cov_xv <<  std::endl;

    // Instantiate the control vector
    Vector2d u;
    u << 0, a; // acceleration on velocity component only

    // Instantiate the Kalman Filter
    KalmanFilter kalman(dt, x0, v0, var_x, var_v, cov_xv, u);

    // empty lists to store measurements, estimates and updates
    vector<double> measurements;
    vector<double> predictions;
    vector<double> updates;

    // random number for noise on measurements
    default_random_engine generator;
    normal_distribution<double> noise(0.0, 3.33); // noise(mean, stddev)

    // loop through true values of trajectory, apply noise to create measurements, 
    // then call the kalman filter

    for (double x : true_x) {
        // extract the predicted state from the filter
        Vector2d x_pred = kalman.predict();
        predictions.push_back(x_pred(0)); // store the predicted position (0th element of state vector)

        // create noisey measurement
        double z = x + noise(generator); // add noise to the true value
        measurements.push_back(z); // store the measurement

        // update the filter with the measurement
        Vector2d z_vec;
        z_vec << z, 0; // create a measurement vector (only position is measured, keep velocity 0)

        Vector2d x_update = kalman.update(z_vec);
        updates.push_back(x_update(0)); // store the updated position (0th element of state vector)
    }

    // Save the input parameters to a CSV file to be used in the Python script for comparisons
    std::ofstream param_file("kalman_params.csv");
    if (param_file.is_open()) {
        param_file << "name,value\n";
        param_file << "dt," << dt << "\n";
        param_file << "t_final," << t_final << "\n";
        param_file << "a," << a << "\n";
        param_file << "x0," << x0 << "\n";
        param_file << "v0," << v0 << "\n";
        param_file << "var_x," << var_x << "\n";
        param_file << "var_v," << var_v << "\n";
        param_file << "cov_xv," << cov_xv << "\n";
        param_file.close();
        std::cout << "Parameters saved to kalman_params.csv" << std::endl;
    } else {
        std::cerr << "Failed to open parameter file for writing." << std::endl;
    }

    // Save the Kalman Filter output to CSV for comparison with a Python implementation
    std::ofstream file("kalman_output.csv");
    if (file.is_open()) {
        file << "time,true,measurement,prediction,update\n";  // header

        for (size_t i = 0; i < times.size(); ++i) {
            file << times[i] << ","
                << true_x[i] << ","
                << measurements[i] << ","
                << predictions[i] << ","
                << updates[i] << "\n";
        }

        file.close();
        std::cout << "Data saved to kalman_output.csv" << std::endl;
    } else {
        std::cerr << "Failed to open file for writing." << std::endl;
    }



}