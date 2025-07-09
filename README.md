# KalmanFilter
A Kalman Filter is an object used for statistical analysis of dynamical systems. It provides a way to use knowledge of the underlying dynamics of the system to smooth out fluctuations due to measurement noise, or due to stochasticisty in the dynamics. For a quick introduction to the Kalman filter, see [here](https://www.intechopen.com/chapters/63164).

This repository contains an implementation of a simple Kalman filter, in C++.

## Build
To build the project run:

```
bash build.sh
```

To run the unit tests:

```
bash test.sh
```

## Example Usage
For an example of how to use the Kalman filter constructed here, see `examples/example_usage.cpp`. This file should have been compiled during the build step of the installation, to run executable:

```
build/bin/example
```

Note that running this example requires [gnuplot](http://www.gnuplot.info/).
