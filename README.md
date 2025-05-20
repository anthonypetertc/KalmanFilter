# KalmanFilter
An implementation of the Kalman filter in C++

## Build
To build the project run:

```
bash build.sh
```

To run the unit tests:

```
bash test.sh
```

## Kalman Filter equations

### KalmanStep class
The aim of the Kalman Filter formalism is to provide an estimate of a vector of paramaters that we will denote by $\alpha$. The system is described by the parameters $\alpha$, a set of dependent variables $x$ and a set of indpendent variables $y$ which are regularly measured.

The whole system is constantly evolving dynamically. The parameters $\alpha$ undergo stochastic dynamics of the form:

$$
\alpha_n = g(\alpha_{n-1}) + w_n
$$

Where $w_n$ is a random vector drawn from a normal distribution with covariance $Q_n$. This covariance matrix is referred to as the *system noise*.

The measurements $y_n$ depend on the true parameters $\alpha_n$ and the dependent variables $x_n$ as well as a random variable $v_n$:

$$
y_n = f(x_n; \alpha_n) + v_n
$$

Where $v_n$ is drawn from a normal distribution with covariance $R_n$. This covariance matrix is referred to as the *measurement noise*.

The information for each measurement (including $x_n, y_n, Q_n, R_n$) are all stored in a KalmanStep object.

### KalmanFilter class
To infer the values of the parameters we use the Kalman Filter equations. This is what is done by the KalmanFilter object.

We will denote the prediction of the parameters $\alpha_n$ using only the first $n-1$ measurements by $\alpha_{n|n-1}$ and the prediction of $\alpha_n$ after $n$ measurements by $\alpha_{n|n}$. These predictions each have some uncertainty around them, which is given by $P_{n|n-1}$ in the first case and $P_{n|n}$ in the second case.

The KalmanFilter equations that we use to update the estimate of the parameters given a new measurement are the following:

$$
P_{n|n-1} = L_{n-1} P_{n-1|n-1} L_{n-1}^T + Q_n
$$

$$
\alpha_{n|n-1} = g(\alpha_{n-1|n-1})
$$

$$
K_n = P_{n|n-1} H_{n-1}^T (H_{n-1}P_{n|n-1}H_{n-1}^T + R_n)^{-1}
$$

$$
P_{n|n} = (I - K_n H_{n-1}) P_{n|n-1}
$$


$$
\alpha_{n|n} = \alpha_{n|n-1} + K_n \cdot (y_n - f(x_n; \alpha_{n|n-1}))
$$

In the above we have introduced the following matrices, which are related to the dynamics of the system:

$$
(L_{n-1})_{ij} = \frac{\partial g_i}{\partial \alpha_j}\bigg|_{\alpha = \alpha_{n-1|n-1}} 
$$

$$
(H_{n-1})_{ij} = \frac{\partial f_i(x; \alpha)}{\partial \alpha_j}\bigg|_{\alpha = \alpha_{n-1|n-1}}
$$