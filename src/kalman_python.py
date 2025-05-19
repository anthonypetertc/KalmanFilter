# here is the Kalman filter class from python 
import numpy as np



class kf():
    
    """kalman filter class """

    def __init__(self, dt, var_x, var_v, cov_xv, u):

        self.dt = dt # time step for each loop iteration
        self.var_x = var_x # variance of position
        self.var_v = var_v # variance of velocity
        self.cov_xv = cov_xv # covariance of position and velocity
        
        # initial state
        self.x = np.array([[0], [0]])

        # initial covariance matrix
        self.P = np.array([[var_x, cov_xv], [cov_xv, var_v]]) 

        # updating matrices
        self.u = u # control input
        self.A = np.array([[1, self.dt], [0, 1]]) 
        self.B = np.array([[0.5*self.dt**2], [self.dt]]) 

        # noise covariance matrices
        self.Q = np.array([[0, 0], [0, 0]]) # process noise covariance 
        # this quantified uncertainity in the dynamics - e.g. it could represent wind or air resistance in our example model

        self.R = np.array([[0.1, 0], [0, 0.1]]) # measurement noise covariance
        
        # other matrices
        self.H = np.array([[1, 0], [0, 0]]) # measurements made only on position and not velocity
        self.I = np.eye(2) # identity matrix

    def predictions(self):
        """ predict the state and covariance matrix """
        # self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u) # state prediction        
        self.x = np.dot(self.A, self.x) + self.B*self.u # state prediction        
        self.P = np.dot(self.A, np.dot(self.P, self.A.T)) + self.Q # covariance prediction        
        return self.x

    def Kalman_gain(self):
        """ calculate the kalman gain """
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(self.P, np.dot(self.H.T, np.linalg.inv(S)))
        return K

    def updates(self, measurement):
        """updates depend on measurement outcome"""

        K = self.Kalman_gain() # calculate the kalman gain

        # update the state with the measurement
        self.x = self.x + np.dot(K, (measurement - np.dot(self.H, self.x)))
        
        # update the covariance matrix
        self.P = np.dot(self.I - np.dot(K, self.H), self.P)
        
        # return the updated state
        return self.x