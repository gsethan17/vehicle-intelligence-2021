import numpy as np
from math import sqrt
from math import atan2
from tools import Jacobian

class KalmanFilter:
    def __init__(self, x_in, P_in, F_in, H_in, R_in, Q_in):
        self.x = x_in
        self.P = P_in
        self.F = F_in
        self.H = H_in
        self.R = R_in
        self.Q = Q_in

    def predict(self):
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z):
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        # Calculate new estimates
        self.x = self.x + np.dot(K, z - np.dot(self.H, self.x))
        self.P = self.P - np.dot(np.dot(K, self.H), self.P)

    def update_ekf(self, z):
        # TODO: Implement EKF update for radar measurements
        # 1. Compute Jacobian Matrix H_j
        # H_j = [
        #        [drho/dp_x, drho/dp_y, drho/dv_x, drho/dv_y]
        #        [dphi/dp_x, dphi/dp_y, dphi/dv_x, dphi/dv_y]
        #        [drho_dot/dp_x, drho_dot/dp_y, drho_dot/dv_x, drho_dot/dv_y]
        #        ]
        # p_x = self.x[0], # p_y = self.x[1], # v_x = self.x[2], v_y = self.x[3]
        # H_j = [
        #        [self.x[0] / np.sqrt(np.power(self.x[0],2)+np.power(self.x[1],2)),
        #         self.x[1] / np.sqrt(np.power(self.x[0],2)+np.power(self.x[1],2)), 0, 0],
        #        [-(self.x[1] / (np.power(self.x[0],2)+np.power(self.x[1],2))),
        #         self.x[0] / (np.power(self.x[0],2)+np.power(self.x[1],2)), 0, 0],
        #        [(self.x[1]*((self.x[2]*self.x[1])-(self.x[3]*self.x[0]))) / (np.power((np.power(self.x[0],2)+np.power(self.x[1],2)),(3/2))),
        #         (self.x[0]*((self.x[3]*self.x[0])-(self.x[2]*self.x[1]))) / (np.power((np.power(self.x[0],2)+np.power(self.x[1],2)),(3/2))),
        #         self.x[0] / np.sqrt(np.power(self.x[0],2)+np.power(self.x[1],2)),
        #         self.x[1] / np.sqrt(np.power(self.x[0],2)+np.power(self.x[1],2))]
        #        ]
        H_j = Jacobian(self.x)

        # 2. Calculate S = H_j * P' * H_j^T + R
        S = np.dot(np.dot(H_j, self.P), H_j.T) + self.R

        # 3. Calculate Kalman gain K = H_j * P' * Hj^T + R
        K = np.dot(np.dot(self.P, H_j.T), np.linalg.inv(S))

        # 4. Estimate y = z - h(x')
        y = z - [np.sqrt(np.power(self.x[0], 2)+np.power(self.x[1], 2)),
                 atan2(self.x[1], self.x[0]),
                 (self.x[0]*self.x[2]+self.x[1]*self.x[3])/(np.sqrt(np.power(self.x[0], 2)+np.power(self.x[1], 2)))]

        # 5. Normalize phi so that it is between -PI and +PI
        if y[1] > np.pi :
            y[1] = 2*np.pi - y[1]
        elif y[1] < -np.pi :
            y[1] = 2*np.pi + y[1]

        # 6. Calculate new estimates
        #    x = x' + K * y
        self.x = self.x + np.dot(K, y)
        #    P = (I - K * H_j) * P
        self.P = self.P - np.dot(np.dot(K, H_j), self.P)
