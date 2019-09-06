import numpy as np

class KalmanFilter(object):
    A = None   # linear recursion coeficient
    C = None   # state to measurement scale factor
    P = None   # prediction error
    G = None   # recursion gain
    R = None   # sensor RMS error
    X = None   # estimated state
    Z = None   # measured state
    Q = None   # prediction covariance

    def __init__(self):
        # x_k = A @ x_k-1 + B @ u_k
        self.A = None   # linear recursion coeficient
        self.P = None   # prediction error
        self.G = None   # recursion gain
        self.R = None   # sensor RMS error
        self.X = None   # estimated state
        self.Z = None   # measured state
        self.C = None   # scale factor
        self.Q = None   # prediction covariance (similar to R in measurement)

    def _predict(self):
        self.X = self.A @ self.X
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self, measurement):
        if (self.X is None):
            self.X = measurement
            return self.X
        self._predict()
        self.Z = measurement
        self.G = self.P @ self.C.T @ np.linalg.inv(self.C @ self.P @self.C.T + self.R)
        self.X = self.X + self.G @ (self.Z - self.C @ self.X)
        self.P = (np.identity(1) - self.G @ self.C) @ self.P
        #self.G = self.P @ self.C.T @ np.inv(self.C @ self.P @ self.C.T + self.R)
        #self.x = self.x + self.G @ (self.Z - self.C @ self.x)
        #self.P = (np.identity(...) - self.G @ self.C) @ self.P
        return self.X


class Bpm280Kalman(KalmanFilter):

    def __init__(self):
        self.A = np.matrix([[1.0]])
        self.P = np.matrix([[0.5]])
        self.R = np.matrix([[0.12]])
        self.C = np.matrix([[1.0]])
        self.Q = np.matrix([[0.000001]])  # smaller number = smoother result
        
    def height(self, pressure):
        """ hb = Ts/kT * [1- (pb/ps)^(R*kT/g0)] + hs
            ps, Ts, hs = reference surface pressure and temperature in geodetic height
            R = 287,1 - gas constant
            kT = 6.5*10-3 - temperature gradient
            g0 = 9.80665
            For stand-alone barometry:
            ps = 101.325 kPa
            Ts = 288.15 K
            then hb-hs is orhometric height up to 10.769km
        """
        return 44330.7692307692 * ( 1 - np.power(pressure/1013.25, 0.1902949572))
    
    def update(self, measurement):
        return super().update(self.height(measurement))
    
class BpmGPSFusionKalmanFilter(KalmanFilter):
    """ Assumes the barometer state is converted to altitude. No raw pressure. """
    def __init__(self):
        self.A = np.matrix([[1.0], [1.0]])
        self.C = np.matrix([[1.0], [1.0]])
        self.R = np.matrix([[1.0, 0.0], [0.0, 5.0]])   # 0.12 hPa or 1m, 5m for GPS
        self.P = np.matrix([[0.5]])
        self.Q = np.matrix([[0.000001]])  # smaller number = smoother result
        
    def update(self, measurement):
        """ For a missing measurement, just use the last state estimate as a measurement 
            but set the covariance matrix of the measurement to essentially infinity. 
            (If the system uses inverse covariance just set the values to zero.) This would 
            cause a Kalman filter to essentially ignore the new measurement since the ratio 
            of the variance of the prediction to the measurement is zero. The result will be 
            a new prediction that maintains velocity/acceleration but whose variance will grow 
            according to the process noise.
        """