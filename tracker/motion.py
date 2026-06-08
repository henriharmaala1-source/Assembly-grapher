import cv2
import numpy as np


class KalmanCenter:
    """
    Constant-velocity Kalman filter over the object centre.

    State  = [x, y, vx, vy]   (position + velocity, units: pixels, pixels/frame)
    Measure = [x, y]          (the tracked box centre each frame)

    Provides smoothed position/velocity, multi-step prediction for the
    motion-vector arrow, and coasting (predict without measurement) to bridge
    occlusions and steer re-acquisition.
    """

    def __init__(self, process_noise: float = 3e-2, meas_noise: float = 1e-1):
        kf = cv2.KalmanFilter(4, 2)
        kf.transitionMatrix = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ], dtype=np.float32)
        kf.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=np.float32)
        kf.processNoiseCov     = np.eye(4, dtype=np.float32) * process_noise
        kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * meas_noise
        kf.errorCovPost        = np.eye(4, dtype=np.float32)
        self.kf = kf
        self.initialized = False

    def start(self, center):
        x, y = center
        self.kf.statePost = np.array([[x], [y], [0], [0]], dtype=np.float32)
        self.kf.statePre  = self.kf.statePost.copy()
        self.initialized = True

    def predict(self):
        """Advance one step; returns the a-priori predicted (x, y)."""
        p = self.kf.predict()
        return float(p[0]), float(p[1])

    def correct(self, center):
        """Fuse a measurement; returns the filtered (x, y)."""
        x, y = center
        m = np.array([[np.float32(x)], [np.float32(y)]])
        s = self.kf.correct(m)
        return float(s[0]), float(s[1])

    @property
    def position(self):
        return float(self.kf.statePost[0]), float(self.kf.statePost[1])

    @property
    def velocity(self):
        return float(self.kf.statePost[2]), float(self.kf.statePost[3])

    def project(self, steps: float):
        """Predicted centre `steps` frames into the future (no state change)."""
        x, y = self.position
        vx, vy = self.velocity
        return (x + vx * steps, y + vy * steps)
