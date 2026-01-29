import numpy as np
import time
import threading

class KalmanFilter:
    def __init__(self, x=0, y=0, vx=0, vy=0, dt=0.1):
        # State: [x, y, vx, vy] - Cartesian 2D position and velocity
        self.X = np.array([[x], [y], [vx], [vy]], dtype=np.float32)
        
        # State transition matrix F (Constant Velocity Model)
        # x_k = x_{k-1} + vx * dt
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float32)
        
        # Covariance matrix P
        self.P = np.eye(4, dtype=np.float32) * 10.0
        
        # Process noise covariance Q
        q = 5.0
        self.Q = np.array([
            [0.25*dt**4, 0, 0.5*dt**3, 0],
            [0, 0.25*dt**4, 0, 0.5*dt**3],
            [0.5*dt**3, 0, dt**2, 0],
            [0, 0.5*dt**3, 0, dt**2]
        ], dtype=np.float32) * q
        
        # Measurement noise covariance R (Polar space: range, angle, radial velocity)
        # range: 5cm, angle: 1 deg, Doppler: 0.2 m/s (20cm/s)
        self.R = np.diag([25.0, 1.0, 400.0]).astype(np.float32)

    def predict(self, dt=None):
        if dt is not None and dt > 0:
            self.F[0, 2] = dt
            self.F[1, 3] = dt
        
        self.X = self.F @ self.X
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.X

    def update(self, z):
        """
        EKF Update Step using Polar Measurements.
        
        Measurement vector z = [r, theta, v_radial]^T
        State vector X = [x, y, vx, vy]^T
        
        Non-linear Measurement function h(X):
        r = sqrt(x^2 + y^2)
        theta = arctan2(x, y)  <-- Radar angle is 0 at Y axis (depth)
        v_radial = (x*vx + y*vy) / r
        
        Jacobian H = dh/dX:
        [ dr/dx      dr/dy      dr/dvx     dr/dvy ]
        [ dtheta/dx  dtheta/dy  dtheta/dvx dtheta/dvy ]
        [ dv_r/dx    dv_r/dy    dv_r/dvx   dv_r/dvy ]
        """
        z = np.array(z, dtype=np.float32).reshape(3, 1)
        
        x, y = self.X[0, 0], self.X[1, 0]
        vx, vy = self.X[2, 0], self.X[3, 0]
        
        r_sq = x**2 + y**2
        r = np.sqrt(r_sq) + 1e-6
        
        # 1. Measurement Function h(X)
        h = np.array([
            [r],
            [np.degrees(np.arctan2(x, y))],
            [(x * vx + y * vy) / r]
        ], dtype=np.float32)
        
        # 2. Jacobian Matrix H
        # Partial derivatives for r
        dr_dx = x / r
        dr_dy = y / r
        
        # Partial derivatives for theta (degrees)
        # d/dx atan2(x,y) = y / (x^2 + y^2)
        # Scale by 180/pi for degrees
        deg_scale = 180.0 / np.pi
        dtheta_dx = (y / r_sq) * deg_scale
        dtheta_dy = (-x / r_sq) * deg_scale
        
        # Partial derivatives for v_radial
        dv_dx = (vx * r - x * (x * vx + y * vy) / r) / r_sq
        dv_dy = (vy * r - y * (x * vx + y * vy) / r) / r_sq
        dv_dvx = x / r
        dv_dvy = y / r
        
        H = np.array([
            [dr_dx,   dr_dy,   0,      0],
            [dtheta_dx, dtheta_dy, 0,      0],
            [dv_dx,   dv_dy,   dv_dvx, dv_dvy]
        ], dtype=np.float32)
        
        # 3. Kalman Gain and State Update
        y_innovation = z - h
        
        # Handle angle wrap-around if necessary (though radar ±30 deg is safe)
        if y_innovation[1, 0] > 180: y_innovation[1, 0] -= 360
        if y_innovation[1, 0] < -180: y_innovation[1, 0] += 360
        
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.X = self.X + (K @ y_innovation)
        self.P = (np.eye(4, dtype=np.float32) - (K @ H)) @ self.P

class Tracker:
    def __init__(self, max_age=10, min_hits=3, distance_threshold=200):
        self.max_age = max_age
        self.min_hits = min_hits
        self.distance_threshold = distance_threshold
        self.tracks = []
        self.next_id = 1
        self.lock = threading.Lock()
        
        # Noise statistics
        self.mag_mean = 35.0  # Slightly higher initial guess
        self.mag_std = 5.0
        self.mag_alpha = 0.01 

    def update(self, measurements, dt=0.1):
        # measurements is a list of (x, y, mag, doppler_speed)
        with self.lock:
            if dt <= 0:
                dt = 0.001
            
            if measurements:
                mags = [m[2] for m in measurements]
                batch_mean = np.mean(mags)
                batch_std = np.std(mags) if len(mags) > 1 else 2.0
                self.mag_mean = (1 - self.mag_alpha) * self.mag_mean + self.mag_alpha * batch_mean
                self.mag_std = (1 - self.mag_alpha) * self.mag_std + self.mag_alpha * batch_std

            # 1. Predict
            for track in self.tracks:
                track['kf'].predict(dt)
                track['age'] += 1
                track['time_since_update'] += 1

            # 2. Data Association
            unassigned_measurements = list(range(len(measurements)))
            if len(self.tracks) > 0 and len(measurements) > 0:
                for i, track in enumerate(self.tracks):
                    track_x = track['kf'].X[0, 0]
                    track_y = track['kf'].X[1, 0]
                    
                    best_dist = float('inf')
                    best_meas_idx = -1
                    
                    for j in unassigned_measurements:
                        m_dist, m_angle, m_mag, m_v = measurements[j]
                        # Converting measurement polar to cartesian for distance gating
                        m_angle_rad = np.radians(m_angle)
                        m_x = m_dist * np.sin(m_angle_rad)
                        m_y = m_dist * np.cos(m_angle_rad)
                        
                        dist = np.sqrt((track_x - m_x)**2 + (track_y - m_y)**2)
                        
                        if dist < best_dist and dist < self.distance_threshold:
                            best_dist = dist
                            best_meas_idx = j
                    
                    if best_meas_idx != -1:
                        md, ma, mag, mv = measurements[best_meas_idx]
                        z = [md, ma, mv]
                        track['kf'].update(z)
                        track['time_since_update'] = 0
                        track['hits'] += 1
                        track['avg_mag'] = 0.8 * track['avg_mag'] + 0.2 * mag
                        unassigned_measurements.remove(best_meas_idx)

            # 3. Create new tracks
            for idx in unassigned_measurements:
                md, ma, mag, mv = measurements[idx]
                if mag > (self.mag_mean + 1.5 * self.mag_std):
                    # Initialize Cartesian state from Polar measurement
                    angle_rad = np.radians(ma)
                    mx = md * np.sin(angle_rad)
                    my = md * np.cos(angle_rad)
                    vx = mv * np.sin(angle_rad)
                    vy = mv * np.cos(angle_rad)
                    
                    new_track = {
                        'kf': KalmanFilter(mx, my, vx, vy, dt),
                        'id': self.next_id,
                        'age': 1,
                        'hits': 1,
                        'time_since_update': 0,
                        'avg_mag': mag
                    }
                    self.tracks.append(new_track)
                    self.next_id += 1

            # 4. Remove dead tracks
            self.tracks = [t for t in self.tracks if t['time_since_update'] < self.max_age]

            # 5. Return tracks
            return self.get_active_tracks()

    def get_active_tracks(self):
        confirmed_tracks = []
        for t in self.tracks:
            # A track is "real" if it has persistence (hits) 
            # AND its average magnitude is above the statistical noise floor
            is_persistent = t['hits'] >= self.min_hits
            # Stricter requirement for "displayed" magnitude if we want to limit noise
            is_strong = t['avg_mag'] > (self.mag_mean + 1.2 * self.mag_std)
            
            if is_persistent and is_strong:
                confirmed_tracks.append({
                    'id': t['id'],
                    'x': float(t['kf'].X[0, 0]),
                    'y': float(t['kf'].X[1, 0]),
                    'vx': float(t['kf'].X[2, 0]),
                    'vy': float(t['kf'].X[3, 0]),
                    'time_since_update': t['time_since_update'],
                    'mag': t['avg_mag'],
                    'hits': t['hits']
                })
        
        # Sort by magnitude descending to pick the most significant ones
        confirmed_tracks.sort(key=lambda x: x['mag'], reverse=True)
        
        # Strictly limit to top 2 tracks as requested
        return confirmed_tracks[:2]
