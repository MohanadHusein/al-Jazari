import math


class KalmanFilter3D:
    def __init__(self, process_var=0.01, sensor_var=0.1, estimate_var=1.0):
        # Initialize the state vector [roll, pitch, yaw]
        self.x = [0.0, 0.0, 0.0]  # Initial state [roll, pitch, yaw]

        # Initialize the uncertainty matrix P
        self.P = [
            [estimate_var, 0, 0],
            [0, estimate_var, 0],
            [0, 0, estimate_var]
        ]

        # Process noise covariance (Q)
        self.Q = [
            [process_var, 0, 0],
            [0, process_var, 0],
            [0, 0, process_var]
        ]

        # Measurement noise covariance (R)
        self.R = [
            [sensor_var, 0, 0],
            [0, sensor_var, 0],
            [0, 0, sensor_var]
        ]

    def predict(self, gyro_roll, gyro_pitch, gyro_yaw, dt):
        """Prediction step using gyro data"""
        # Predict the next state based on the gyroscope
        self.x[0] += gyro_roll * dt  # Update roll
        self.x[1] += gyro_pitch * dt  # Update pitch
        self.x[2] += gyro_yaw * dt  # Update yaw (integrate gyro yaw)

        # Update the uncertainty matrix (P)
        self.P[0][0] += self.Q[0][0]
        self.P[1][1] += self.Q[1][1]
        self.P[2][2] += self.Q[2][2]

    def update(self, accel_x, accel_y, accel_z):
        """Update step using accelerometer data"""
        # Calculate roll and pitch from accelerometer data
        accel_roll = math.atan2(accel_y, accel_z) * 180 / math.pi
        accel_pitch = math.atan2(-accel_x, math.sqrt(accel_y ** 2 + accel_z ** 2)) * 180 / math.pi

        # Compute the Kalman gain
        K = [
            [self.P[0][0] / (self.P[0][0] + self.R[0][0]), 0, 0],
            [0, self.P[1][1] / (self.P[1][1] + self.R[1][1]), 0],
            [0, 0, self.P[2][2] / (self.P[2][2] + self.R[2][2])]
        ]

        # Correct the state estimates using the Kalman gain
        self.x[0] += K[0][0] * (accel_roll - self.x[0])  # Correct roll
        self.x[1] += K[1][1] * (accel_pitch - self.x[1])  # Correct pitch
        # We don't use yaw from the accelerometer, so we leave yaw as is (no correction here)

        # Update the uncertainty matrix (P)
        self.P[0][0] *= (1 - K[0][0])
        self.P[1][1] *= (1 - K[1][1])
        self.P[2][2] *= (1 - K[2][2])

        # Return the filtered roll, pitch, yaw
        return self.x[0], self.x[1], self.x[2]

    def filter(self, accel_x, accel_y, accel_z, gyro_roll, gyro_pitch, gyro_yaw, dt):
        """Call this function every loop with new data to get filtered roll, pitch, yaw"""
        self.predict(gyro_roll, gyro_pitch, gyro_yaw, dt)
        return self.update(accel_x, accel_y, accel_z)