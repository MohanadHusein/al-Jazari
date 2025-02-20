

class PIDController:
    def __init__(self, kp,ki, kd):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, set_point, measured_value, dt): # target value, measured value, time step
        error = set_point - measured_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        self.prev_error = error
        return output
