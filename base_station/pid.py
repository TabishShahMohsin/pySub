class PID:
    def __init__(self, KP, KI, KD, limit_max, limit_min, is_angle=False):
        self.kp = KP
        self.ki = KI
        self.kd = KD
        self.limit_max = limit_max
        self.limit_min = limit_min
        self.is_angle = is_angle # Enable for Yaw/Roll/Pitch

        self.integral = 0
        self.prev_error = 0

    def compute(self, measurement, setpoint, dt):
        error = setpoint - measurement
        
        # Logic for Angles: Shortest path wrap (e.g. 350->10 is +20, not -340)
        if self.is_angle:
            error = (error + 180) % (2 * 180) - 180

        # Proportional
        p_out = self.kp * error

        # Integral
        self.integral += error * dt
        # Anti-windup (clamp integral contribution)
        limit_i = abs(self.limit_max) * 0.5 
        self.integral = max(min(self.integral, limit_i), -limit_i)
        i_out = self.ki * self.integral

        # Derivative
        derivative = (error - self.prev_error) / dt
        d_out = self.kd * derivative

        output = p_out + i_out + d_out
        
        # Saturation
        output = max(min(output, self.limit_max), self.limit_min)
        
        self.prev_error = error
        return output