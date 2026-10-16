"""PID controller with angle wrapping and anti-windup."""


def wrap_angle(angle):
    """Wrap an angle in degrees to [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


class PID:
    def __init__(self, kp, ki, kd, limit_max, limit_min, is_angle=False):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit_max = limit_max
        self.limit_min = limit_min
        self.is_angle = is_angle  # Enable for yaw / roll / pitch

        self.integral = 0.0
        self.prev_error = None
        self.output = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = None
        self.output = 0.0

    def compute(self, measurement, setpoint, dt):
        # A zero or negative dt would divide by zero in the derivative term.
        if dt <= 0:
            return self.output

        error = setpoint - measurement

        # Shortest-path wrap for angles (e.g. 350 -> 10 is +20, not -340)
        if self.is_angle:
            error = wrap_angle(error)

        # Proportional
        p_out = self.kp * error

        # Integral, with anti-windup: limit the integral *contribution* to half
        # of the output range (previously the raw integral was clamped, which
        # made the limit depend on the units of the error instead).
        self.integral += error * dt
        if self.ki != 0:
            i_limit = 0.5 * max(abs(self.limit_max), abs(self.limit_min)) / abs(self.ki)
            self.integral = max(min(self.integral, i_limit), -i_limit)
        i_out = self.ki * self.integral

        # Derivative. Skip it on the first sample to avoid a kick from
        # prev_error = 0, and wrap the difference for angles so crossing
        # +/-180 does not look like a 360 degree jump.
        if self.prev_error is None:
            d_out = 0.0
        else:
            delta = error - self.prev_error
            if self.is_angle:
                delta = wrap_angle(delta)
            d_out = self.kd * delta / dt

        output = p_out + i_out + d_out

        # Saturation
        output = max(min(output, self.limit_max), self.limit_min)

        self.prev_error = error
        self.output = output
        return output
