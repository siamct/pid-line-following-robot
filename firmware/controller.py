"""
Line-following control logic.

This file has no hardware dependencies so the exact same code runs on the
Raspberry Pi Pico W (MicroPython) and inside the desktop simulation
(CPython). Everything hardware-specific lives in main.py, sensors.py and
motors.py.

Conventions
    error      -1.0 = line is fully under the left sensor
                0.0 = line is centred
               +1.0 = line is fully under the right sensor
    correction  positive = steer right (speed up the left wheel, slow the right)
    speeds      -1.0 .. +1.0 as a fraction of full PWM duty
"""


class PID:
    """A plain PID controller with integral clamping and output clamping."""

    def __init__(self, kp, ki, kd, integral_limit=1.0, output_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_error = None
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0

    def update(self, error, dt):
        """Return the correction for this error, dt seconds after the last call."""
        # Integral term, clamped so it cannot wind up while the robot is off the line.
        self.integral += error * dt
        if self.integral > self.integral_limit:
            self.integral = self.integral_limit
        elif self.integral < -self.integral_limit:
            self.integral = -self.integral_limit

        # Derivative of the error. On the first call there is no previous error.
        if self.previous_error is None or dt <= 0:
            derivative = 0.0
        else:
            derivative = (error - self.previous_error) / dt
        self.previous_error = error

        self.last_p = self.kp * error
        self.last_i = self.ki * self.integral
        self.last_d = self.kd * derivative
        output = self.last_p + self.last_i + self.last_d

        if output > self.output_limit:
            output = self.output_limit
        elif output < -self.output_limit:
            output = -self.output_limit
        return output


def line_position(left, centre, right, threshold=0.3, last_error=0.0):
    """
    Turn three normalised reflectance readings (0 = white, 1 = black) into a
    line position between -1 and +1.

    Uses a weighted average with weights -1, 0, +1, which gives a smoother
    error than treating each sensor as on/off. If the total signal is below
    the threshold the line has been lost; the robot then keeps steering
    towards the side where it last saw the line.

    Returns (error, line_seen).
    """
    total = left + centre + right
    if total < threshold:
        return (1.0 if last_error >= 0 else -1.0), False
    return (right - left) / total, True


def mix(base_speed, correction, max_speed=1.0):
    """
    Convert a base forward speed and a steering correction into left and
    right wheel speeds. Speeds may go negative so the robot can pivot on a
    tight corner when the correction is larger than the base speed.
    """
    left = base_speed + correction
    right = base_speed - correction
    left = max(-max_speed, min(max_speed, left))
    right = max(-max_speed, min(max_speed, right))
    return left, right
