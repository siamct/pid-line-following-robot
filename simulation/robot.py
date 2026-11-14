"""
A simple physical model of the robot: differential drive with two small
gearmotors, three reflectance sensors on a bar ahead of the axle.

The model is deliberately a bit imperfect, like the real thing:
    - wheel speed lags the commanded duty (first-order response)
    - the left motor is slightly weaker than the right
    - below a small duty the motors do not turn at all (stiction)
    - sensor readings carry a little noise
"""

import math
import random


class RobotParams:
    wheelbase = 0.090          # m between the wheel contact points
    sensor_offset = 0.040      # m from the axle to the sensor bar
    sensor_spacing = 0.012     # m between adjacent sensors
    body_length = 0.110        # m, for drawing
    body_width = 0.080         # m, for drawing
    max_speed = 0.40           # m/s at full duty (N20 gearmotor, 32 mm wheels, 4.5 V)
    motor_time_constant = 0.20 # s, includes gearbox and chassis inertia
    left_motor_gain = 0.90     # left motor weaker than right
    right_motor_gain = 1.00
    duty_deadband = 0.08       # duty below which a motor does not turn
    sensor_noise = 0.03        # standard deviation of reflectance noise


class Robot:
    def __init__(self, params=None, rng=None):
        self.p = params or RobotParams()
        self.rng = rng or random.Random(1)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_left = 0.0
        self.v_right = 0.0

    def place(self, x, y, theta):
        self.x, self.y, self.theta = x, y, theta
        self.v_left = self.v_right = 0.0

    def _target_speed(self, duty, gain):
        if abs(duty) < self.p.duty_deadband:
            return 0.0
        return duty * gain * self.p.max_speed

    def step(self, duty_left, duty_right, dt):
        """Advance the robot by dt seconds under the given wheel duties."""
        p = self.p
        # Motors approach their target speed with a first-order lag.
        alpha = dt / (p.motor_time_constant + dt)
        self.v_left += alpha * (self._target_speed(duty_left, p.left_motor_gain) - self.v_left)
        self.v_right += alpha * (self._target_speed(duty_right, p.right_motor_gain) - self.v_right)

        # Differential-drive kinematics.
        v = 0.5 * (self.v_left + self.v_right)
        omega = (self.v_right - self.v_left) / p.wheelbase
        self.theta += omega * dt
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

    @property
    def speed(self):
        return 0.5 * (self.v_left + self.v_right)

    def sensor_positions(self):
        """World positions of the left, centre and right sensors."""
        p = self.p
        fx, fy = math.cos(self.theta), math.sin(self.theta)   # forward
        lx, ly = -fy, fx                                       # left
        cx = self.x + fx * p.sensor_offset
        cy = self.y + fy * p.sensor_offset
        s = p.sensor_spacing
        return (
            (cx + lx * s, cy + ly * s),   # left
            (cx, cy),                     # centre
            (cx - lx * s, cy - ly * s),   # right
        )

    def read_sensors(self, track):
        """Normalised reflectance at each sensor, with noise, clamped to 0..1."""
        out = []
        for sx, sy in self.sensor_positions():
            value = track.reflectance(sx, sy) + self.rng.gauss(0.0, self.p.sensor_noise)
            out.append(0.0 if value < 0 else 1.0 if value > 1 else value)
        return tuple(out)

    def body_outline(self):
        """Corners of the chassis rectangle in world coordinates, for drawing."""
        p = self.p
        fx, fy = math.cos(self.theta), math.sin(self.theta)
        lx, ly = -fy, fx
        front = p.body_length * 0.65
        back = p.body_length * 0.35
        half = p.body_width / 2
        return [
            (self.x + fx * front + lx * half, self.y + fy * front + ly * half),
            (self.x + fx * front - lx * half, self.y + fy * front - ly * half),
            (self.x - fx * back - lx * half, self.y - fy * back - ly * half),
            (self.x - fx * back + lx * half, self.y - fy * back + ly * half),
        ]

    def wheel_outlines(self, wheel_length=0.032, wheel_width=0.010):
        p = self.p
        fx, fy = math.cos(self.theta), math.sin(self.theta)
        lx, ly = -fy, fx
        out = []
        for side in (+1, -1):
            cx = self.x + lx * side * p.wheelbase / 2
            cy = self.y + ly * side * p.wheelbase / 2
            hl, hw = wheel_length / 2, wheel_width / 2
            out.append([
                (cx + fx * hl + lx * hw, cy + fy * hl + ly * hw),
                (cx + fx * hl - lx * hw, cy + fy * hl - ly * hw),
                (cx - fx * hl - lx * hw, cy - fy * hl - ly * hw),
                (cx - fx * hl + lx * hw, cy - fy * hl + ly * hw),
            ])
        return out
