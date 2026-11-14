"""
Runs the robot model against the track using the firmware's controller.

The controller module is imported from ../firmware so the simulation and the
robot share one implementation of the PID, line-position and motor-mixing
code. Only the hardware layers differ.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "firmware"))

from controller import PID, line_position, mix  # noqa: E402  (shared with the Pico firmware)
from robot import Robot, RobotParams            # noqa: E402
from track import Track                         # noqa: E402

LOOP_DT = 0.010  # 100 Hz, same as LOOP_PERIOD_MS in firmware/config.py


class Gains:
    def __init__(self, kp=0.8, ki=0.2, kd=0.06, base_speed=0.60, max_speed=0.8,
                 integral_limit=0.5, line_threshold=0.3):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.base_speed = base_speed
        self.max_speed = max_speed
        self.integral_limit = integral_limit
        self.line_threshold = line_threshold


class Simulator:
    def __init__(self, gains=None, track=None, params=None, seed=1):
        self.gains = gains or Gains()
        self.track = track or Track()
        self.robot = Robot(params or RobotParams(), rng=__import__("random").Random(seed))
        self.pid = PID(self.gains.kp, self.gains.ki, self.gains.kd,
                       integral_limit=self.gains.integral_limit)
        self.reset()

    def reset(self):
        x, y, theta = self.track.start_pose()
        self.robot.place(x, y, theta)
        self.pid.kp, self.pid.ki, self.pid.kd = self.gains.kp, self.gains.ki, self.gains.kd
        self.pid.integral_limit = self.gains.integral_limit
        self.pid.reset()
        self.time = 0.0
        self.last_error = 0.0
        self.error = 0.0
        self.line_seen = True
        self.sensors = (0.0, 0.0, 0.0)
        self.duty = (0.0, 0.0)
        self.laps = 0
        self.lost_count = 0
        self._was_lost = False
        self._last_progress = 0.0
        self.history = []   # (time, x, y, error, seen, speed)
        self.path = []

    def step(self):
        """One control period: read sensors, run the controller, move the robot."""
        g = self.gains
        self.sensors = self.robot.read_sensors(self.track)
        left, centre, right = self.sensors
        self.error, self.line_seen = line_position(left, centre, right, g.line_threshold, self.last_error)
        if self.line_seen:
            self.last_error = self.error

        correction = self.pid.update(self.error, LOOP_DT)
        self.duty = mix(g.base_speed, correction, g.max_speed)
        self.robot.step(self.duty[0], self.duty[1], LOOP_DT)
        self.time += LOOP_DT

        # Lap counting: progress wraps from near the end of the track to near the start.
        _, progress = self.track.nearest(self.robot.x, self.robot.y)
        if self._last_progress > 0.8 * self.track.length and progress < 0.2 * self.track.length:
            self.laps += 1
        self._last_progress = progress

        if not self.line_seen and not self._was_lost:
            self.lost_count += 1
        self._was_lost = not self.line_seen

        self.history.append((self.time, self.robot.x, self.robot.y, self.error, self.line_seen, self.robot.speed))
        self.path.append((self.robot.x, self.robot.y))

    def run(self, seconds):
        steps = int(round(seconds / LOOP_DT))
        for _ in range(steps):
            self.step()
        return self.metrics()

    def metrics(self):
        """Summary numbers for a run."""
        if not self.history:
            return {}
        errors = [h[3] for h in self.history if h[4]]
        n = max(1, len(errors))
        rms = (sum(e * e for e in errors) / n) ** 0.5
        max_abs = max((abs(e) for e in errors), default=0.0)
        mean_speed = sum(h[5] for h in self.history) / len(self.history)
        # Distance of the centre sensor from the line, in mm, as a physical measure.
        deviations = []
        for _, x, y, _, _, _ in self.history[::5]:
            d, _ = self.track.nearest(x, y)
            deviations.append(d)
        return {
            "time_s": self.time,
            "laps": self.laps,
            "rms_error": rms,
            "max_error": max_abs,
            "line_lost": self.lost_count,
            "mean_speed_mps": mean_speed,
            "max_deviation_mm": 1000 * max(deviations) if deviations else 0.0,
            "mean_deviation_mm": 1000 * sum(deviations) / len(deviations) if deviations else 0.0,
        }
