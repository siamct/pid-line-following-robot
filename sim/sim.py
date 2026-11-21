"""2D test track for the line-follower code.

Runs rule_based.py, pid_hybrid.py or pid_improved.py *unchanged* by giving
them a fake `robot` module (motors, sensors, button, timer) backed by a simple
differential-drive model. Good for comparing versions and tuning gains before
trying them on the real robot. The physics is rough, so treat results as
relative, not exact.

    python sim/sim.py pid_improved.py            # live window (tkinter)
    python sim/sim.py rule_based.py --headless   # just print the results
    python sim/sim.py --compare                  # all three, side by side
"""

import argparse
import importlib.util
import math
import os
import sys
import types

# --- Robot model (rough guesses, change to match yours) ---
WHEELBASE = 0.10        # m between wheels
TOP_SPEED = 0.60        # m/s of a wheel at speed 65535
MOTOR_LAG = 0.05        # s, how quickly a wheel reaches its new speed
SENSOR_AHEAD = 0.06     # m in front of the axle
SENSOR_SPACING = 0.015  # m between neighbouring sensors
LINE_WIDTH = 0.019      # m (19 mm tape)
ON_LINE, OFF_LINE = 60000, 8000   # raw readings over black / over white

SIM_TIME = 40.0         # s


def make_track():
    """Closed track built like a turtle drawing so every piece joins up:
    a long straight, a hairpin, an S-bend and two tight corners."""
    pts = [(0.1, 0.0)]
    state = {"x": 0.1, "y": 0.0, "h": 0.0}

    def straight(length, n=None):
        n = n or max(2, int(length / 0.02))
        for _ in range(n):
            state["x"] += length / n * math.cos(state["h"])
            state["y"] += length / n * math.sin(state["h"])
            pts.append((state["x"], state["y"]))

    def turn(radius, degrees):          # +ve = left, -ve = right
        steps = max(4, int(abs(degrees) / 4))
        da = math.radians(degrees) / steps
        for _ in range(steps):
            state["h"] += da / 2
            straight(radius * abs(da), 1)
            state["h"] += da / 2

    straight(1.1)
    turn(0.35, 180)     # hairpin
    straight(0.3)
    turn(0.15, -90)     # S-bend
    turn(0.15, 90)
    straight(0.5)
    turn(0.10, 90)      # tight corner
    straight(0.8)
    turn(0.10, 90)      # tight corner back onto the start straight
    return pts


def dist_to_track(px, py, track):
    best = 1e9
    for (x0, y0), (x1, y1) in zip(track, track[1:]):
        dx, dy = x1 - x0, y1 - y0
        l2 = dx * dx + dy * dy or 1e-12
        t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / l2))
        cx, cy = x0 + t * dx, y0 + t * dy
        best = min(best, math.hypot(px - cx, py - cy))
    return best


class FakeRobot:
    """Pretends to be the uni `robot` library."""

    def __init__(self, track):
        self.track = track
        self.x, self.y, self.heading = 0.15, 0.0, 0.0
        self.cmd = [0, 0]          # what the code asked for (0-65535)
        self.wheel = [0.0, 0.0]    # actual wheel speeds in m/s
        self.freq = 100
        self.callback = None

        sim = self

        class Motor:
            def __init__(self, i):
                self.i = i
            def speed(self, value):
                sim.cmd[self.i] = max(0, min(65535, int(value)))

        class Sensor:
            def __init__(self, offset):
                self.offset = offset
            def read(self):
                return sim.read_sensor(self.offset)

        # Same indexes the code uses: 5 = left, 3 = middle, 4 = right
        sensors = [Sensor(0)] * 6
        sensors[5] = Sensor(+SENSOR_SPACING)
        sensors[3] = Sensor(0.0)
        sensors[4] = Sensor(-SENSOR_SPACING)

        mod = types.ModuleType("robot")
        mod.motor = [Motor(0), Motor(1)]
        mod.sensor = sensors
        mod.isButtonToggleOn = lambda: True      # press "go" straight away
        def timer(frequency, callback):
            sim.freq, sim.callback = frequency, callback
        mod.timer = timer
        self.module = mod

    def sensor_pos(self, offset):
        c, s = math.cos(self.heading), math.sin(self.heading)
        return (self.x + SENSOR_AHEAD * c - offset * s,
                self.y + SENSOR_AHEAD * s + offset * c)

    def read_sensor(self, offset):
        d = dist_to_track(*self.sensor_pos(offset), self.track)
        # smooth edge so readings fade as a sensor slides off the tape
        k = max(0.0, min(1.0, (LINE_WIDTH - d) / LINE_WIDTH + 0.5))
        return int(OFF_LINE + (ON_LINE - OFF_LINE) * k)

    def step(self, dt):
        for i in (0, 1):
            target = self.cmd[i] / 65535 * TOP_SPEED
            self.wheel[i] += (target - self.wheel[i]) * min(1.0, dt / MOTOR_LAG)
        v = (self.wheel[0] + self.wheel[1]) / 2
        w = (self.wheel[1] - self.wheel[0]) / WHEELBASE
        self.heading += w * dt
        self.x += v * math.cos(self.heading) * dt
        self.y += v * math.sin(self.heading) * dt


def load_controller(path, fake):
    sys.modules["robot"] = fake.module
    spec = importlib.util.spec_from_file_location("controller", path)
    mod = importlib.util.module_from_spec(spec)
    import io, contextlib
    with contextlib.redirect_stdout(io.StringIO()):   # hide the code's prints
        spec.loader.exec_module(mod)
    return mod


def run(path, track, live=None):
    fake = FakeRobot(track)
    load_controller(path, fake)
    dt = 1.0 / fake.freq
    t, travelled, off_time, errors, trail = 0.0, 0.0, 0.0, [], []
    import io, contextlib
    while t < SIM_TIME:
        with contextlib.redirect_stdout(io.StringIO()):
            fake.callback()
        x0, y0 = fake.x, fake.y
        fake.step(dt)
        travelled += math.hypot(fake.x - x0, fake.y - y0)
        err = dist_to_track(fake.x, fake.y, track)
        errors.append(err)
        if err > LINE_WIDTH * 3:
            off_time += dt
        trail.append((fake.x, fake.y))
        if live and not live(fake, trail, t):
            break
        t += dt
    track_len = sum(math.dist(a, b) for a, b in zip(track, track[1:]))
    return {
        "name": os.path.basename(path),
        "laps": travelled / track_len,
        "avg_speed": travelled / t if t else 0,
        "mean_err_mm": 1000 * sum(errors) / len(errors),
        "off_line_s": off_time,
        "trail": trail,
    }


def report(r):
    print(f"{r['name']:<18} laps {r['laps']:5.2f}   avg {r['avg_speed']:.2f} m/s   "
          f"mean error {r['mean_err_mm']:5.1f} mm   off the line {r['off_line_s']:4.1f} s")


def live_window(path, track):
    import tkinter as tk
    scale, pad = 380, 40
    xs, ys = [p[0] for p in track], [p[1] for p in track]
    minx, maxy = min(xs) - 0.1, max(ys) + 0.1
    W = int((max(xs) - minx + 0.1) * scale) + 2 * pad
    H = int((maxy - min(ys) + 0.1) * scale) + 2 * pad
    to_px = lambda x, y: (pad + (x - minx) * scale, pad + (maxy - y) * scale)

    root = tk.Tk()
    root.title(f"line follower sim: {os.path.basename(path)}")
    c = tk.Canvas(root, width=W, height=H, bg="white")
    c.pack()
    c.create_line(*[v for p in track for v in to_px(*p)], width=LINE_WIDTH * scale, fill="black")
    info = c.create_text(10, 10, anchor="nw", font=("Consolas", 10), text="")
    state = {"frame": 0}

    def draw(fake, trail, t):
        state["frame"] += 1
        if state["frame"] % 3:
            return True
        c.delete("bot")
        if len(trail) > 1:
            c.create_line(*[v for p in trail[-400:] for v in to_px(*p)], fill="#ff4d4d", tags="bot")
        for off in (SENSOR_SPACING, 0, -SENSOR_SPACING):
            sx, sy = to_px(*fake.sensor_pos(off))
            c.create_oval(sx - 3, sy - 3, sx + 3, sy + 3, fill="#2f81f7", tags="bot")
        bx, by = to_px(fake.x, fake.y)
        c.create_oval(bx - 8, by - 8, bx + 8, by + 8, outline="#333", width=2, tags="bot")
        c.itemconfig(info, text=f"t = {t:4.1f} s   L {fake.cmd[0]:5d}   R {fake.cmd[1]:5d}")
        root.update()
        root.after(10)
        return True

    try:
        r = run(path, track, live=draw)
        report(r)
        root.mainloop()   # keep the final picture up until the window is closed
    except tk.TclError:
        pass   # window closed


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.dirname(here)
    ap = argparse.ArgumentParser()
    ap.add_argument("controller", nargs="?", default="pid_improved.py")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--compare", action="store_true")
    a = ap.parse_args()
    track = make_track()
    if a.compare:
        for f in ("rule_based.py", "pid_hybrid.py", "pid_improved.py"):
            report(run(os.path.join(repo, f), track))
        return
    path = a.controller if os.path.exists(a.controller) else os.path.join(repo, a.controller)
    if a.headless:
        report(run(path, track))
    else:
        live_window(path, track)


if __name__ == "__main__":
    main()
