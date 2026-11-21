"""Line-follower simulator.

Runs rule_based.py, pid_hybrid.py and pid_improved.py *unchanged* by giving
them a fake `robot` module (motors, sensors, button, timer) and driving a
simple two-wheel robot around a top-down track.

    python sim/sim.py              # open the simulator window
    python sim/sim.py --compare    # no window, just run every version on every track

The robot model is rough (wheel speed, sensor spacing and motor lag are
guesses below), so use it to compare versions and try gains, not to predict
real lap times.
"""

import argparse
import contextlib
import importlib.util
import io
import math
import os
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

CONTROLLERS = [
    ("Rule-based", "rule_based.py"),
    ("PID hybrid", "pid_hybrid.py"),
    ("PID improved", "pid_improved.py"),
]

# --- Robot model (rough guesses, change to match yours) ---
WHEELBASE = 0.095       # m between the wheels
TOP_SPEED = 0.60        # m/s of a wheel at motor speed 65535
MOTOR_LAG = 0.05        # s, how quickly a wheel reaches a new speed
SENSOR_AHEAD = 0.06     # m from the axle to the sensor bar
SENSOR_SPACING = 0.015  # m between neighbouring sensors
LINE_WIDTH = 0.019      # m, 19 mm tape
ON_LINE, OFF_LINE = 60000, 8000   # raw readings over black / white
LOST_AFTER = 2.0        # s more than LOST_DIST off the line = run over
LOST_DIST = 0.08        # m


# ---------------------------------------------------------------- tracks ---

def spline_track(points, per_seg=24):
    """Smooth closed loop through the control points (Catmull-Rom)."""
    out = []
    n = len(points)
    for i in range(n):
        p0, p1, p2, p3 = (points[(i + k) % n] for k in (-1, 0, 1, 2))
        for s in range(per_seg):
            t = s / per_seg
            t2, t3 = t * t, t * t * t
            out.append(tuple(
                0.5 * (2 * p1[j] + (-p0[j] + p2[j]) * t
                       + (2 * p0[j] - 5 * p1[j] + 4 * p2[j] - p3[j]) * t2
                       + (-p0[j] + 3 * p1[j] - 3 * p2[j] + p3[j]) * t3)
                for j in (0, 1)))
    out.append(out[0])
    return out


TRACKS = [
    ("Track 1: loop", spline_track([
        (0.45, 0.15), (0.9, 0.15), (1.6, 0.15), (1.95, 0.45), (1.8, 0.85),
        (1.35, 0.8), (1.0, 0.6), (0.65, 0.9), (0.25, 0.95), (0.05, 0.55)])),
    ("Track 2: hairpins", spline_track([
        (0.45, 0.15), (1.0, 0.15), (1.8, 0.15), (2.0, 0.35), (1.8, 0.55),
        (1.0, 0.5), (0.7, 0.55), (0.75, 0.75), (1.6, 0.8), (1.75, 0.98),
        (1.5, 1.1), (0.3, 1.05), (0.05, 0.6)])),
    ("Track 3: twisty", spline_track([
        (0.35, 0.14), (0.65, 0.12), (1.0, 0.28), (1.35, 0.12), (1.85, 0.2),
        (1.95, 0.55), (1.7, 0.72), (1.8, 1.0), (1.3, 1.1), (1.0, 0.92),
        (0.7, 1.1), (0.3, 1.0), (0.4, 0.65), (0.08, 0.45)])),
]


class Track:
    def __init__(self, name, pts):
        self.name, self.pts = name, pts
        self.segs = list(zip(pts, pts[1:]))
        self.cum = [0.0]
        for a, b in self.segs:
            self.cum.append(self.cum[-1] + math.dist(a, b))
        self.length = self.cum[-1]
        (x0, y0), (x1, y1) = pts[0], pts[1]
        self.start_heading = math.atan2(y1 - y0, x1 - x0)

    def nearest(self, px, py, hint=None, window=40):
        """Distance to the centre line and how far round the lap that point is."""
        n = len(self.segs)
        idx = range(n) if hint is None else ((hint + k) % n for k in range(-window, window))
        best = (1e9, 0, 0.0)
        for i in idx:
            (x0, y0), (x1, y1) = self.segs[i]
            dx, dy = x1 - x0, y1 - y0
            l2 = dx * dx + dy * dy or 1e-12
            t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / l2))
            d = math.hypot(px - x0 - t * dx, py - y0 - t * dy)
            if d < best[0]:
                best = (d, i, t)
        d, i, t = best
        return d, i, self.cum[i] + t * (self.cum[i + 1] - self.cum[i])


# ----------------------------------------------------------------- robot ---

class SimRobot:
    """The car on the track, plus a fake `robot` module for the code to import."""

    def __init__(self, track):
        self.track = track
        x, y = track.pts[0]
        h = track.start_heading
        self.x, self.y, self.heading = x + 0.03 * math.cos(h), y + 0.03 * math.sin(h), h
        self.cmd = [0.0, 0.0]
        self.wheel = [0.0, 0.0]
        self.freq, self.callback = 100, None
        self.hint = 0
        self.prints = io.StringIO()

        sim = self

        class Motor:
            def __init__(self, i):
                self.i = i
            def speed(self, value):
                sim.cmd[self.i] = (int(value) & 0xFFFF) / 65535   # same 16-bit wrap as a PWM duty

        class Sensor:
            def __init__(self, offset):
                self.offset = offset
            def read(self):
                return sim.read_sensor(self.offset)

        # indexes the code uses: 5 = left, 3 = middle, 4 = right
        sensors = [Sensor(None)] * 6
        sensors[5], sensors[3], sensors[4] = Sensor(SENSOR_SPACING), Sensor(0.0), Sensor(-SENSOR_SPACING)

        mod = types.ModuleType("robot")
        mod.motor = (Motor(0), Motor(1))
        mod.sensor = tuple(sensors)
        mod.isButtonToggleOn = lambda: True
        mod.isButtonPressed = lambda: True
        def timer(frequency=100, callback=None):
            sim.freq, sim.callback = frequency, callback
        mod.timer = timer
        self.module = mod

    def sensor_pos(self, offset):
        c, s = math.cos(self.heading), math.sin(self.heading)
        return (self.x + SENSOR_AHEAD * c - offset * s, self.y + SENSOR_AHEAD * s + offset * c)

    def read_sensor(self, offset):
        if offset is None:
            return OFF_LINE
        d, _, _ = self.track.nearest(*self.sensor_pos(offset), hint=self.hint)
        k = max(0.0, min(1.0, (LINE_WIDTH - d) / LINE_WIDTH + 0.5))
        return int(OFF_LINE + (ON_LINE - OFF_LINE) * k)

    def tick(self, dt):
        with contextlib.redirect_stdout(self.prints):
            self.callback()
        for i in (0, 1):
            target = self.cmd[i] * TOP_SPEED
            self.wheel[i] += (target - self.wheel[i]) * min(1.0, dt / MOTOR_LAG)
        v = (self.wheel[0] + self.wheel[1]) / 2
        w = (self.wheel[1] - self.wheel[0]) / WHEELBASE
        self.heading += w * dt
        self.x += v * math.cos(self.heading) * dt
        self.y += v * math.sin(self.heading) * dt
        d, self.hint, along = self.track.nearest(self.x, self.y, hint=self.hint)
        return d, along


def load_controller(filename, bot):
    sys.modules["robot"] = bot.module
    spec = importlib.util.spec_from_file_location("controller", os.path.join(REPO, filename))
    mod = importlib.util.module_from_spec(spec)
    with contextlib.redirect_stdout(bot.prints):
        spec.loader.exec_module(mod)
    if bot.callback is None:
        raise RuntimeError(f"{filename} never called robot.timer()")


class Race:
    """Lap timing and off-track detection, shared by the window and --compare."""

    def __init__(self, track, filename):
        self.track = track
        self.bot = SimRobot(track)
        load_controller(filename, self.bot)
        self.dt = 1.0 / self.bot.freq
        self.t = 0.0
        self.lap_start = 0.0
        self.last_lap = self.best_lap = None
        self.laps = 0
        self.progress = 0.0     # distance round the lap, can go past track.length
        self.last_along = 0.0
        self.off_for = 0.0
        self.over = None

    def step(self):
        d, along = self.bot.tick(self.dt)
        self.t += self.dt
        delta = along - self.last_along
        if delta < -self.track.length / 2:
            delta += self.track.length
        elif delta > self.track.length / 2:
            delta -= self.track.length
        self.last_along = along
        self.progress += delta
        if self.progress >= (self.laps + 1) * self.track.length:
            self.laps += 1
            self.last_lap = self.t - self.lap_start
            self.best_lap = min(self.best_lap or 1e9, self.last_lap)
            self.lap_start = self.t
        self.off_for = self.off_for + self.dt if d > LOST_DIST else 0.0
        if self.off_for > LOST_AFTER:
            self.over = "Left the track"


def fmt(t):
    if t is None:
        return "--:--.--"
    return f"{int(t // 60):02d}:{t % 60:05.2f}"


# ---------------------------------------------------------------- window ---

def run_window():
    import tkinter as tk

    SCALE, PAD = 400, 30
    xs = [p[0] for tr in TRACKS for p in tr[1]]
    ys = [p[1] for tr in TRACKS for p in tr[1]]
    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    W = int((maxx - minx) * SCALE) + 2 * PAD
    H = int((maxy - miny) * SCALE) + 2 * PAD
    px = lambda x, y: (PAD + (x - minx) * SCALE, PAD + (maxy - y) * SCALE)

    root = tk.Tk()
    root.title("Line-follower sim")
    root.configure(bg="white")
    canvas = tk.Canvas(root, width=W, height=H, bg="#f2f2f2", highlightthickness=0)
    canvas.pack(padx=10, pady=(10, 0))

    timers = tk.Frame(root, bg="white")
    timers.pack(pady=6)
    labels = {}
    for col, name in enumerate(("This lap", "Last lap", "Best lap", "Laps")):
        v = tk.Label(timers, text="--:--.--", font=("Consolas", 22, "bold"), bg="white", fg="#222")
        v.grid(row=0, column=col, padx=18)
        tk.Label(timers, text=name, font=("Segoe UI", 9), bg="white", fg="#666").grid(row=1, column=col)
        labels[name] = v

    state = {"track": 0, "ctrl": 0, "race": None, "running": False, "fast": False}
    buttons = {"track": [], "ctrl": []}

    def styled(btn, on):
        btn.configure(relief="sunken" if on else "raised", bg="#ffe680" if on else "#f0f0f0")

    bar = tk.Frame(root, bg="white")
    bar.pack(pady=4)
    tk.Label(bar, text="Code:", bg="white").grid(row=0, column=0, sticky="e", padx=4)
    for i, (name, _) in enumerate(CONTROLLERS):
        b = tk.Button(bar, text=name, width=13, command=lambda i=i: choose("ctrl", i))
        b.grid(row=0, column=i + 1, padx=3, pady=2)
        buttons["ctrl"].append(b)
    tk.Label(bar, text="Track:", bg="white").grid(row=1, column=0, sticky="e", padx=4)
    for i, (name, _) in enumerate(TRACKS):
        b = tk.Button(bar, text=name.split(":")[0], width=13, command=lambda i=i: choose("track", i))
        b.grid(row=1, column=i + 1, padx=3, pady=2)
        buttons["track"].append(b)

    ctl = tk.Frame(root, bg="white")
    ctl.pack(pady=4)
    run_btn = tk.Button(ctl, text="Run", width=10, command=lambda: toggle_run())
    run_btn.pack(side="left", padx=4)
    tk.Button(ctl, text="Reset", width=10, command=lambda: reset()).pack(side="left", padx=4)
    fast_btn = tk.Button(ctl, text="Speed x1", width=10, command=lambda: toggle_fast())
    fast_btn.pack(side="left", padx=4)

    out = tk.Text(root, height=6, width=90, font=("Consolas", 9), bg="#fafafa", relief="solid", bd=1)
    out.pack(padx=10, pady=(4, 10))

    def log(msg):
        out.insert("end", msg + "\n")
        out.see("end")

    def draw_track():
        canvas.delete("all")
        name, pts = TRACKS[state["track"]]
        canvas.create_line(*[v for p in pts for v in px(*p)], width=LINE_WIDTH * SCALE,
                           fill="black", capstyle="round", joinstyle="round", smooth=True)
        tr = state["race"].track
        (x0, y0) = tr.pts[0]
        h = tr.start_heading
        nx, ny = -math.sin(h), math.cos(h)
        sq = 0.016
        fx, fy = math.cos(h), math.sin(h)
        for row in range(-3, 3):
            for colm in range(2):
                corners = []
                for du, dv in ((0, 0), (1, 0), (1, 1), (0, 1)):
                    u, v = (colm - 1 + du) * sq, (row + dv) * sq
                    corners.append(px(x0 + u * fx + v * nx, y0 + u * fy + v * ny))
                canvas.create_polygon(*[c for p in corners for c in p], outline="",
                                      fill="white" if (row + colm) % 2 else "#111")

    def draw_robot():
        canvas.delete("bot")
        bot = state["race"].bot
        c, s = math.cos(bot.heading), math.sin(bot.heading)
        def world(fx, fy):           # forward / left offsets from the axle centre
            return px(bot.x + fx * c - fy * s, bot.y + fx * s + fy * c)
        body = [world(-0.035, 0.04), world(0.03, 0.04), world(0.07, 0.008),
                world(0.07, -0.008), world(0.03, -0.04), world(-0.035, -0.04)]
        canvas.create_polygon(*[v for p in body for v in p], fill="#8a8a8a", outline="#555", tags="bot")
        for side in (1, -1):
            wheel = [world(-0.02, side * 0.04), world(0.02, side * 0.04),
                     world(0.02, side * 0.052), world(-0.02, side * 0.052)]
            canvas.create_polygon(*[v for p in wheel for v in p], fill="#333", tags="bot")
        for off in (SENSOR_SPACING, 0.0, -SENSOR_SPACING):
            sx, sy = px(*bot.sensor_pos(off))
            on = bot.read_sensor(off) > 35000
            canvas.create_oval(sx - 3, sy - 3, sx + 3, sy + 3, fill="#e03030" if on else "#2f81f7",
                               outline="", tags="bot")

    def update_labels():
        r = state["race"]
        labels["This lap"].config(text=fmt(r.t - r.lap_start))
        labels["Last lap"].config(text=fmt(r.last_lap))
        labels["Best lap"].config(text=fmt(r.best_lap))
        labels["Laps"].config(text=str(r.laps))

    def flush_prints():
        buf = state["race"].bot.prints
        text = buf.getvalue()
        if text:
            out.insert("end", text)
            out.see("end")
            buf.seek(0)
            buf.truncate()

    def reset():
        state["running"] = False
        run_btn.config(text="Run")
        name, fname = CONTROLLERS[state["ctrl"]]
        try:
            state["race"] = Race(Track(*TRACKS[state["track"]]), fname)
        except Exception as e:
            log(f"Could not load {fname}: {e}")
            return
        out.delete("1.0", "end")
        log(f"{name} ({fname}) on {TRACKS[state['track']][0]}. Press Run.")
        for kind in ("ctrl", "track"):
            for i, b in enumerate(buttons[kind]):
                styled(b, i == state[kind])
        draw_track()
        draw_robot()
        update_labels()

    def choose(kind, i):
        state[kind] = i
        reset()

    def toggle_run():
        if state["race"] is None or state["race"].over:
            return
        state["running"] = not state["running"]
        run_btn.config(text="Pause" if state["running"] else "Run")
        if state["running"]:
            state["wall"] = time.perf_counter()
            loop()

    def toggle_fast():
        state["fast"] = not state["fast"]
        fast_btn.config(text="Speed x4" if state["fast"] else "Speed x1")

    def loop():
        if not state["running"]:
            return
        r = state["race"]
        now = time.perf_counter()
        sim_time = min(0.1, now - state["wall"]) * (4 if state["fast"] else 1)
        state["wall"] = now
        steps = max(1, int(sim_time / r.dt))
        lap_before = r.laps
        for _ in range(steps):
            r.step()
            if r.over:
                break
        if r.laps > lap_before:
            log(f"Lap {r.laps}: {fmt(r.last_lap)}")
        flush_prints()
        draw_robot()
        update_labels()
        if r.over:
            state["running"] = False
            run_btn.config(text="Run")
            log(f"{r.over} after {r.t:.1f} s.")
            return
        root.after(20, loop)

    reset()
    root.mainloop()


# --------------------------------------------------------------- compare ---

def run_compare(seconds=150.0):
    print(f"{'':<14}" + "".join(f"{name:>22}" for name, _ in TRACKS))
    for cname, fname in CONTROLLERS:
        row = f"{cname:<14}"
        for tname, pts in TRACKS:
            r = Race(Track(tname, pts), fname)
            while r.t < seconds and not r.over:
                r.step()
            cell = f"best {fmt(r.best_lap)}" if r.best_lap else (r.over or "no full lap")
            row += f"{cell:>22}"
        print(row)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--compare", action="store_true", help="run every version on every track, no window")
    if ap.parse_args().compare:
        run_compare()
    else:
        run_window()
