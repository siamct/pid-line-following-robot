"""
Top-down animated simulation of the line follower (Tkinter, no extra packages).

    python simulate.py                 open the window
    python simulate.py --snapshot out.png   run 12 s and save a picture (needs Pillow)

Sliders change the PID gains and base speed live, so the effect of each term
can be seen straight away. The controller code being exercised is
firmware/controller.py, exactly as it runs on the Pico.
"""

import argparse
import math
import tkinter as tk
from tkinter import ttk

from simulator import Simulator, Gains, LOOP_DT
from track import BOARD_WIDTH, BOARD_HEIGHT

SCALE = 560                       # pixels per metre
CANVAS_W = int(BOARD_WIDTH * SCALE)
CANVAS_H = int(BOARD_HEIGHT * SCALE)
STEPS_PER_FRAME = 2               # 2 x 10 ms per 20 ms frame = real time
TRAIL_LENGTH = 600                # points of path to show behind the robot

COLOUR_BOARD = "#f4f2ec"
COLOUR_LINE = "#1a1a1a"
COLOUR_BODY = "#3b7dd8"
COLOUR_WHEEL = "#333333"
COLOUR_TRAIL = "#e07a2f"
COLOUR_TEXT = "#333333"


def to_px(x, y):
    """World metres to canvas pixels (y up in the world, down on screen)."""
    return x * SCALE, CANVAS_H - y * SCALE


def sensor_colour(value):
    """White for 0, black for 1."""
    g = int(255 * (1 - value))
    return "#%02x%02x%02x" % (g, g, g)


# ----------------------------------------------------------------------------
# Drawing backends: the same scene description is drawn to a Tk canvas for the
# live view and to a Pillow image for the --snapshot option.
# ----------------------------------------------------------------------------

class TkBackend:
    def __init__(self, canvas):
        self.c = canvas

    def clear(self):
        self.c.delete("all")

    def line(self, pts, fill, width, smooth=False):
        flat = [v for p in pts for v in to_px(*p)]
        self.c.create_line(*flat, fill=fill, width=width, capstyle=tk.ROUND, joinstyle=tk.ROUND, smooth=smooth)

    def polygon(self, pts, fill, outline=""):
        flat = [v for p in pts for v in to_px(*p)]
        self.c.create_polygon(*flat, fill=fill, outline=outline)

    def circle(self, x, y, r_px, fill, outline="#555555"):
        px, py = to_px(x, y)
        self.c.create_oval(px - r_px, py - r_px, px + r_px, py + r_px, fill=fill, outline=outline)

    def text(self, px, py, s, anchor="nw"):
        self.c.create_text(px, py, text=s, anchor=anchor, fill=COLOUR_TEXT, font=("Segoe UI", 10))


class PilBackend:
    def __init__(self, image):
        from PIL import ImageDraw, ImageFont
        self.draw = ImageDraw.Draw(image)
        try:
            self.font = ImageFont.truetype("segoeui.ttf", 14)
        except OSError:
            self.font = ImageFont.load_default()

    def clear(self):
        pass

    def line(self, pts, fill, width, smooth=False):
        px = [to_px(*p) for p in pts]
        self.draw.line(px, fill=fill, width=int(width), joint="curve")
        # Round the ends like the Tk capstyle.
        r = width / 2
        for (x, y) in (px[0], px[-1]):
            self.draw.ellipse((x - r, y - r, x + r, y + r), fill=fill)

    def polygon(self, pts, fill, outline=""):
        self.draw.polygon([to_px(*p) for p in pts], fill=fill, outline=outline or None)

    def circle(self, x, y, r_px, fill, outline="#555555"):
        px, py = to_px(x, y)
        self.draw.ellipse((px - r_px, py - r_px, px + r_px, py + r_px), fill=fill, outline=outline or None)

    def text(self, px, py, s, anchor="nw"):
        self.draw.text((px, py), s, fill=COLOUR_TEXT, font=self.font, anchor="la")


def draw_scene(backend, sim, show_trail=True):
    backend.clear()
    track = sim.track

    # Track: a closed thick line.
    pts = track.points + track.points[:1]
    backend.line(pts, COLOUR_LINE, track.line_width * SCALE)

    # Start marker.
    sx, sy, _ = track.start_pose()
    backend.circle(sx, sy, 5, "#2ca02c", outline="")

    # Trail of where the robot has been.
    if show_trail and len(sim.path) > 2:
        backend.line(sim.path[-TRAIL_LENGTH:], COLOUR_TRAIL, 2)

    # Robot.
    for wheel in sim.robot.wheel_outlines():
        backend.polygon(wheel, COLOUR_WHEEL)
    backend.polygon(sim.robot.body_outline(), COLOUR_BODY, outline="#1f4e8c")
    for (x, y), value in zip(sim.robot.sensor_positions(), sim.sensors):
        backend.circle(x, y, 4, sensor_colour(value))

    # Readout.
    m = sim.gains
    status = "line lost" if not sim.line_seen else "on line"
    backend.text(8, 6,
                 "t = %5.1f s   laps %d   error %+.2f (%s)   wheels L %.2f R %.2f   speed %.2f m/s" % (
                     sim.time, sim.laps, sim.error, status, sim.duty[0], sim.duty[1], sim.robot.speed))
    backend.text(8, 24, "Kp %.2f  Ki %.2f  Kd %.3f   base speed %.2f" % (m.kp, m.ki, m.kd, m.base_speed))


# ----------------------------------------------------------------------------
# The window
# ----------------------------------------------------------------------------

class App:
    def __init__(self, root):
        self.root = root
        root.title("Line follower simulation")
        root.resizable(False, False)

        self.sim = Simulator(Gains())
        self.running = False

        self.canvas = tk.Canvas(root, width=CANVAS_W, height=CANVAS_H, bg=COLOUR_BOARD, highlightthickness=0)
        self.canvas.grid(row=0, column=0, rowspan=2, padx=8, pady=8)
        self.backend = TkBackend(self.canvas)

        panel = ttk.Frame(root, padding=8)
        panel.grid(row=0, column=1, sticky="n")

        ttk.Label(panel, text="Controller", font=("Segoe UI", 10, "bold")).grid(row=0, column=0, sticky="w", pady=(0, 4))
        self.kp = self._slider(panel, 1, "Kp", 0.0, 2.0, self.sim.gains.kp, 0.05)
        self.ki = self._slider(panel, 2, "Ki", 0.0, 1.0, self.sim.gains.ki, 0.05)
        self.kd = self._slider(panel, 3, "Kd", 0.0, 0.20, self.sim.gains.kd, 0.005)
        self.base = self._slider(panel, 4, "Base speed", 0.10, 0.80, self.sim.gains.base_speed, 0.05)

        ttk.Label(panel, text="Robot", font=("Segoe UI", 10, "bold")).grid(row=5, column=0, sticky="w", pady=(10, 4))
        self.noise = self._slider(panel, 6, "Sensor noise", 0.0, 0.15, self.sim.robot.p.sensor_noise, 0.01)
        self.mismatch = self._slider(panel, 7, "Left motor gain", 0.80, 1.00, self.sim.robot.p.left_motor_gain, 0.01)

        buttons = ttk.Frame(panel)
        buttons.grid(row=8, column=0, pady=(12, 0), sticky="w")
        self.start_button = ttk.Button(buttons, text="Start", command=self.toggle, width=9)
        self.start_button.grid(row=0, column=0, padx=(0, 4))
        ttk.Button(buttons, text="Reset", command=self.reset, width=9).grid(row=0, column=1, padx=(0, 4))
        ttk.Button(buttons, text="Nudge", command=self.nudge, width=9).grid(row=0, column=2)

        presets = ttk.Frame(panel)
        presets.grid(row=9, column=0, pady=(10, 0), sticky="w")
        ttk.Label(presets, text="Presets:").grid(row=0, column=0, padx=(0, 4))
        for i, (name, g) in enumerate([("P only", (0.8, 0.0, 0.0)), ("PD", (0.8, 0.0, 0.06)), ("PID", (0.8, 0.2, 0.06))]):
            ttk.Button(presets, text=name, width=7, command=lambda g=g: self.preset(*g)).grid(row=0, column=i + 1, padx=2)

        self.metrics = tk.Label(panel, justify="left", anchor="nw", font=("Consolas", 9), fg=COLOUR_TEXT)
        self.metrics.grid(row=10, column=0, sticky="w", pady=(12, 0))

        help_text = ("Space: start / pause    R: reset\n"
                     "Nudge pushes the robot 3 cm sideways\n"
                     "to see how the controller recovers.\n"
                     "Sensors are drawn white (board) to\n"
                     "black (tape).")
        ttk.Label(panel, text=help_text, foreground="#666666").grid(row=11, column=0, sticky="w", pady=(12, 0))

        root.bind("<space>", lambda e: self.toggle())
        root.bind("r", lambda e: self.reset())
        root.bind("R", lambda e: self.reset())

        self.draw()
        self.root.after(20, self.tick)

    def _slider(self, parent, row, name, lo, hi, value, step):
        frame = ttk.Frame(parent)
        frame.grid(row=row, column=0, sticky="w")
        var = tk.DoubleVar(value=value)
        label = ttk.Label(frame, width=18)
        label.grid(row=0, column=0, sticky="w")

        def update(_=None):
            v = round(var.get() / step) * step
            label.config(text="%s: %.3g" % (name, v))
            self.apply_sliders()

        scale = ttk.Scale(frame, from_=lo, to=hi, variable=var, orient="horizontal", length=170, command=update)
        scale.grid(row=0, column=1)
        update()
        return var

    def apply_sliders(self):
        if not hasattr(self, "mismatch"):
            return  # still building the window
        g = self.sim.gains
        g.kp, g.ki, g.kd = self.kp.get(), self.ki.get(), self.kd.get()
        g.base_speed = self.base.get()
        self.sim.pid.kp, self.sim.pid.ki, self.sim.pid.kd = g.kp, g.ki, g.kd
        self.sim.robot.p.sensor_noise = self.noise.get()
        self.sim.robot.p.left_motor_gain = self.mismatch.get()

    def preset(self, kp, ki, kd):
        self.kp.set(kp)
        self.ki.set(ki)
        self.kd.set(kd)
        self.apply_sliders()
        self.reset()

    def toggle(self):
        self.running = not self.running
        self.start_button.config(text="Pause" if self.running else "Start")

    def reset(self):
        self.sim.reset()
        self.draw()

    def nudge(self):
        r = self.sim.robot
        r.x += -math.sin(r.theta) * 0.03
        r.y += math.cos(r.theta) * 0.03

    def tick(self):
        if self.running:
            for _ in range(STEPS_PER_FRAME):
                self.sim.step()
            self.draw()
        self.root.after(20, self.tick)

    def draw(self):
        draw_scene(self.backend, self.sim)
        if self.sim.history:
            m = self.sim.metrics()
            self.metrics.config(text=(
                "RMS error        %.3f\n"
                "max error        %.3f\n"
                "mean deviation   %.1f mm\n"
                "max deviation    %.1f mm\n"
                "line lost        %d times\n"
                "mean speed       %.2f m/s"
            ) % (m["rms_error"], m["max_error"], m["mean_deviation_mm"],
                 m["max_deviation_mm"], m["line_lost"], m["mean_speed_mps"]))


def snapshot(path, seconds=12.0, gains=None):
    """Run without a window and save a picture of the scene (needs Pillow)."""
    from PIL import Image
    sim = Simulator(gains or Gains())
    sim.run(seconds)
    image = Image.new("RGB", (CANVAS_W, CANVAS_H), COLOUR_BOARD)
    draw_scene(PilBackend(image), sim)
    image.save(path)
    print("saved", path)
    return sim.metrics()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--snapshot", metavar="PNG", help="run headless and save a picture")
    parser.add_argument("--seconds", type=float, default=12.0, help="seconds to simulate for --snapshot")
    args = parser.parse_args()

    if args.snapshot:
        snapshot(args.snapshot, args.seconds)
    else:
        root = tk.Tk()
        App(root)
        root.mainloop()
