"""
A closed test track made of black tape on a white board.

The centre line is a Catmull-Rom spline through a handful of waypoints,
sampled into a polyline. The default track has straights, gentle curves,
an S-bend and one tight corner, which is roughly what the competition
track looked like.
"""

import math

# Board size in metres (an A0 sheet is 1.189 x 0.841; this is a bit bigger).
BOARD_WIDTH = 1.6
BOARD_HEIGHT = 1.0

# Black electrical tape is about 19 mm wide.
LINE_WIDTH = 0.019

DEFAULT_WAYPOINTS = [
    (0.20, 0.50),
    (0.35, 0.22),
    (0.75, 0.15),
    (1.10, 0.28),
    (1.42, 0.22),
    (1.48, 0.55),
    (1.28, 0.78),
    (1.05, 0.58),
    (0.85, 0.82),
    (0.55, 0.86),
    (0.28, 0.78),
]


def catmull_rom_closed(points, samples_per_segment=24):
    """Sample a closed Catmull-Rom spline through the given points."""
    n = len(points)
    out = []
    for i in range(n):
        p0 = points[(i - 1) % n]
        p1 = points[i]
        p2 = points[(i + 1) % n]
        p3 = points[(i + 2) % n]
        for s in range(samples_per_segment):
            t = s / samples_per_segment
            t2, t3 = t * t, t * t * t
            x = 0.5 * ((2 * p1[0]) + (-p0[0] + p2[0]) * t
                       + (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2
                       + (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3)
            y = 0.5 * ((2 * p1[1]) + (-p0[1] + p2[1]) * t
                       + (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2
                       + (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3)
            out.append((x, y))
    return out


class Track:
    def __init__(self, waypoints=None, line_width=LINE_WIDTH):
        self.points = catmull_rom_closed(waypoints or DEFAULT_WAYPOINTS)
        self.line_width = line_width
        n = len(self.points)
        # Precompute segments and cumulative length for progress / lap counting.
        self.segments = []
        self.cumulative = [0.0]
        for i in range(n):
            a = self.points[i]
            b = self.points[(i + 1) % n]
            dx, dy = b[0] - a[0], b[1] - a[1]
            length = math.hypot(dx, dy)
            self.segments.append((a, b, dx, dy, length * length))
            self.cumulative.append(self.cumulative[-1] + length)
        self.length = self.cumulative[-1]

    def start_pose(self):
        """Position and heading at the first track point."""
        a, b = self.points[0], self.points[1]
        return a[0], a[1], math.atan2(b[1] - a[1], b[0] - a[0])

    def nearest(self, x, y):
        """Return (distance to the centre line, distance along the track) for a point."""
        best_d2 = float("inf")
        best_s = 0.0
        for i, (a, b, dx, dy, len2) in enumerate(self.segments):
            if len2 == 0:
                continue
            t = ((x - a[0]) * dx + (y - a[1]) * dy) / len2
            if t < 0:
                t = 0.0
            elif t > 1:
                t = 1.0
            px, py = a[0] + t * dx, a[1] + t * dy
            d2 = (x - px) ** 2 + (y - py) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_s = self.cumulative[i] + t * math.sqrt(len2)
        return math.sqrt(best_d2), best_s

    def reflectance(self, x, y, spot_radius=0.004):
        """
        What a reflectance sensor sees at (x, y): 1.0 over black tape, 0.0 over
        white board, and a linear ramp across the tape edge the width of the
        sensor's illuminated spot.
        """
        distance, _ = self.nearest(x, y)
        inner = self.line_width / 2 - spot_radius
        outer = self.line_width / 2 + spot_radius
        if distance <= inner:
            return 1.0
        if distance >= outer:
            return 0.0
        return (outer - distance) / (outer - inner)
