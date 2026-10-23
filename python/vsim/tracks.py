"""Track templates (design doc §14.3, R-20) and helpers for *.track.json files.

Regenerate the shipped templates with

    python -m vsim.tracks templates/tracks

Each generator returns a track dictionary in the *.track.json format
(schema_version 1): control points of the centerline with boundary widths,
start/finish and sector markers at control points, and driving lines as
lateral offsets n [m, positive left] at u = s / length.
"""

import json
import math
import sys
from pathlib import Path

SCHEMA_VERSION = 1


def _point(x, y, w_left, w_right=None):
    return {"x": round(x, 4), "y": round(y, 4), "z": 0.0, "w_left": w_left,
            "w_right": w_left if w_right is None else w_right, "bank_deg": 0.0}


def _track(name, points, *, closed, start_point=0, finish_point=-1, sector_points=(), lines=()):
    return {"schema_version": SCHEMA_VERSION, "name": name, "closed": closed, "points": points,
            "start_point": start_point, "finish_point": finish_point,
            "sector_points": list(sector_points), "lines": list(lines)}


def _turtle(segments, start=(0.0, 0.0, 0.0), spacing=6.0):
    """Points every ~`spacing` m along straights ("S", length) and arcs
    ("A", radius, degrees; positive turns left). Returns (points, x, y, heading)."""
    x, y, h = start
    points = [(x, y)]
    for segment in segments:
        if segment[0] == "S":
            length = segment[1]
            steps = max(1, round(length / spacing))
            for _ in range(steps):
                x += length / steps * math.cos(h)
                y += length / steps * math.sin(h)
                points.append((x, y))
        else:
            radius, angle = segment[1], math.radians(segment[2])
            steps = max(2, round(abs(angle) * radius / spacing))
            sign = 1.0 if angle > 0 else -1.0
            cx, cy = x - sign * radius * math.sin(h), y + sign * radius * math.cos(h)
            for k in range(1, steps + 1):
                hk = h + angle * k / steps
                points.append((cx + sign * radius * math.sin(hk), cy - sign * radius * math.cos(hk)))
            h += angle
            x, y = points[-1]
    return points, x, y, h


def acceleration(width=5.0):
    """FS acceleration: 75 m from the start line to the finish, then run-off."""
    half = width / 2
    points = [_point(x, 0.0, half) for x in (-5.0, 0.0, 25.0, 50.0, 75.0, 125.0, 175.0)]
    return _track("acceleration 75 m", points, closed=False, start_point=1, finish_point=4)


def skidpad(width=3.0, inner_diameter=15.25, spacing=4.0):
    """FS skidpad: two laps clockwise on the right circle, two counter-clockwise
    on the left one. Sectors split the four laps; the timed laps are the
    second on each side (sectors 2 and 4)."""
    half = width / 2
    radius = inner_diameter / 2 + half
    lap = 2 * math.pi * radius
    steps = max(8, round(lap / spacing))
    points = [_point(x, 0.0, half) for x in (-20.0, -10.0)]
    start = len(points)
    points.append(_point(0.0, 0.0, half))
    markers = []
    for side, sign in (("right", -1.0), ("left", 1.0)):
        for _ in range(2):
            for k in range(1, steps + 1):
                a = 2 * math.pi * k / steps
                # Circle through the origin, centre at (0, sign * radius).
                points.append(_point(radius * math.sin(a), sign * radius * (1 - math.cos(a)), half))
            markers.append(len(points) - 1)
    finish = markers.pop()
    points += [_point(x, 0.0, half) for x in (10.0, 20.0)]
    return _track("skidpad", points, closed=False, start_point=start, finish_point=finish, sector_points=markers)


def circle(radius=15.0, width=4.0, count=24):
    """Constant-radius pad for steady-state cornering (ISO 4138 style)."""
    half = width / 2
    points = []
    for k in range(count):
        a = 2 * math.pi * k / count - math.pi / 2
        points.append(_point(radius * math.cos(a), radius * (1 + math.sin(a)), half))
    return _track(f"circle r{radius:g}", points, closed=True)


def slalom(pitch=15.0, amplitude=1.5, cones=8, width=3.0):
    """Slalom between cones every `pitch` m: the centerline weaves."""
    half = width / 2
    points = [_point(-20.0, 0.0, half), _point(-pitch / 2, 0.0, half)]
    for k in range(cones * 2 + 1):
        x = k * pitch / 2
        points.append(_point(x, amplitude * math.sin(math.pi * k / 2) if k % 2 else 0.0, half))
    end = points[-1]["x"]
    points += [_point(end + pitch / 2, 0.0, half), _point(end + 30.0, 0.0, half)]
    return _track("slalom", points, closed=False, start_point=1, finish_point=len(points) - 2)


def double_lane_change(car_width=1.4, offset=3.5):
    """Double lane change after ISO 3888-1 (approximate): lane widths from the
    car width, lateral offset `offset` m."""
    w1 = (1.1 * car_width + 0.25) / 2
    w3 = (car_width + 1.0) / 2
    w5 = (1.3 * car_width + 0.25) / 2
    points = [_point(-20.0, 0.0, w1 * 2), _point(0.0, 0.0, w1), _point(7.5, 0.0, w1), _point(15.0, 0.0, w1)]

    def blend(x0, x1, y0, y1, wa, wb):
        out = []
        for k in range(1, 6):
            f = k / 6
            g = 0.5 - 0.5 * math.cos(math.pi * f)
            out.append(_point(x0 + f * (x1 - x0), y0 + g * (y1 - y0), max(wa, wb) * 2))
        return out

    points += blend(15.0, 45.0, 0.0, offset, w1, w3)
    points += [_point(45.0, offset, w3), _point(57.5, offset, w3), _point(70.0, offset, w3)]
    points += blend(70.0, 95.0, offset, 0.0, w3, w5)
    points += [_point(95.0, 0.0, w5), _point(102.5, 0.0, w5), _point(110.0, 0.0, w5), _point(140.0, 0.0, w5 * 2)]
    finish = len(points) - 2
    return _track("double lane change", points, closed=False, start_point=1, finish_point=finish)


def autocross(width=3.5):
    """A compact formula student style handling course (~600 m): sweeper,
    kink, hairpin, slalom section and a long straight."""
    half = width / 2
    segments = [("S", None), ("A", 25, 90), ("S", 10), ("A", 10, -45), ("A", 10, 90), ("S", 15),
                ("A", 7, 135), ("S", 12), ("A", 12, -90), ("A", 15, 30), ("A", 15, -60), ("A", 15, 60),
                ("A", 15, -30), ("S", 30), ("A", 18, 90), ("S", None), ("A", 12, 90)]
    # Solve the two open straights (headings 0 and 270 deg) so the loop closes.
    first, second = 0, 15

    def end_point(a, b):
        segs = list(segments)
        segs[first] = ("S", a)
        segs[second] = ("S", b)
        _, x, y, _ = _turtle(segs, spacing=1e9)
        return x, y

    x0, y0 = end_point(0.0, 0.0)
    # Straight `first` runs along +x, straight `second` along -y.
    a, b = -x0, y0
    if a <= 0 or b <= 0:
        raise ValueError(f"autocross layout does not close with positive straights ({a:.1f}, {b:.1f})")
    segs = list(segments)
    segs[first] = ("S", a)
    segs[second] = ("S", b)
    pts, _, _, _ = _turtle(segs, spacing=7.0)
    pts = pts[:-1]  # the last point closes onto the first
    points = [_point(x, y, half) for x, y in pts]
    count = len(points)
    return _track("autocross example", points, closed=True, start_point=1,
                  sector_points=[count // 3, 2 * count // 3])


def hill_circuit(width=8.0):
    """A small club circuit (~700 m) to show the 3D road (M5): 4 m of
    elevation change, banked sweepers, kerbs at the apexes, an ISO 8608
    class A surface and a wet patch on the back straight."""
    half = width / 2
    segments = [("S", None), ("A", 30, 90), ("S", None), ("A", 15, -60), ("A", 18, 60), ("S", 30),
                ("A", 25, 90), ("S", 60), ("A", 20, 90), ("S", 80), ("A", 35, 90)]
    first, second = 0, 2

    def turtle(a, b, spacing):
        segs = list(segments)
        segs[first] = ("S", a)
        segs[second] = ("S", b)
        return _turtle_tagged(segs, spacing)

    _, x0, y0 = turtle(0.0, 0.0, 1e9)
    a, b = -x0, -y0  # first straight along +x, second along +y
    if a <= 0 or b <= 0:
        raise ValueError(f"hill circuit layout does not close ({a:.1f}, {b:.1f})")
    tagged, _, _ = turtle(a, b, 8.0)
    tagged = tagged[:-1]  # the last point closes onto the first
    # Chord length for the elevation profile.
    s = [0.0]
    for (p, _), (q, _) in zip(tagged, tagged[1:]):
        s.append(s[-1] + math.hypot(q[0] - p[0], q[1] - p[1]))
    length = s[-1] + math.hypot(tagged[0][0][0] - tagged[-1][0][0], tagged[0][0][1] - tagged[-1][0][1])
    points = []
    for (xy, (kind, turn)), sk in zip(tagged, s):
        p = _point(xy[0], xy[1], half)
        u = 2 * math.pi * sk / length
        p["z"] = round(2.0 * math.sin(u) + 1.0 * math.sin(2 * u + 1.0) - 0.5 * math.sin(3 * u), 3)
        # Bank the long sweepers towards their inside (left turns: right edge up).
        if kind == "A" and abs(turn[1]) >= 90 and turn[0] >= 25:
            p["bank_deg"] = -5.0 if turn[1] > 0 else 5.0
        points.append(p)
    # Kerbs on the inside of every corner, a wet patch on the long straight.
    kerbs, patches = [], []
    i = 0
    while i < len(tagged):
        kind, turn = tagged[i][1]
        if kind != "A":
            i += 1
            continue
        j = i
        while j + 1 < len(tagged) and tagged[j + 1][1] == (kind, turn):
            j += 1
        if j > i:
            kerbs.append({"from_point": i, "to_point": j, "side": "left" if turn[1] > 0 else "right",
                          "width": 1.0, "height": 0.025})
        i = j + 1
    straight = [k for k, (_, tag) in enumerate(tagged) if tag == ("S", 80)]
    if len(straight) > 3:
        patches.append({"from_point": straight[1], "to_point": straight[-2], "n_min": -half, "n_max": 1.0, "mu": 0.6})
    track = _track("hill circuit", points, closed=True, start_point=1,
                   sector_points=[len(points) // 3, 2 * len(points) // 3])
    track["surface"] = {"mu": 1.0, "roughness": "A", "grass_mu": 0.6, "grass_roughness": "C"}
    track["kerbs"] = kerbs
    track["patches"] = patches
    return track


def _turtle_tagged(segments, spacing):
    """_turtle, with each point tagged by the segment that ends at it."""
    points, tags = [], []
    x = y = h = 0.0
    points.append((x, y))
    tags.append(("S", segments[0][1]))
    for segment in segments:
        pts, x, y, h = _turtle([segment], start=(x, y, h), spacing=spacing)
        for p in pts[1:]:
            points.append(p)
            tags.append((segment[0], segment[1:]) if segment[0] == "A" else ("S", segment[1]))
    return list(zip(points, tags)), x, y


def _signed_curvature(a, b, c):
    """Curvature of the circle through three points, positive turning left."""
    abx, aby = b[0] - a[0], b[1] - a[1]
    bcx, bcy = c[0] - b[0], c[1] - b[1]
    cross = abx * bcy - aby * bcx
    denom = math.hypot(abx, aby) * math.hypot(bcx, bcy) * math.hypot(c[0] - a[0], c[1] - a[1])
    return 2.0 * cross / denom if denom > 1e-12 else 0.0


def minimum_curvature_offsets(centerline, *, margin=1.0, spacing=2.0, iterations=300, closed=True):
    """Racing line of (approximately) minimum curvature inside the track.

    `centerline` is a dict as returned by vsim.Track.centerline(). Points every
    `spacing` m move along the centerline normal so that each point's
    curvature approaches the mean of its neighbours' (the K1999 path
    optimiser from TORCS), staying `margin` m inside the boundaries. Returns
    offsets [(u, n), ...] for a *.track.json line."""
    s, xs, ys, hs = centerline["s"], centerline["x"], centerline["y"], centerline["heading"]
    wl, wr = centerline["w_left"], centerline["w_right"]
    length = float(s[-1] + (s[1] - s[0])) if closed else float(s[-1])
    step = max(1, round(spacing / float(s[1] - s[0])))
    idx = list(range(0, len(s), step))
    cx = [float(xs[i]) for i in idx]
    cy = [float(ys[i]) for i in idx]
    nx = [-math.sin(float(hs[i])) for i in idx]
    ny = [math.cos(float(hs[i])) for i in idx]
    lo = [min(0.0, -float(wr[i]) + margin) for i in idx]
    hi = [max(0.0, float(wl[i]) - margin) for i in idx]
    count = len(idx)
    n = [0.0] * count

    def point(k, offset=None):
        k = k % count if closed else min(max(k, 0), count - 1)
        o = n[k] if offset is None else offset
        return cx[k] + o * nx[k], cy[k] + o * ny[k]

    def curvature(k, offset=None):
        return _signed_curvature(point(k - 1), point(k, offset), point(k + 1))

    span = range(count) if closed else range(2, count - 2)
    for _ in range(iterations):
        for k in span:
            target = 0.5 * (curvature(k - 1) + curvature(k + 1))
            now = curvature(k)
            slope = (curvature(k, n[k] + 1e-3) - now) / 1e-3
            if abs(slope) > 1e-9:
                n[k] = min(max(n[k] + 0.8 * (target - now) / slope, lo[k]), hi[k])
    every = max(1, round(5.0 / spacing))
    return [(round(float(s[idx[k]]) / length, 6), round(n[k], 3)) for k in range(0, count, every)]


TEMPLATES = {
    "acceleration": acceleration,
    "skidpad": skidpad,
    "circle_r15": circle,
    "slalom": slalom,
    "double_lane_change": double_lane_change,
    "autocross": autocross,
    "hill_circuit": hill_circuit,
}


def write(track, path):
    Path(path).write_text(json.dumps(track, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")


def main(argv):
    if len(argv) != 1:
        print(__doc__)
        return 1
    out = Path(argv[0])
    out.mkdir(parents=True, exist_ok=True)
    from . import Track  # needs the built extension

    for name, generator in TEMPLATES.items():
        track = generator()
        if name in ("autocross", "hill_circuit"):
            geometry = Track.from_json(json.dumps(track))
            track["lines"] = [{"name": "racing",
                               "offsets": minimum_curvature_offsets(geometry.centerline())}]
        path = out / f"{name}.track.json"
        write(track, path)
        geometry = Track(str(path))
        notes = "; ".join(geometry.warnings()) or "ok"
        print(f"{path}: {geometry.length:.1f} m, {len(track['points'])} points - {notes}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
