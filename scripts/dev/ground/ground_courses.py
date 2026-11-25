#!/usr/bin/env python3
"""PathFollowing test courses for the UGV / USV (ground_driver.py --frame start: rows "north,east" [m] relative
to the vehicle, which stands at the origin facing --heading unless --start-offset moves the path away).
  ground_courses.py SHAPE --size S [--spacing D] [--heading DEG] [--laps N] [--corner R] [--turn right|left]
                    [--start-offset LATERAL,ANGLE] --out CSV
    line     straight, S long
    circle   radius S, --laps turns (default 2); the vehicle starts on the circle, tangent to it
    square   side S; corners rounded with radius --corner (0 = sharp 90 deg)
    slalom   S long, a sine of amplitude --corner (default S/10) and wavelength S/2
    eight    figure of eight, two circles of radius S
  --spacing  waypoint spacing [m]: PathFollowing flies MPPI only while the waypoint it heads for is not the
             last one, so every course is a dense polyline (default S/40)
  --start-offset LATERAL,ANGLE  the path starts LATERAL m to the right of the vehicle and is turned ANGLE deg
             away from its heading - a start that is not aligned with the path"""
import argparse
import math


def densify(pts, d):
    out = [pts[0]]
    for a, b in zip(pts, pts[1:]):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(int(math.ceil(L / d)), 1)
        for i in range(1, n + 1):
            out.append((a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n))
    return out


def arc(c, r, a0, a1, d):
    n = max(int(math.ceil(abs(a1 - a0) * r / d)), 2)
    return [(c[0] + r * math.cos(a0 + (a1 - a0) * i / n), c[1] + r * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]


def build(a):
    S, d = a.size, a.spacing or a.size / 40.0
    sgn = 1.0 if a.turn == 'right' else -1.0
    # forward = +x, right = +y (then rotated to the heading: x -> north, y -> east at heading 0)
    if a.shape == 'line':
        pts = densify([(0.0, 0.0), (S, 0.0)], d)
    elif a.shape == 'circle':
        laps = a.laps or 2.0
        c = (0.0, sgn * S)
        pts = arc(c, S, -sgn * math.pi / 2, -sgn * math.pi / 2 + sgn * 2 * math.pi * laps, d)
    elif a.shape == 'eight':
        pts = arc((0.0, S), S, -math.pi / 2, 1.5 * math.pi, d)[:-1] + arc((0.0, -S), S, math.pi / 2, -1.5 * math.pi, d)
    elif a.shape == 'square':
        r = a.corner or 0.0
        corners = [(0.0, 0.0), (S, 0.0), (S, sgn * S), (0.0, sgn * S), (0.0, 0.0)]
        if r <= 0.0:
            pts = densify(corners, d)
        else:
            pts = [(0.0, 0.0)]
            for i in (1, 2, 3):
                p, q, n = corners[i - 1], corners[i], corners[i + 1]
                u = ((q[0] - p[0]) / S, (q[1] - p[1]) / S)
                v = ((n[0] - q[0]) / S, (n[1] - q[1]) / S)
                a_in = (q[0] - u[0] * r, q[1] - u[1] * r)
                c = (a_in[0] + v[0] * r, a_in[1] + v[1] * r)
                a0 = math.atan2(a_in[1] - c[1], a_in[0] - c[0])
                pts += densify([pts[-1], a_in], d)[1:] + arc(c, r, a0, a0 + sgn * math.pi / 2, d)[1:]
            pts += densify([pts[-1], (0.0, 0.0)], d)[1:]
    elif a.shape == 'slalom':
        amp = a.corner or S / 10.0
        n = max(int(math.ceil(S / d)), 2)
        pts = [(S * i / n, sgn * amp * math.sin(2 * math.pi * (S * i / n) / (S / 2.0))) for i in range(n + 1)]
    lat, ang = (float(v) for v in a.start_offset.split(',')) if a.start_offset else (0.0, 0.0)
    h = math.radians(a.heading + ang)
    h0 = math.radians(a.heading)
    off = (-lat * math.sin(h0), lat * math.cos(h0))           # LATERAL m to the right of the vehicle
    return [(off[0] + x * math.cos(h) - y * math.sin(h), off[1] + x * math.sin(h) + y * math.cos(h)) for x, y in pts]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('shape', choices=('line', 'circle', 'square', 'slalom', 'eight'))
    ap.add_argument('--size', type=float, required=True)
    ap.add_argument('--spacing', type=float, default=None)
    ap.add_argument('--heading', type=float, default=0.0)
    ap.add_argument('--laps', type=float, default=None)
    ap.add_argument('--corner', type=float, default=None)
    ap.add_argument('--turn', choices=('right', 'left'), default='right')
    ap.add_argument('--start-offset', default='')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    pts = build(a)
    with open(a.out, 'w') as f:
        for n, e in pts:
            f.write(f'{n:.3f},{e:.3f}\n')
    L = sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(pts, pts[1:]))
    print(f'{a.shape}: {len(pts)} waypoints, {L:.1f} m -> {a.out}')


if __name__ == '__main__':
    main()
