#!/usr/bin/env python3
"""PathFollowing test paths for flight_driver (mission wp_local / wp_gps): line, square, circle, spiral, starting
at the take-off point and sized for the cruise speed (the X8: corner / turn speed of PathFollowing's corner limit,
lateral acceleration <= ~4 m/s^2, accel / brake ~2 m/s^2).

  pf_shapes.py SHAPE SPEED [--heading DEG] [--alt H] [--size S] [--laps N] [--turn right|left]
               [--ref LAT,LON] [--out CSV]
    SHAPE    line | square | circle | spiral
    SPEED    cruise speed [m/s] (MISSION_SPEED); sets the default size (table below)
    --heading compass bearing [deg] of the line / the square's first side / the direction from the take-off
             point to the circle's and spiral's centre (default 0 = north): turn the shape into the free part of
             the field
    --alt    height above the take-off point [m] (default 5)
    --size   line length / square side / circle radius / spiral outer radius [m] (default by speed)
    --laps   circle laps / spiral turns (default 2)
    --turn   right (clockwise seen from above, default) | left
    --ref    the take-off point's LAT,LON: write wp_gps rows "lat,lon,alt" instead of local "north,east,height"
             (PX4's projection, scripts/dev/gps_mission.py)
  pf_shapes.py table        the default sizes and the space each shape needs, per speed

Every path starts at the take-off point (flight_driver flies there after the take-off) and the line ends
--size away from it, the others back at it. Lines: accelerate, ~4 s at speed, brake (PathFollowing slows down
for the last waypoint). Circles / spirals: polylines at <= 5 deg per vertex (PathFollowing's corner limit
ignores turns under 10 deg, so a circle is flown at the cruise speed).
"""
import argparse
import math
import sys

R_EARTH = 6371000.0


def reproject(ref_lat, ref_lon, x, y):
    """local north x, east y [m] about the reference -> lat, lon (PX4's MapProjection, as gps_mission.py)"""
    rl, rlon = math.radians(ref_lat), math.radians(ref_lon)
    xr, yr = x / R_EARTH, y / R_EARTH
    c = math.hypot(xr, yr)
    if c == 0.0:
        return ref_lat, ref_lon
    sc, cc = math.sin(c), math.cos(c)
    lat = math.asin(cc * math.sin(rl) + xr * sc * math.cos(rl) / c)
    lon = rlon + math.atan2(yr * sc, c * math.cos(rl) * cc - xr * math.sin(rl) * sc)
    return math.degrees(lat), math.degrees(lon)

SPEEDS = (1, 3, 5, 7, 10)


def default_size(shape, v):
    if shape == 'line':                   # accel + brake v^2/(2*2) each, ~4 s at speed, 10 m spare
        return math.ceil((v * v / 2.0 + 4.0 * v + 10.0) / 5.0) * 5.0
    if shape == 'square':                 # corners at ~4.5 m/s (90 deg, 1.5 m corner error): room to get to v
        return math.ceil((v * v / 4.0 + 2.0 * v + 10.0) / 5.0) * 5.0
    if shape == 'circle':                 # v^2 / R <= 4 m/s^2 (~22 deg tilt), at least 5 m
        return float(max(5, math.ceil(v * v / 4.0)))
    if shape == 'spiral':                 # inner radius as the circle's, then 2 turns 4 m apart
        return default_size('circle', v) + 8.0
    raise ValueError(shape)


def shape_points(shape, size, laps, turn, spiral_inner=None):
    """local (forward, right) points of the shape, starting at (0, 0), forward = --heading"""
    s = 1.0 if turn == 'right' else -1.0
    if shape == 'line':
        return [(0.0, 0.0), (size, 0.0)]
    if shape == 'square':
        return [(0.0, 0.0), (size, 0.0), (size, s * size), (0.0, s * size), (0.0, 0.0)]
    if shape == 'circle':
        # centre straight ahead at `size`; the take-off point is on the circle (bearing pi from the centre);
        # theta = bearing about the centre, clockwise seen from above = increasing (right turn)
        n = int(round(72 * laps))
        return [(size + size * math.cos(math.pi + s * 2.0 * math.pi * k / 72.0),
                 size * math.sin(math.pi + s * 2.0 * math.pi * k / 72.0)) for k in range(n + 1)]
    if shape == 'spiral':
        # outward from the inner radius r0 (centre r0 ahead, the take-off point on the inner circle) to `size`
        r0 = spiral_inner if spiral_inner is not None else max(3.0, size - 8.0)
        n = int(round(72 * laps))
        pts = []
        for k in range(n + 1):
            f = k / n
            r = r0 + (size - r0) * f
            th = math.pi + s * 2.0 * math.pi * laps * f
            pts.append((r0 + r * math.cos(th), r * math.sin(th)))
        return pts
    raise ValueError(shape)


def to_ne(pts, heading_deg):
    h = math.radians(heading_deg)
    fn, fe = math.cos(h), math.sin(h)              # forward
    rn, re_ = -math.sin(h), math.cos(h)            # right
    return [(f * fn + r * rn, f * fe + r * re_) for f, r in pts]


def length(ne):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(ne, ne[1:]))


def table():
    print('speed  line L   square side   circle R (diam)   spiral R in->out   | est. time line/square/circle x2/spiral x2 [s]')
    for v in SPEEDS:
        L, S, C, P = (default_size(s, v) for s in ('line', 'square', 'circle', 'spiral'))
        lens = [length(shape_points(s, z, 1 if s in ('line', 'square') else 2, 'right', None))
                for s, z in (('line', L), ('square', S), ('circle', C), ('spiral', P))]
        t = [x / v + 2.0 * v / 2.0 for x in lens]
        print(f'{v:4d}   {L:5.0f} m   {S:6.0f} m      {C:4.0f} m ({2 * C:3.0f} m)     {max(3.0, P - 8):3.0f} -> {P:3.0f} m'
              f'        | {t[0]:4.0f} / {t[1]:4.0f} / {t[2]:4.0f} / {t[3]:4.0f}')


def main():
    if len(sys.argv) > 1 and sys.argv[1] == 'table':
        table()
        return
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('shape', choices=('line', 'square', 'circle', 'spiral'))
    ap.add_argument('speed', type=float)
    ap.add_argument('--heading', type=float, default=0.0)
    ap.add_argument('--alt', type=float, default=5.0)
    ap.add_argument('--size', type=float, default=None)
    ap.add_argument('--laps', type=float, default=None)
    ap.add_argument('--turn', choices=('right', 'left'), default='right')
    ap.add_argument('--ref', default=None)
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    size = a.size if a.size else default_size(a.shape, a.speed)
    laps = a.laps if a.laps else (1.0 if a.shape == 'line' or a.shape == 'square' else 2.0)
    inner = None
    if a.shape == 'spiral':
        inner = max(3.0, default_size('circle', a.speed)) if not a.size else max(3.0, size - 8.0)
    ne = to_ne(shape_points(a.shape, size, laps, a.turn, inner), a.heading)
    rows = []
    if a.ref:
        lat0, lon0 = (float(x) for x in a.ref.split(','))
        rows = [f'{lat:.8f}, {lon:.8f}, {a.alt:.1f}' for lat, lon in (reproject(lat0, lon0, n, e) for n, e in ne)]
        head = 'lat,lon,alt\n# WGS84 deg, height above the take-off point [m]'
    else:
        rows = [f'{n:.2f}, {e:.2f}, {a.alt:.1f}' for n, e in ne]
        head = 'x,y,z\n# north, east, height above the take-off point [m]'
    text = head + f'\n# pf_shapes.py {a.shape} {a.speed:g} m/s, size {size:g} m, heading {a.heading:g} deg, ' \
                  f'{a.turn}, laps {laps:g}\n' + '\n'.join(rows) + '\n'
    ns, es = [p[0] for p in ne], [p[1] for p in ne]
    info = (f'{a.shape} {a.speed:g} m/s: size {size:g} m, {len(ne)} waypoints, path {length(ne):.0f} m, '
            f'extent N {min(ns):+.0f}..{max(ns):+.0f} E {min(es):+.0f}..{max(es):+.0f} m from the take-off point, '
            f'farthest {max(math.hypot(n, e) for n, e in ne):.0f} m')
    if a.out:
        open(a.out, 'w').write(text)
        print(info, '->', a.out, file=sys.stderr)
    else:
        sys.stdout.write(text)
        print(info, file=sys.stderr)


if __name__ == '__main__':
    main()
