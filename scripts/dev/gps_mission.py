#!/usr/bin/env python3
"""GPS waypoint missions for flight_driver (mission wp_gps): rows "lat,lon,alt", WGS84 degrees, alt = height
above the take-off point [m].

  gps_mission.py check MISSION.csv [--home LAT,LON]   legs, bearings, total length, distance of each waypoint
                                                       from the take-off point (--home, default the first row)
  gps_mission.py from-local REF_LAT REF_LON LOCAL.csv  local rows "x,y,z" (north, east, height) around a
                                                       reference -> GPS rows (stdout)
  gps_mission.py to-local REF_LAT REF_LON MISSION.csv  GPS rows -> local north, east, height around the reference

Projection = PX4's MapProjection (src/lib/geo/geo.cpp, azimuthal equidistant, R 6371000 m) - the one
flight_driver uses with PX4's own reference, so to-local about PX4's ref_lat/ref_lon gives the local
coordinates flight_driver will fly."""
import argparse
import math
import sys

R = 6371000.0


def project(ref_lat, ref_lon, lat, lon):
    rl, rlon, la, lo = map(math.radians, (ref_lat, ref_lon, lat, lon))
    cos_d_lon = math.cos(lo - rlon)
    arg = max(-1.0, min(1.0, math.sin(rl) * math.sin(la) + math.cos(rl) * math.cos(la) * cos_d_lon))
    c = math.acos(arg)
    k = c / math.sin(c) if abs(c) > 0.0 else 1.0
    return (k * (math.cos(rl) * math.sin(la) - math.sin(rl) * math.cos(la) * cos_d_lon) * R,
            k * math.cos(la) * math.sin(lo - rlon) * R)


def reproject(ref_lat, ref_lon, x, y):
    rl, rlon = math.radians(ref_lat), math.radians(ref_lon)
    xr, yr = x / R, y / R
    c = math.hypot(xr, yr)
    if c == 0.0:
        return ref_lat, ref_lon
    sc, cc = math.sin(c), math.cos(c)
    lat = math.asin(cc * math.sin(rl) + xr * sc * math.cos(rl) / c)
    lon = rlon + math.atan2(yr * sc, c * math.cos(rl) * cc - xr * math.sin(rl) * sc)
    return math.degrees(lat), math.degrees(lon)


def rows(path):
    out = []
    for line in open(path):
        line = line.split('#')[0].strip()
        try:
            v = [float(t) for t in line.split(',')]
        except ValueError:
            continue                                  # header
        if len(v) >= 3:
            out.append(v[:3])
    return out


ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
sub = ap.add_subparsers(dest='cmd', required=True)
c = sub.add_parser('check')
c.add_argument('csv')
c.add_argument('--home', default=None)
f = sub.add_parser('from-local')
f.add_argument('ref_lat', type=float)
f.add_argument('ref_lon', type=float)
f.add_argument('csv')
t = sub.add_parser('to-local')
t.add_argument('ref_lat', type=float)
t.add_argument('ref_lon', type=float)
t.add_argument('csv')
a = ap.parse_args()

if a.cmd == 'from-local':
    print('lat,lon,alt')
    for x, y, z in rows(a.csv):
        lat, lon = reproject(a.ref_lat, a.ref_lon, x, y)
        print(f'{lat:.8f},{lon:.8f},{z:.2f}')
    sys.exit(0)

w = rows(a.csv)
if not w:
    sys.exit(f'no lat,lon,alt rows in {a.csv}')
bad = [r for r in w if abs(r[0]) > 90 or abs(r[1]) > 180]
if bad:
    sys.exit(f'not lat,lon,alt: {bad[0]}')
if a.cmd == 'to-local':
    print('x,y,z')
    for lat, lon, alt in w:
        n, e = project(a.ref_lat, a.ref_lon, lat, lon)
        print(f'{n:.3f},{e:.3f},{alt:.2f}')
    sys.exit(0)

home = tuple(map(float, a.home.split(','))) if a.home else tuple(w[0][:2])
print(f'take-off point {home[0]:.7f}, {home[1]:.7f}; {len(w)} waypoints')
total = 0.0
prev = None
for i, (lat, lon, alt) in enumerate(w):
    n, e = project(home[0], home[1], lat, lon)
    line = f'  wp {i}: N {n:+8.1f}  E {e:+8.1f}  height {alt:5.1f} m  ({math.hypot(n, e):6.1f} m from the take-off point)'
    if prev is not None:
        leg = math.hypot(n - prev[0], e - prev[1])
        brg = math.degrees(math.atan2(e - prev[1], n - prev[0])) % 360.0
        total += leg
        line += f'  leg {leg:6.1f} m bearing {brg:5.1f} deg'
    print(line)
    if alt < 2.0:
        print(f'    !! height {alt:.1f} m above the take-off point - flight_driver refuses < min_wp_alt_m (1.0)')
    prev = (n, e)
far = max(math.hypot(*project(home[0], home[1], la, lo)) for la, lo, _ in w)
print(f'path {total:.1f} m; farthest {far:.1f} m (flight_driver: refused > 300 m, fence {far + 20:.0f} m)')
