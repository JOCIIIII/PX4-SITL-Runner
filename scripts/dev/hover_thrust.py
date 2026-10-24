#!/usr/bin/env python3
"""The vehicle's normalised hover thrust, for NOCLIMB_HOVER_THRUST / MISSION_HOVER_THRUST (both PathFollowing
nodes' thrust scale: thrust = mass * |acc| / (n * T_max), and only mass * g / (n * T_max) = the hover thrust
enters the command, the disturbance observer and the MPPI rollout).

  hover_thrust.py LOG.ulg                    PX4's own estimate (hover_thrust_estimate) while airborne
  hover_thrust.py pf_log.csv --assumed H     from a PathFollowing log flown with hover_thrust H: in steady flight
                                             the observer's vertical estimate ndo_z (NED) is g * (h_true / H - 1)
A mismatch shows in flight as ndo_z away from 0: the observer carries it (capped at 4 m/s^2 with the
horizontal part), so a 10 % error costs ~1 m/s^2 of the cap. The real X500 logs (10-02, before the lidar):
0.644-0.645."""
import argparse
import sys

import numpy as np

G = 9.81
ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('log')
ap.add_argument('--assumed', type=float, default=None, help='hover_thrust the PathFollowing log was flown with')
a = ap.parse_args()

if a.log.endswith('.ulg'):
    from pyulog import ULog
    u = ULog(a.log, ['hover_thrust_estimate', 'vehicle_land_detected'])
    d = {x.name: x.data for x in u.data_list}
    if 'hover_thrust_estimate' not in d:
        sys.exit('no hover_thrust_estimate in this log')
    h = d['hover_thrust_estimate']
    t = h['timestamp'] * 1e-6
    ok = h['valid'] > 0
    if 'vehicle_land_detected' in d:
        ld = d['vehicle_land_detected']
        landed = np.interp(t, ld['timestamp'] * 1e-6, ld['landed'].astype(float)) > 0.5
        ok &= ~landed
    if not ok.any():
        sys.exit('no valid airborne estimate')
    v = h['hover_thrust'][ok]
    tt = t[ok]
    late = tt > tt[0] + 5.0                          # the estimator converges in the first seconds
    v = v[late] if late.sum() > 10 else v
    print(f'{a.log}: hover thrust {np.median(v):.3f} (p10 {np.percentile(v, 10):.3f}, p90 {np.percentile(v, 90):.3f}, '
          f'{tt[-1] - tt[0]:.0f} s airborne)')
    print(f'-> NOCLIMB_HOVER_THRUST={np.median(v):.3f}')
else:
    if a.assumed is None:
        sys.exit('--assumed H: the hover_thrust this PathFollowing log was flown with')
    p = np.genfromtxt(a.log, delimiter=',', names=True)
    vz, nz = p['vel_z'], p['ndo_z']
    # PathFollowing flying the path (it keeps computing - unforwarded - through the landing and on the ground,
    # where the observer saturates): not done, on the path, level-ish, thrust not saturated. (Its time column
    # steps once a second, so no acceleration filter; the median over a flight is robust to the turns.)
    steady = ((np.abs(vz) < 0.3) & (p['thrust_norm_cmd'] > 0.05) & (p['thrust_norm_cmd'] < 0.98)
              & (p['pf_done'] < 0.5) & (np.abs(p['cross_track_error']) < 1.0))
    if steady.sum() < 50:
        sys.exit(f'only {steady.sum()} steady samples')
    d = np.median(nz[steady])
    h = a.assumed * (1.0 + d / G)
    print(f'{a.log}: steady vertical disturbance estimate {d:+.2f} m/s^2 over {steady.sum()} samples '
          f'(flown with hover_thrust {a.assumed:.3f})')
    print(f'-> NOCLIMB_HOVER_THRUST={h:.3f}')
