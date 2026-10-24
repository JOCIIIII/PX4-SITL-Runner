# Flight profiles

One file per airframe and speed. **The PILS gate and the vehicle source the same file**, so what was checked in PILS is
what flies. Callers:

- PILS gate: `scripts/dev/pils_gate.sh PROFILE NAME`
- The vehicle: `scripts/dev/onboard_real.sh fly PROFILE NAME`

Format: `KEY=value` lines that bash can source. The keys are the `NOCLIMB_*` knobs of `scripts/ros2/apps/noclimb_ca.sh`,
plus `PROFILE_AIRFRAME` (`x500` or `x8`).

Lines marked `MEASURE` are set on the vehicle in the morning; the values here are the ones in use until then:

| Key | What it is | How it is measured |
|---|---|---|
| `NOCLIMB_HOVER_THRUST` (X500) | normalised hover thrust | `hover_thrust_estimate` in the first hover's log (`scripts/dev/hover_thrust.py`) |
| `NOCLIMB_LIDAR_YAW_DEG` / `_ROLL_DEG` / `_PITCH_DEG` / `_Z` | lidar mount | `scripts/dev/lidar_mount_calib.py` |
| `NOCLIMB_LIDAR_LATENCY` | lidar frame end to receipt | `scripts/dev/lidar_mount_calib.py latency` |
| `NOCLIMB_SELF_RADIUS` | radius of the vehicle's own returns | `scripts/dev/livox_bench_check.py` (returns by azimuth) |

| Profile | Bundle | SILS | PILS (Jetson, over 5G, +~40 ms loop) |
|---|---|---|---|
| `x500_3` | `x500_speed_onnx/v3` (`ca_x500_v3`) | +1.04 m | +0.93 m |
| `x500_5` | `x500_onnx` (`ca_x500`) | +0.95 m | **contact −0.18 m (wood wall)**, then +1.02 m |
| `x8_3` | `x8_speed_onnx/v3` (`ca_x8_v3`) | +0.86 m | +0.82 m |
| `x8_5` | `x8_30m_soft_onnx` (`ca_x8_30m_soft`) | +1.44 m | +1.60 m |
| `x8_3n` | `x8_smooth_early_onnx/v3` (`ca_x8_v3`, early avoidance) - the first X8 CA flight | +1.49..+1.87 m (3 runs) | see docs/flight-test-2026-10-17.md |
| `x8_5s` | `x8_30m_soft_onnx` + re-acceleration cap 1.5 m/s² (`NOCLIMB_CMD_ACCEL_MAX`) - the least shaking X8 setup | +0.52..+1.19 m (6 runs) | +0.70 m (orin-nano-02, LAN) |

Closest prop-tip clearance on C-Track WP3 → GOAL at 1.7 m, flown 2026-10-16 with `flight_driver` and these files
(one run each, two for `x500_5` PILS). X500 5 m/s: the vehicle follows the commanded speed 0.7–0.8 s late and
brakes late at broad walls - fly 3 m/s twice cleanly first, keep broad obstacles out of 5 m/s runs
(`docs/real-flight-2026-10-16.md` §5).
