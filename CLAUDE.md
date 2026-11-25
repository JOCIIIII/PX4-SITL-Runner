# CLAUDE.md

Guidance for Claude Code on the `ugv-usv` branch of PX4-SITL-Runner (the A4VAI subproject-2 SILS / PILS
simulator). Answer the user in Korean unless they write otherwise. On `main` this file is local-only
(`.gitignore`); on this branch it is tracked and covers only the UGV / USV work.

## What this branch adds

`main` (`inhousesim-v1.0`, e26b708) + `scripts/dev/ground/`: a Gazebo Classic SITL harness that runs the A4VAI
path following (PathFollowing: speed-regulated pursuit = guid 0, MPPI = guid 2) and path planning (Hybrid PP
0.3.2) on a ground vehicle (UGV) and a surface vessel (USV). Collision avoidance is not ported - the learned
policy needs retraining (its observation is a 3D lidar at flight height, its action a 3D velocity). Everything
else (drone SILS / PILS / real vehicle, `scripts/scenario.sh`) is as on `main`: `README.md`, `docs/MANUAL.md`.

| Piece | Where |
|---|---|
| Harness (the only code on this branch) | `scripts/dev/ground/` |
| PathFollowing ground mode | branch `ugv-usv` of `JOCIIIII/A4VAI-PathFollowing` (87d41a6, on top of `inhousesim-v1.0`) |
| Runtime data, not in git | `~/Documents/A4VAI-SITL/ground_sim/`: `etc/` `models/` `worlds/` `courses/` `pp/` `ws/` `runs/` |
| UGV | PX4 `r1_rover` (differential drive, 0.50 x 0.45 m), airframe `2010_gazebo-classic_a4_ugv` |
| USV | PX4 `boat` (two thrusters, 4.9 x 2.5 m), airframe `2011_gazebo-classic_a4_usv` |

Both vehicles run PX4 v1.16 `rover_differential` (the boat's thrusters act as left / right wheels). The shared
PX4 tree, `ros2_ws` and the compose stack are only read: PX4 runs the installed build with a copied etc dir, and
the PathFollowing branch is a second git worktree of the installed submodule, built as an overlay on
`ros2_ws/install`.

Data flow: planner (before the run, from the layout map) → waypoints → `ground_driver.py` → PathFollowing
(+ node_mppi) → `/pf_ground_cmd` [speed, bearing, ...] → `ground_driver.py` → PX4 offboard velocity setpoint →
rover_differential (speed PID, bearing → yaw rate → left / right motors) → Gazebo → EKF2 → PathFollowing.
`ground_driver.py` plays the role `flight_manager` / `flight_driver` play for drones (arm, offboard, relay; it
holds when the command is older than 0.5 s).

## Running

Prerequisites: `./scripts/install.sh` done (PX4 build, ros2_ws build, images, GPU in docker); for path planning
once `scripts/dev/scenario_pp.sh --install WHEEL` (the lab's hybrid-learning-path-planning 0.3.2 wheel).

```bash
scripts/dev/ground/setup.sh all       # etc + airframes, model copies + fixes, worlds, PathFollowing worktree, build
scripts/dev/ground/make_courses.sh    # course CSVs, planned paths (ground_pp.sh), yard / harbour worlds
python3 scripts/dev/ground/ground_batch.py PREFIX --parallel 4 [--only SUBSTR] [--reps N]   # runs + summary
python3 scripts/dev/ground/ground_rtf.py ~/Documents/A4VAI-SITL/ground_sim/runs/PREFIX_*/  # 0.98-1.02 or invalid
```

`make_courses.sh` regenerates the batch inputs byte for byte (checked end to end on 2026-11-25); only the
planning times vary from run to run.

- One run: `ground_run.sh ugv|usv NAME -- /ground_scripts/ground_mission.sh /ground_courses/X.csv SPEED [GUID]
  [--timeout S]`. A planned-path run also needs `GROUND_WORLD`, `GROUND_SPAWN_POSE` and `--frame local --origin
  X,Y` (the layout's START); `start()` in `ground_batch.py` builds them.
- PX4 alone: `ground_run.sh ugv NAME -- python3 /ground_scripts/ground_step_test.py --steps "speed:dbearing:dur,..."`,
  then `step_summary.py` and `fit_ground_model.py VEHICLE STEP_CSV --drive-only` (rollout model constants).
- Per run: `ground_metrics.py RUN` (path error of the estimate and of the true position), `ground_clearance.py
  RUN LAYOUT ugv|usv` (true footprint vs obstacles), `ground_plot.py RUN [--layout ...]` (map).
- `ground_batch.py` runs up to 4 instances, each on its own docker network 172.(60+K).0.0/16 with ROS_DOMAIN_ID
  40+K, and skips runs that already have a result (re-run = resume). Never edit these scripts while a batch runs.
- **After every PathFollowing edit run `setup.sh build`.** The overlay install holds copies of the Python files
  and yaml even with `--symlink-install` (this image's setuptools); a stale install silently runs old code.

## Simulator fixes (why the stock classic models do not work as they are)

All in the model copies and the two airframes (`setup.sh models`, `airframes/`):
1. Lockstep off: with it the models stall once armed (rover_differential writes the motors on a 10 ms timer).
   PX4 (lockstep-scheduler build) then takes the simulator's clock while the ROS nodes run on wall time - check
   every batch with `ground_rtf.py`.
2. r1_rover: the stock 1061 motor mapping is left / right reversed for rover_differential (500 deg/s spin) - swapped.
3. r1_rover skid steering: effective `RD_WHEEL_TRACK` 1.4 m (geometric 0.3), else 1/3 of the commanded yaw rate.
4. boat: no ground-truth plugin, so the magnetometer model sits at latitude 0 (8 deg heading error, no arming) - added.
5. boat thrusters: no reverse thrust (a stop command accelerated it) and the wrong turning direction - `<reversible>`, ccw.
6. IMU turn-on bias set to 0 (tilt error at rest).
7. ODE contact correction leaks fake roll / pitch rates into the gyro in skid turns (EKF tilt 26 deg, GPS
   velocity rejected, 6.9 m position error): `worlds/ugv_flat.world` `contact_max_correcting_vel` 0.1,
   `contact_surface_layer` 0.002, wheel contact `max_vel` 0.01 / `min_depth` 0 → 0.19 m.
8. PX4 starts 6 s after the spawn (`GROUND_SPAWN_SETTLE_S`): aligning before the wheels touch gave 16 deg yaw error.

UGV EKF2: `EKF2_GYR_NOISE` 0.3, `EKF2_GPS_V_NOISE` 0.05, `EKF2_IMU_CTRL` 5 (no accel-bias learning).
PX4 interface: `TrajectorySetpoint.velocity` (NED) - norm = speed setpoint, direction = bearing (yaw setpoint);
no yaw-rate feed-forward. A zero vector means bearing north, so `hold()` sends 1 mm/s along the kept bearing.

## PathFollowing ground mode (branch `ugv-usv`)

- `vehicle_type` 3 (UGV) / 4 (USV) → `config/ugv.yaml` / `usv.yaml` (`ground:` section: output stage + rollout
  model) and `sim_ground_ugv.yaml` / `sim_ground_usv.yaml` (guidance).
- Guidance laws unchanged. Output stage: acceleration → speed + bearing (yaw rate = a_n / v, clamped; bearing =
  yaw + rate / `yaw_kp` within `bearing_lead_max_deg`; speed = forward speed + a_t · `tau_speed`).
- Virtual target always one lookahead ahead of the closest point (the multicopter rule made a rover weave);
  planar state; end-line latch; spot-turn alignment at the start; no disturbance observer.
- Velocity used: heading frame (forward speed along yaw) for pursuit and the UGV's MPPI; course frame (true
  velocity, sway included) for the USV's MPPI.
- MPPI: `core_mppi._ground_defines()` turns the yaml `ground:` section into `#define`s; `mppi_kernel.cu` under
  `GROUND_VEHICLE` replaces the quad dynamics with the fitted PX4 response (first-order speed and yaw rate,
  limits, spot-turn switch, speed cap while turning) plus a sway state for the boat (`sway_tau` 2.5 s).
  Without the define the preprocessed kernel equals the original: multicopters unchanged
  (`test_core_runtime.py` 13/13, `run_core_smoke.py` identical).

## Results (batch of 2026-11-24; run data stays out of git)

34/34 runs completed, no contact, real-time ratio 0.998-1.000. Path error = 95th percentile of the true position.

| | UGV 1.5 m/s | USV 2.5 m/s |
|---|---|---|
| Courses (UGV 6, USV 5), 1 run each | pursuit 0.02-0.27 m, MPPI 0.07-0.32 m | pursuit 0.01-1.02 m, MPPI 0.33-0.86 m |
| Planned path, 3 runs each | 0.27-0.37 m, clearance >= 0.83 m | pursuit 1.28-1.31 m, MPPI 0.90-0.92 m, clearance >= 1.76 m |
| Planning (Hybrid PP), 2 runs | 79-87 ms, 1.15 x the reference shortest path | 84-110 ms, 1.11 x |

Weak spots: the USV's MPPI weaves about 0.3 m on straights, is worse than pursuit on the slalom and passes the
breakwater tip 1.6 m closer than pursuit; the UGV's MPPI is no better than pursuit at 1.5 m/s; UGV spot turns
overshoot (161 deg/s against the 120 deg/s limit), which the rollout model does not reproduce.

## Limits and what was not run

- Flat ground, still water (no waves, current or wind), non-RTK GPS (0.3-0.5 m), ODE wheel contact.
- A real vehicle needs the PX4 parameters (`RO_*`, `RD_*`) and the rollout constants refitted from its own step
  responses (`fit_ground_model.py`).
- Not run: a drone SITL flight with the `ugv-usv` PathFollowing (only the unit tests and the kernel identity
  above); any hookup to `scripts/scenario.sh`, `flight_manager` or `flight_driver`; Ackermann UGVs and rudder USVs.
- The planner does not know the turning radius: its radius = body + tracking margin (UGV 0.65 m, USV 5.0 m).

## Conventions

- Commits: no Co-Authored-By / Claude-Session trailers in this project.
- Report results as measured (true-position error, clearance, completion) with the number of runs.
