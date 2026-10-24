# CLAUDE.md

Guidance for Claude Code in this repository (InhouseSim v1.0, branch `inhousesim-v1.0`). The user-facing guide is
`README.md` (Korean); answer the user in Korean unless they write otherwise.

## What this is

A Docker-based simulator for the A4VAI autonomous drone stack: PX4 v1.16 (SITL) + Gazebo Classic (physics) + an
Unreal Engine 5.8 test field with Cosys-AirSim 3.5 (lidar, rendering) + ROS2 Humble apps (learned collision
avoidance, PathFollowing MPPI, SUPER planner, a flight driver). The same apps run in three places:

| Mode | Where the ROS2 apps run | Entry |
|---|---|---|
| SILS | this PC (container `ros2-env`) | `./scripts/ctrack.sh fly PROFILE NAME` |
| PILS | a Jetson Orin (JetPack 6) over LAN/Tailscale; this PC runs only Unreal + Gazebo/PX4 | `ONBOARD_IP=.. ONBOARD_SSH=user@.. ./scripts/ctrack.sh pils PROFILE NAME` |
| Real vehicle | the Jetson on the vehicle, Pixhawk over serial, Livox Mid-360 | `scripts/dev/onboard_real.sh` (run on the Jetson) |

This repo holds only scripts, configs, airframe models and docs. Sources and binaries come from:
- `JOCIIIII/A4VAI-Algorithms-ROS2` branch `inhousesim-v1.0` (submodules `A4VAI-PathFollowing`, `super` on their
  `inhousesim-v1.0` branches, `px4_msgs` release/1.16) → `~/Documents/A4VAI-SITL/ROS2/ros2_ws/src`
- `JOCIIIII/A4VAI-Unreal` tag `inhousesim-v1.0` (sparse checkout) + its packaged Shipping build and Gazebo binaries,
  which are NOT on GitHub: the user copies `CTrackSim-Linux.tar.gz`, `ctrack-gazebo-prebuilt.tar.gz`, `SHA256SUMS`
  from the lab NAS (InhouseSim v1.0 folder) into `~/Downloads/inhousesim-v1.0` → `~/Documents/A4VAI-SITL/A4VAI-Unreal`
- PX4-Autopilot v1.16.0 + `A4VAI-Custom-PX4-File` → `~/Documents/A4VAI-SITL/PX4-Autopilot`

## First steps on a new machine

```bash
./scripts/setup-host.sh     # Docker CE + NVIDIA Container Toolkit (sudo; then log out/in for the docker group)
# the three C-Track package files from the lab NAS -> ~/Downloads/inhousesim-v1.0 (ask the user if missing)
./scripts/install.sh        # everything else, idempotent - re-run after any failure; log in ~/Documents/A4VAI-SITL/logs
./scripts/ctrack.sh fly x8_5s first_flight
```

Requirements: Ubuntu 22.04 x86_64, NVIDIA driver with Vulkan (the packaged Unreal needs no UE install), a graphical
session (DISPLAY), ~60 GB disk, python3 (host analysis tools also need `pip3 install --user numpy pyulog`).
`install.sh` overrides (env): `ROS2_SRC_REPO/BRANCH`, `UNREAL_REPO/REF`, `UNREAL_ASSET_DIR` (the folder with the three
package files, default `~/Downloads/inhousesim-v1.0`), `UNREAL_ASSET_URL` (an HTTP folder serving them instead).

## C-Track flights (`scripts/ctrack.sh` → `scripts/dev/pils_gate.sh` → `scripts/dev/ctrack_super.sh`)

- `ctrack.sh unreal [x8|x500] [CTrack|Yard]` starts the packaged Unreal with the airframe's lidar mount:
  x8 → `A4VAI-Unreal/settings/settings_gazebo_mid360.json`, x500 → `scripts/dev/unreal_settings/settings_gazebo_mid360_x500.json`.
  **The Unreal settings must match the flown airframe** - `fly`/`pils` restart Unreal when they differ (state file
  `$XDG_RUNTIME_DIR/ctrack_unreal.state`). AirSim API: 127.0.0.1:41451.
- `fly PROFILE NAME [FROM GOAL ALT SPEED TIMEOUT]`: profile = `scripts/dev/flight_profiles/PROFILE.env`
  (`PROFILE_AIRFRAME` + `NOCLIMB_*` knobs; caller env wins). FROM/GOAL are layout waypoint names (C-Track: WP3 → GOAL
  default; `A4VAI-Unreal/layout/ctrack_layout.json`) or `X,Y` field coordinates; ALT default 1.7 m above ground.
  Yard map: `CTRACK_MAP=Yard`, the validated course is `2,-78 40,1 2.0`.
- What runs: container `ctrack-sitl` (image px4io/px4-dev-simulation-focal, host network: Gazebo + PX4 + GazeboDrone
  mirroring the pose into Unreal), container `ros2-env` (jociiiii/a4vai:inhousesim-v0.9 on docker network
  `ctrack-ros2` 172.70.0.0/16 at 172.70.0.5 - PX4's uXRCE-DDS client targets 172.70.0.5:8888, DDS domain 0) running
  `scripts/ros2/sitl-px4-airsim.sh` with `ROS2_APP=noclimb_ca` (policy → PathFollowing → flight_driver).
- Results: `~/Documents/A4VAI-SITL/ROS2/test_runs/NAME/` - `result.txt` (landed / timeout / airsim_frozen / ...),
  `px4_sitl.ulg`, `noclimb_ca.csv` (per-step policy log, `mode` column), `pf_log.csv`, `flight_driver.csv`, node logs.
- Analysis: `python3 scripts/dev/ctrack_clearance.py RUN_DIR` (prop-tip margin per obstacle, speed, goal error),
  `python3 scripts/dev/shake_metrics.py RUN_DIR` (roll/pitch rate shake). Compare shake per course section, not one
  whole-flight number: the goal stop dominates the total (see `docs/handoff-x500-demo.md` §2).
- `ctrack.sh stop` stops Unreal, the containers and the `ctrack-ros2` network (it collides with the compose stack's
  `sitl_network`; never run both stacks at once).
- One flight at a time: the harness owns Unreal, the container names and the network. Never edit a script while a
  batch of flights is executing it.

## PILS (Jetson)

Jetson setup, once, from this PC (ssh key login): `scripts/dev/jetson_base_setup.sh user@host` (our base image
`a4vai-base:humble`: JetPack r36.4 + Humble + Micro XRCE-DDS Agent 2.4.3 + onnxruntime), then
`scripts/dev/pils_jetson_setup.sh user@host` (copies this checkout and the installed workspace, builds in the image),
`scripts/dev/onboard_livox_setup.sh user@host` (vehicle image with the Mid-360 driver). Re-run `pils_jetson_setup.sh`
after changing apps (`PILS_SKIP_BUILD=1` for Python-only changes). A Tailscale 100.x `ONBOARD_IP` selects the routed
link mode (`PILS_LINK=wan`: PX4 time sync off, AirSim bridge on the PC relayed over TCP). Verified with Orin Nano
(orin-nano-01/02); JetPack 5 is not supported by the images.

## Real vehicle (`scripts/dev/onboard_real.sh` on the Jetson)

`agent start` (resident uXRCE-DDS agent, UDP-only Fast DDS profile) → `fly PROFILE NAME` / `mission` / `shadow` /
`fly-t1000` → `state` → `go` → `stop NAME`. Read `docs/flight-test-2026-10-17.md` and `docs/real-flight-2026-10-16.md`
before helping with a real flight. Hard rules: restarting the agent requires rebooting the Pixhawk (PX4 1.16); measured
lidar mount / ground height values go in front of the command (`NOCLIMB_LIDAR_Z=.. NOCLIMB_GROUND_Z=.. ...`); the
pilot's RC always overrides; GPS loss → the driver lands in place without horizontal hold (drifts). Never start a
real flight, arm, or send `go` on your own - only on the user's explicit instruction at that moment.

## Legacy AirSim forest flow (v0.9)

`./scripts/run.sh gazebo-classic-airsim-sitl run` (or `run-integrated-algorithm.sh`, `run-pf-test.sh`): compose stack
px4-env / gz-sim / airsim-binary / ros2-env / qgc-app on `sitl_network`. `run.sh` copies `compose.yml`, `envs/` and
`scripts/` into `~/Documents/A4VAI-SITL` on every invocation and fills the blank `VAR=""` placeholders there - edit
the repo, never the deployed copies, and keep the `VAR=""` convention (the sed patterns match it, including the
intentional `GAEZBO_HEADLESS` typo). Swappable apps: `scripts/ros2/apps/<name>.sh` selected by `ROS2_APP`.

## Building

- ROS2: builds run in the ROS2 image with the workspace mounted at `/home/user/workspace/ros2` - the SAME path the
  sim uses (colcon hardcodes absolute paths). `./scripts/run.sh ros2 build ros2_ws [pkg ...]`, or re-run
  `install.sh` (it rebuilds when the source revision changed).
- PX4: `./scripts/run.sh px4 build` (copies the X8/X500 models from `scripts/px4/x8`, `scripts/px4/x500`).
- The Unreal field is a release build: changing maps/plugins happens in the A4VAI-Unreal repo (its `package.sh`).

## Repository map

- `scripts/ctrack.sh` - C-Track entry; `scripts/install.sh`, `uninstall.sh`, `setup-host.sh` - lifecycle
- `scripts/dev/` - C-Track harness (`ctrack_super.sh`, `pils_gate.sh`), PILS/Jetson setup, `onboard_real.sh`,
  `flight_profiles/`, analysis (`ctrack_clearance.py`, `shake_metrics.py`, `pf_*`, `x8_*`, `lidar_mount_calib.py`)
- `scripts/ros2/` - container entrypoints; `apps/noclimb_ca.sh` + `noclimb_ca_node.py` (learned avoidance, all
  `NOCLIMB_*` knobs documented in the .sh header), `apps/mission.sh`, `apps/t1000_pf.sh`, `apps/super_pf.sh`;
  `onboard-real.sh` (vehicle container); `tools/topic_relay.py` (PILS WAN relay)
- `scripts/px4/` - PX4 clone/build, X8 (airframe 2000) and X500 (2001) Gazebo Classic models identified from flight logs
- `envs/*.env` - compose/app configuration (blank placeholders filled by `run.sh`)
- `docs/` - real-flight procedures, results and the X500 demo handoff

## Conventions

- Commits: no Co-Authored-By / Claude-Session trailers in this project.
- Report flight results as measured (landed or not, margins, speed); never present a whole-flight shake number as an
  improvement without the per-section breakdown.
- Lab-specific addresses are parameters (`ONBOARD_IP`, `ONBOARD_SSH`, script arguments); do not hardcode hosts.
