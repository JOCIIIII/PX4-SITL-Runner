#!/usr/bin/env python3
"""Run T1000 Control 445 with SITL overrides, without editing its sources.

  T1000_FLIGHT_ALTITUDE_M  cruise height (settings default 2.0). The trained policy flew at
                           2 m; in this AirSim forest 1-2.5 m is the dense undergrowth layer.
  T1000_SETTINGS_JSON      JSON object of settings.py overrides (the README's tuning knobs,
                           e.g. {"AVOID_FORWARD_HALF_ANGLE_DEG": 60}). Model-contract values
                           are refused.
  T1000_CLIMB_FIRST        fraction of the cruise height below which horizontal commands are
                           zeroed (default 0.8; 0 = off). T1000 starts following the path as
                           soon as it lifts off, i.e. through the undergrowth.
  T1000_GOAL_UNLATCH_MARGIN_M  release T1000's goal-stop latch when the vehicle is this far
                           outside GOAL_STOP_RADIUS_M again (default 0.5; 0 = off).

Settings are patched before the node module is imported; the avoidance code reads them at
call time. Everything else is T1000's own CLI.
"""
import dataclasses
import json
import os
import sys

from t1000_control import settings as CFG

CONTRACT = {'OBS_DIM', 'ACT_DIM', 'VX_TARGET', 'MODEL_FORWARD_ACTION_SCALE_MPS',
            'MODEL_LATERAL_ACTION_SCALE_MPS', 'LIDAR_H_BINS', 'LIDAR_V_LAYERS',
            'LIDAR_V_BOUNDARIES', 'LIDAR_MAX', 'LIDAR_MIN', 'LIDAR_TOTAL_BINS'}

alt = os.environ.get('T1000_FLIGHT_ALTITUDE_M')
if alt:
    CFG.FLIGHT_ALTITUDE_M = CFG.ALT_TARGET = CFG.GOAL_ALT = float(alt)

overrides = json.loads(os.environ.get('T1000_SETTINGS_JSON') or '{}')
for key, value in overrides.items():
    if key in CONTRACT:
        sys.exit(f'[t1000_run] refusing to override model-contract setting {key}')
    if not hasattr(CFG, key):
        sys.exit(f'[t1000_run] unknown T1000 setting {key}')
    setattr(CFG, key, value)

# The node opens the ONNX session without naming providers; in this image onnxruntime is
# built with TensorRT/CUDA, which would pick a GPU provider (TensorRT engine build) for a
# 445x2 MLP. Pin CPU (the provider T1000's own preflight uses).
from t1000_control.inference import onnx_policy  # noqa: E402

_policy_init = onnx_policy.OnnxPolicy.__init__


def _cpu_policy_init(self, model_path, *, providers=None):
    _policy_init(self, model_path, providers=providers or ['CPUExecutionProvider'])


onnx_policy.OnnxPolicy.__init__ = _cpu_policy_init

# Climb first: hold horizontal position until the vehicle is near the cruise height.
from t1000_control.core import controller as _ctl  # noqa: E402

CLIMB_FIRST = float(os.environ.get('T1000_CLIMB_FIRST', '0.8'))
# Goal-stop latch: the controller latches _goal_stopped the first time the vehicle is inside
# GOAL_STOP_RADIUS_M, whatever its speed, and from then on caps every command at 0. Passing
# through the radius at 2 m/s during an avoidance manoeuvre left it hovering 2-4 m past the
# goal for good (no dwell -> no LAND). Release the latch once it is clearly outside again so
# its own final-homing branch brings it back.
GOAL_UNLATCH_MARGIN_M = float(os.environ.get('T1000_GOAL_UNLATCH_MARGIN_M', '0.5'))
_step = _ctl.ControlPipeline.step


def _step_sitl(self, **kwargs):
    result = _step(self, **kwargs)
    goal_distance = result.aux.get('goal_distance_m')
    if (GOAL_UNLATCH_MARGIN_M > 0.0 and getattr(self, '_goal_stopped', False)
            and goal_distance is not None
            and goal_distance > CFG.GOAL_STOP_RADIUS_M + GOAL_UNLATCH_MARGIN_M):
        self._goal_stopped = False
    if CLIMB_FIRST > 0.0 and float(result.position_enu[2]) < CLIMB_FIRST * self.flight_altitude_m:
        result = dataclasses.replace(result, velocity_enu=(0.0, 0.0, result.velocity_enu[2]))
    return result


_ctl.ControlPipeline.step = _step_sitl

from t1000_control.__main__ import main  # noqa: E402

if __name__ == '__main__':
    print(f'[t1000_run] FLIGHT_ALTITUDE_M = {CFG.FLIGHT_ALTITUDE_M}  overrides = {overrides}  '
          f'climb_first = {CLIMB_FIRST}', flush=True)
    sys.exit(main())
