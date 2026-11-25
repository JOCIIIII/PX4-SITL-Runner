#!/usr/bin/env python3
"""Real-time ratio of ground_run.sh runs: with lockstep off in the model copies Gazebo free-runs, PX4 (lockstep
scheduler build) takes its clock from the simulator's sensor messages, and the ROS side (PathFollowing, MPPI,
driver) runs on the wall clock - so a slow simulator would change the control timing. PX4's log holds the
uXRCE-DDS time synchronisation (timesync_status: PX4 time vs the agent's wall clock): the slope over the run is
the simulator's real-time ratio. As the R2 Monte Carlo: outside 0.98-1.02 = simulator fault.
  ground_rtf.py RUN_DIR [RUN_DIR ...]      (also the minimum over 10 s windows)"""
import glob
import os
import sys

import numpy as np
from pyulog import ULog

bad = 0
for run in sys.argv[1:]:
    try:
        u = ULog(glob.glob(os.path.join(run, 'px4_rootfs', 'log', '*', '*.ulg'))[0], ['timesync_status'])
        ts = u.data_list[0].data
        t, r = ts['timestamp'].astype(float) * 1e-6, ts['remote_timestamp'].astype(float) * 1e-6
        m = r > 0
        t, r = t[m] - t[m][0], r[m] - r[m][0]
        slope = float(np.polyfit(r, t, 1)[0])
        win = [float(np.polyfit(r[(r >= a) & (r < a + 10)], t[(r >= a) & (r < a + 10)], 1)[0])
               for a in np.arange(0, max(r[-1] - 10, 0.1), 5) if ((r >= a) & (r < a + 10)).sum() >= 5]
        wmin = min(win) if win else slope
        ok = 0.98 <= slope <= 1.02 and 0.98 <= wmin <= 1.02
        bad += not ok
        print(f'{os.path.basename(run.rstrip("/")):34s} real-time ratio {slope:.4f}  (10 s windows min {wmin:.4f})  {"ok" if ok else "OUT OF RANGE"}')
    except Exception as ex:
        bad += 1
        print(f'{os.path.basename(run.rstrip("/")):34s} n/a ({type(ex).__name__}: {ex})')
print(f'{len(sys.argv) - 1 - bad}/{len(sys.argv) - 1} within 0.98-1.02')
