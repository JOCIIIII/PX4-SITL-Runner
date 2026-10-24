#!/usr/bin/env python3
"""Speed guard for the SUPER -> PathFollowing pipeline: brake for what moves into the path.

SUPER plans on a static map. When an obstacle moves into space SUPER has already planned
through, every replan fails and SUPER keeps flying the committed trajectory. In ctrack_123_d
the C-Track RC cart's panel (1 m/s across the lane) did this: the X8 hit it at 3-5 m/s, the
spin broke PX4's estimate and the flight ended upside down. This node sits between the
bridge's /desired_speed and PathFollowing and caps the speed so the vehicle can still stop
before what blocks the path ahead:

  /desired_speed_raw (Float32, super_pf_bridge remapped) -> min(raw, limit) -> /desired_speed
  + /local_waypoint_setpoint_to_PF (bridge path, NED) + /odom, /cloud (world ENU)

Objects, not points: the returns in the vehicle's height band are grouped into clusters (2D
grid, cells within link_cells merge) and each cluster is tracked by the centroid of its occupied
cells; only clusters up to max_moving_extent_m across (and fully in range) can be moving. A track is
moving when a straight-line fit of its centroid over vel_window_s gives >= moving_speed_min with
an RMS residual <= fit_rms_max (a cluster that merged with / split from another jumps, which
the residual rejects) and its cells of the window's start, moved by that velocity, cover its
cells now clearly better than unmoved (rigid motion, not a shrinking view of a static object);
it stays moving for moving_hold_s after the last such fit. The first
version flagged single returns with no return near them a second earlier - the cart's panel
moves along its own length, so all but its front 0.4 m sat on its own old footprint and the
cart was seen 7 m ahead, on and off (ctrack_123_h: hit at 2.6 m/s).

A cluster blocks the path at arc length s (from the vehicle along the path) when
  - static: one of its returns is within block_radius_m of the path (X8 0.8 m: prop tips
    0.77 m; SUPER keeps its path >= robot_r 0.8 m from what it has mapped, so such a return
    is new);
  - moving: one of its returns, carried along the track's velocity over look_m / nominal_speed
    + react_s, comes within block_radius_m of the path, and the vehicle would meet it there
    (see the next paragraph). Moving away from the path: no block.
A moving object is carried at least at moving_speed_assumed (1 m/s): a cart starting from rest
is fitted at half its speed for a while (closed-loop sim: braked 1.2 s late). It does not block
when it has left that stretch of the path clear_margin_s before even a fast vehicle (current
speed or recent commands) gets there, or when the vehicle at its current speed has left it
clear_margin_s before the object comes.
Limit = the speed that still stops standoff_m before the closest block, braking at
brake_accel after delay_s:  v * delay + v^2 / (2 a) <= s - standoff. Above it the command is 0
until the vehicle has stopped (GuardCore.command). PathFollowing guid 0 follows a /desired_speed
drop at ~3.2 m/s^2 after ~0.5 s (ctrack_sp_c/d); 2.5 m/s^2 after 0.6 s leaves room for it: in the
closed-loop sim (2.7 m/s^2 after 0.5 s) 3 m/s^2 after 0.5 s stopped 1.1 m past the standoff.
Mover regions: the cells a confirmed mover covered, and those along its velocity for
mover_region_ahead_s, are remembered mover_region_hold_s (30 s; the cart shuttles in 29 s): their
returns never reach SUPER, and a cluster in one counts as moving with the region's velocity even
when its own track has lapsed (a 2 cm panel seen edge-on: 1-6 returns; its trail stopped the
vehicle on the cart's track in ctrack_s900_g and the cart's return trip hit it).
The closest block is held for block_hold_s, a moving track out of sight is carried along its
velocity for coast_s (braking pitches the lidar's forward rays up: the cart vanished for 1 s in
ctrack_123_i), a static block needs static_seen_min scans in a row (violent pitching lifts
mis-registered ground returns into the band for single scans), and the brake is released only
after the path has been clear for release_s; stopped for a moving block, the vehicle waits there.
Only the latest scan is used, so a moving obstacle's trail does not hold the vehicle after it
has passed; static obstacles stay SUPER's job.

static_cloud_topic (optional): /cloud (or static_cloud_input: the lidar adapter's tilt-gated scans)
republished without the returns of moving objects and below static_cloud_min_z, for
SUPER - its map (raycasting off) never clears a cell, so a moving object leaves a wall of its
past positions that SUPER routes round (ctrack_123_l: along the cart's lane to where it turns) -
and without returns beyond static_cloud_max_range_m (25 m; mis-registered far ground returns
stay in that map as obstacles for good).
dump_path (optional): every scan's band returns + pose + path are pickled there, one record
per scan, for offline replay (GuardCore runs without ROS).
"""
import math
import pickle
from collections import deque

import numpy as np
from scipy import ndimage


def path_coords(path, pts):
    """(lateral distance, arc length) of points (N, 2) to / along the polyline path (M, 2)"""
    a, b = path[:-1], path[1:]
    ab = b - a
    L2 = np.maximum(np.einsum('ij,ij->i', ab, ab), 1e-9)
    cum = np.concatenate([[0.0], np.cumsum(np.sqrt(L2))])[:-1]
    w = np.clip(np.einsum('nmj,mj->nm', pts[:, None, :] - a[None], ab) / L2, 0.0, 1.0)
    d = np.hypot(*(a[None] + w[..., None] * ab[None] - pts[:, None, :]).transpose(2, 0, 1))
    j = np.argmin(d, axis=1)
    n = np.arange(len(pts))
    return d[n, j], cum[j] + w[n, j] * np.sqrt(L2[j])


class Track:
    def __init__(self, tid, t, c, n):
        self.id = tid
        self.hist = deque([(t, float(c[0]), float(c[1]))])
        self.cells = deque()             # (t, occupied cells (K, 2) int) over the fit window
        self.last_xy = None              # returns of the last cluster (coasted when out of sight)
        self.born = t
        self.fits = deque(maxlen=4)      # last fit results (moving or not)
        self.first_moving = None         # centroid at the first moving verdict
        self.seen = 1                    # consecutive scans with a cluster
        self.n = n                       # returns in the last cluster
        self.v = np.zeros(2)             # velocity of the last moving fit
        self.moving_until = -1.0
        self.reported = False
        self.last_t = t

    def centroid(self):
        return np.array(self.hist[-1][1:])

    def predicted(self, t, moving):
        """where to look for it: the median of its last 3 centroids (one off centroid from a few
        sparse returns broke the cart's track in ctrack_123_m), moved on when it is moving"""
        h = np.array(list(self.hist)[-3:])[:, 1:] if self.hist else np.zeros((1, 2))
        return np.median(h, axis=0) + (self.v * (t - self.last_t) if moving else 0.0)


class GuardCore:
    """the guard without ROS: add_scan() per lidar scan, set_path(), limit() per speed tick"""

    def __init__(self, block_radius=0.8, standoff=2.0, brake_accel=2.5, delay=0.6, look=15.0,
                 nominal_speed=5.0, react=0.5, dz=(0.7, 0.5), cell=0.3, link_cells=1,
                 cluster_range=20.0, gate=1.5, vel_window=1.5, vel_min_span=0.6, vel_min_n=5,
                 moving_speed_min=0.5, fit_rms_max=0.2, moving_hold=1.5, block_hold=0.8,
                 track_drop=0.6, pred_dt=0.25, max_pred_pts=40, moving_speed_assumed=1.0, stopped_speed=0.3,
                 clear_margin=1.0, max_moving_extent=2.0, shift_gain_min=0.15, coast=2.5, release=0.5,
                 static_seen_min=2, min_track_age=1.0, fits_in_row=3, region_hold=30.0, region_ahead=10.0,
                 min_moving_points=5, region_ahead_after_m=1.0):
        self.r_block, self.standoff, self.a, self.tau = block_radius, standoff, brake_accel, delay
        self.look, self.v_nom, self.t_react, self.dz = look, nominal_speed, react, dz
        self.cell, self.link, self.c_range, self.gate = cell, link_cells, cluster_range, gate
        self.vel_window, self.vel_min_span, self.vel_min_n = vel_window, vel_min_span, vel_min_n
        self.v_move, self.rms_max, self.move_hold = moving_speed_min, fit_rms_max, moving_hold
        self.block_hold, self.track_drop, self.pred_dt, self.max_pred = block_hold, track_drop, pred_dt, max_pred_pts
        self.u_min, self.v_stopped, self.clear_margin = moving_speed_assumed, stopped_speed, clear_margin
        self.max_extent, self.shift_gain = max_moving_extent, shift_gain_min
        self.coast, self.release, self.static_seen = coast, release, static_seen_min
        self.min_age, self.fits_in_row = min_track_age, fits_in_row
        # mover regions: the cells a confirmed mover covered (and those along its velocity for
        # region_ahead seconds), one cell dilated, kept region_hold seconds with its velocity.
        # A 2 cm panel edge-on drops to 1-6 returns, its track lapses within 1.5 s and its returns
        # reached SUPER: a trail across the lane in a map that never clears (ctrack_s900_f / g,
        # the vehicle stopped on the cart's track and the cart's return trip hit it). The cart's
        # shuttle period is 29 s.
        self.region_hold, self.region_ahead = region_hold, region_ahead
        # a fit counts only with min_moving_points returns: 1-3 returns of the balloon 17 m behind
        # the vehicle jittered into a 0.6 m/s 'mover' (replay of ctrack_s900_e); and the region
        # reaches ahead only of a mover that has gone region_ahead_after_m since its verdict
        self.min_pts_moving, self.ahead_after = min_moving_points, region_ahead_after_m
        self.region = {}                 # cell key -> (expiry, vx, vy)
        self.clear_since = None          # since when the path is clear while braking
        self.braking = False             # hard brake latched (command 0 until stopped or clear)
        self.cmd_hist = deque()          # (t, command) of the last delay seconds
        self.tracks = []
        self.next_id = 0
        self.clusters = []               # (xy returns, track) of the latest scan
        self.scan_t = None
        self.path = None
        self.held = None                 # (expiry, point on the path, kind, lateral)
        self.events = []                 # log lines for the node

    def set_path(self, path_xy):
        if len(path_xy) >= 2:
            self.path = np.asarray(path_xy, dtype=float)

    def _clusters(self, xy):
        ij = np.floor(xy / self.cell).astype(np.int64)
        ij -= ij.min(axis=0) - self.link
        occ = np.zeros(tuple(ij.max(axis=0) + self.link + 1), dtype=bool)
        occ[ij[:, 0], ij[:, 1]] = True
        k = 2 * self.link + 1
        lab, n = ndimage.label(ndimage.binary_dilation(occ, structure=np.ones((k, k))),
                               structure=np.ones((3, 3)))
        pl = lab[ij[:, 0], ij[:, 1]]
        order = np.argsort(pl, kind='stable')
        cuts = np.flatnonzero(np.diff(pl[order])) + 1
        return [xy[g] for g in np.split(order, cuts)]

    @staticmethod
    def _overlap(new, old, shift):
        """share of the cells new that lie within one cell of the cells old moved by shift (cells)"""
        o = old + np.round(shift).astype(np.int64)
        near = np.concatenate([o + np.array([di, dj]) for di in (-1, 0, 1) for dj in (-1, 0, 1)])
        key = lambda ij: ij[:, 0] * 1000003 + ij[:, 1]
        return float(np.isin(key(new), key(near)).mean())

    def _update_track(self, tr, t, c, n, truncated, cells):
        if truncated or not 0.5 <= n / max(tr.n, 1) <= 2.0:
            tr.hist.clear()                  # cut off by the range, or merged / split: start over
            tr.cells.clear()
        tr.hist.append((t, float(c[0]), float(c[1])))
        tr.cells.append((t, cells))
        tr.n, tr.last_t = n, t
        while tr.hist and t - tr.hist[0][0] > self.vel_window:
            tr.hist.popleft()
        while tr.cells and t - tr.cells[0][0] > self.vel_window:
            tr.cells.popleft()
        if len(tr.hist) < self.vel_min_n or tr.hist[-1][0] - tr.hist[0][0] < self.vel_min_span:
            return
        tr.fits.append(False)
        h = np.array(tr.hist)
        # straight-line fit, refitted without outliers: at 10-15 m a few sparse returns put the
        # centroid 0.5-1.2 m off for a scan, which failed every fit of the cart in ctrack_123_m
        inl = np.ones(len(h), dtype=bool)
        for _ in range(2):
            tm = h[inl, 0].mean()
            tt = h[inl, 0] - tm
            den = float(tt @ tt)
            if den < 1e-6:
                return
            vx, vy = (tt @ h[inl, 1]) / den, (tt @ h[inl, 2]) / den
            mx, my = h[inl, 1].mean(), h[inl, 2].mean()
            res = np.hypot(h[:, 1] - mx - vx * (h[:, 0] - tm), h[:, 2] - my - vy * (h[:, 0] - tm))
            inl = res <= max(0.25, 2.5 * float(np.median(res)))
        if inl.sum() < max(self.vel_min_n, 0.7 * len(h)) or np.ptp(h[inl, 0]) < self.vel_min_span:
            return
        if math.hypot(vx, vy) < self.v_move or math.sqrt(float(np.mean(res[inl] ** 2))) > self.rms_max:
            return
        if n < self.min_pts_moving:
            return
        # rigid motion: the oldest inlier cells moved by v dt must cover the new ones clearly
        # better than unmoved. A centroid also walks when the visible part of a static object
        # shrinks (a wall end-on as the vehicle comes alongside: 1.2 m/s, replay of ctrack_123_e) -
        # the new cells are then a subset of the old ones either way.
        t_first = float(h[inl, 0][0])
        old = next((c for tc, c in tr.cells if tc >= t_first - 1e-6), tr.cells[0][1])
        dt = t - t_first
        moved = self._overlap(cells, old, np.array([vx, vy]) * dt / self.cell)
        if moved < 0.5 or moved < self._overlap(cells, old, np.zeros(2)) + self.shift_gain:
            return
        tr.fits[-1] = True
        # a new track (just come into view: its visible part still changes) or a lone fit (sparse
        # returns at 15 m: a wall 2-3 rings tall wandered 0.9 m in 0.6 s, ctrack_123_k) is not
        # enough: 3 of the last 4 fits
        if t - tr.born < self.min_age or sum(tr.fits) < self.fits_in_row:
            return
        tr.v = np.array([vx, vy])
        tr.moving_until = t + self.move_hold
        if tr.first_moving is None:
            tr.first_moving = np.array(c, dtype=float)
        if not tr.reported:
            for _, old_cells in tr.cells:                 # what it swept before the verdict
                self._mark_region(t, old_cells, np.array([vx, vy]))
            tr.reported = True
            self.events.append(f'moving object #{tr.id} at ({c[0]:.1f}, {c[1]:.1f}): '
                               f'{math.hypot(vx, vy):.2f} m/s toward {math.degrees(math.atan2(vy, vx)):.0f} deg')

    @staticmethod
    def _keys(ij):
        return ij[:, 0] * 1000003 + ij[:, 1]

    def _mark_region(self, t, cells, v):
        """cells (K, 2) of a confirmed mover + those along v for region_ahead s, dilated one cell"""
        sp = float(np.hypot(*v))
        steps = [0.0]
        if sp > 1e-3:
            steps += list(np.arange(self.cell / sp, self.region_ahead + 1e-6, self.cell / sp))
        ahead = np.concatenate([cells + np.round(v * tau / self.cell).astype(np.int64) for tau in steps])
        near = np.concatenate([ahead + np.array([di, dj]) for di in (-1, 0, 1) for dj in (-1, 0, 1)])
        entry = (t + self.region_hold, float(v[0]), float(v[1]))
        for k in np.unique(self._keys(near)).tolist():
            self.region[k] = entry

    def region_velocity(self, xy, t):
        """(mask of xy in a live mover region, median velocity of those cells) """
        if not self.region or not len(xy):
            return np.zeros(len(xy), dtype=bool), None
        ks = self._keys(np.floor(xy / self.cell).astype(np.int64)).tolist()
        hit = [self.region.get(k) for k in ks]
        mask = np.array([e is not None and e[0] >= t for e in hit])
        if not mask.any():
            return mask, None
        vs = np.array([[e[1], e[2]] for e, m in zip(hit, mask) if m])
        return mask, np.median(vs, axis=0)

    def moving_mask(self, xyz):
        """returns of xyz (any height) within one cell of a moving track's latest cluster - its
        footprint, so the cart's body below the band goes too - or in a live mover region"""
        keys = [np.unique(np.floor(xy / self.cell).astype(np.int64), axis=0) for xy, tr in self.clusters
                if tr.moving_until >= self.scan_t]
        ij = np.floor(xyz[:, :2] / self.cell).astype(np.int64)
        mask = np.zeros(len(xyz), dtype=bool)
        if keys:
            k = np.concatenate(keys)
            near = np.concatenate([k + np.array([di, dj]) for di in (-1, 0, 1) for dj in (-1, 0, 1)])
            mask |= np.isin(self._keys(ij), self._keys(near))
        if self.region:
            live = [k for k, e in self.region.items() if e[0] >= self.scan_t]
            if live:
                mask |= np.isin(self._keys(ij), np.array(live, dtype=np.int64))
        return mask

    def add_scan(self, t, xyz, pos):
        """xyz: (N, 3) returns, world; pos: vehicle position (3,)"""
        rel = xyz - pos
        r2 = rel[:, 0] ** 2 + rel[:, 1] ** 2
        xyb = xyz[(rel[:, 2] > -self.dz[0]) & (rel[:, 2] < self.dz[1]) & (r2 <= self.c_range ** 2), :2]
        clusters = self._clusters(xyb) if len(xyb) else []
        # centroid of the occupied cells, not of the returns: the near end of a wall gets more
        # returns as the vehicle closes in, which walked the return centroid toward the vehicle
        # at up to 1.2 m/s (replay of ctrack_123_e)
        cells = [np.unique(np.floor(c / self.cell).astype(np.int64), axis=0) for c in clusters]
        cents = np.array([(k.mean(axis=0) + 0.5) * self.cell for k in cells]).reshape(-1, 2)
        # only a cluster with all its returns in range and at most max_moving_extent across can be
        # moving (a cart, a person); the visible part of anything bigger changes with the viewpoint
        far = [bool(np.max(np.hypot(*(c - pos[:2]).T)) > self.c_range - 1.0
                    or np.hypot(*np.ptp(c, axis=0)) > self.max_extent) for c in clusters]
        # greedy nearest association to the live tracks (a moving one at its predicted place; it
        # is kept coast seconds out of sight: braking pitches the lidar's forward rays up and the
        # cart's panel vanished for 1 s - its track was dropped and the brake released, ctrack_123_i)
        self.tracks = [tr for tr in self.tracks
                       if t - tr.last_t <= (self.coast if tr.moving_until >= tr.last_t else self.track_drop)]
        owner = [None] * len(clusters)
        if self.tracks and clusters:
            tc = np.array([tr.predicted(t, tr.moving_until >= tr.last_t) for tr in self.tracks])
            d = np.hypot(cents[:, None, 0] - tc[None, :, 0], cents[:, None, 1] - tc[None, :, 1])
            used = set()
            for k in np.argsort(d, axis=None):
                i, j = divmod(int(k), len(self.tracks))
                if d[i, j] > self.gate:
                    break
                if owner[i] is None and j not in used:
                    owner[i] = self.tracks[j]
                    used.add(j)
        for tr in self.tracks:
            if tr not in owner:
                tr.seen = 0
        for i, c in enumerate(clusters):
            if owner[i] is None:
                owner[i] = Track(self.next_id, t, cents[i], len(c))
                owner[i].cells.append((t, cells[i]))
                self.next_id += 1
                self.tracks.append(owner[i])
            else:
                owner[i].seen += 1
                self._update_track(owner[i], t, cents[i], len(c), far[i], cells[i])
            owner[i].last_xy = c
        self.clusters = list(zip(clusters, owner))
        self.scan_t = t
        for i, tr in enumerate(owner):
            if tr is not None and tr.moving_until >= t:
                gone_far = (tr.first_moving is not None
                            and float(np.hypot(*(cents[i] - tr.first_moving))) >= self.ahead_after)
                self._mark_region(t, cells[i], tr.v if gone_far else np.zeros(2))
        if self.region and (len(self.region) > 20000 or int(t * 10) % 50 == 0):
            self.region = {k: e for k, e in self.region.items() if e[0] >= t}

    def limit(self, t, pos, speed=0.0, speed_fast=None):
        """(speed limit, arc length of the block, kind, lateral) or None when the path is clear.
        speed: the vehicle's; speed_fast: the most it may reach soon (recent commands)"""
        speed_fast = speed if speed_fast is None else speed_fast
        if self.path is None:
            return None
        lat0, s_veh = path_coords(self.path, pos[None, :2])
        s0 = float(s_veh[0])
        best = None                                       # (s, kind, lateral)
        if self.scan_t is not None and t - self.scan_t <= 0.5:
            items = [(xy, tr, tr.moving_until >= t) for xy, tr in self.clusters]
            here = {id(tr) for _, tr in self.clusters}
            items += [(tr.last_xy + tr.v * (t - tr.last_t), tr, True) for tr in self.tracks
                      if id(tr) not in here and tr.moving_until >= tr.last_t and tr.last_xy is not None]
            for xy, tr, moving in items:
                near = xy[np.hypot(*(xy - pos[:2]).T) <= self.look + 3.0 + self.r_block]
                v_track = tr.v
                if not moving:
                    # in a mover region: the mover (its track lapsed) - carried at the region's velocity
                    in_reg, v_reg = self.region_velocity(xy, t)
                    if v_reg is not None and in_reg.mean() >= 0.5 and float(np.hypot(*v_reg)) > 0.1:
                        moving, v_track = True, v_reg
                if moving:
                    pts = xy if len(xy) <= self.max_pred else xy[np.linspace(0, len(xy) - 1, self.max_pred).astype(int)]
                    taus = np.arange(0.0, self.look / self.v_nom + self.t_react + 1e-6, self.pred_dt)
                    sp = float(np.hypot(*v_track))
                    u = v_track * max(1.0, self.u_min / max(sp, 1e-6))  # at least moving_speed_assumed
                    q = (pts[:, None, :] + u[None, None, :] * taus[None, :, None]).reshape(-1, 2)
                    lat, s = path_coords(self.path, q)
                    s = s - s0
                    tq = np.tile(taus, len(pts))
                    ok = (s > 0.0) & (s <= self.look) & (lat <= self.r_block)
                    if ok.any():
                        # the object is on the stretch [s_lo, s_hi] of the path from t_in to t_out
                        # (still there at the horizon: for good). No block when it has left
                        # clear_margin before even a fast vehicle arrives, or when the vehicle at
                        # its speed has passed clear_margin before it comes (braking would only
                        # slow the vehicle down in its way: closed-loop sim, -0.67 m).
                        t_in, t_out = float(tq[ok].min()), float(tq[ok].max())
                        if t_out >= taus[-1] - 1e-6:
                            t_out = math.inf
                        s_lo, s_hi = float(s[ok].min()), float(s[ok].max())
                        gone = t_out + self.clear_margin < (s_lo - self.r_block) / max(speed_fast, 0.1)
                        passed = (s_hi + self.r_block) / max(speed, 0.1) + self.clear_margin < t_in
                        if gone or passed:
                            ok[:] = False
                    kind = 'moving'
                elif len(near) and tr.seen >= self.static_seen:      # not a one-scan artefact
                    lat, s = path_coords(self.path, near)
                    s = s - s0
                    ok = (s > 0.0) & (s <= self.look) & (lat <= self.r_block)
                    kind = 'static'
                else:
                    continue
                if ok.any():
                    i = int(np.argmin(np.where(ok, s, np.inf)))
                    if best is None or s[i] < best[0]:
                        best = (float(s[i]), kind, float(lat[i]))
        if best is not None:
            # hold the block as a point on the path
            seg = self.path
            arc = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(seg, axis=0).T))])
            p = np.array([np.interp(s0 + best[0], arc, seg[:, 0]), np.interp(s0 + best[0], arc, seg[:, 1])])
            self.held = (t + self.block_hold, p, best[1], best[2])
        elif self.held is not None and t <= self.held[0]:
            lat, s = path_coords(self.path, self.held[1][None, :])
            if s[0] - s0 > 0.0 and lat[0] <= self.r_block:
                best = (float(s[0] - s0), self.held[2] + ' (held)', self.held[3])
        if best is None:
            return None
        gap = max(best[0] - self.standoff, 0.0)
        v = self.a * (math.sqrt(self.tau ** 2 + 2.0 * gap / self.a) - self.tau)
        return v, best[0], best[1], best[2]

    def command(self, t, pos, speed, raw):
        """(speed command, limit() result or None). Below the stopping curve the command is capped
        by it; above it (the vehicle can no longer stop smoothly - PathFollowing tracks a slowly
        falling command with more lag than the curve assumes: the closed-loop sim hit the panel
        following it) the command is 0 until the vehicle has stopped or the path is clear. 'Above'
        counts the commands of the last delay seconds too: the vehicle is still heading for them."""
        while self.cmd_hist and t - self.cmd_hist[0][0] > self.tau:
            self.cmd_hist.popleft()
        v_eff = max([speed] + [c for _, c in self.cmd_hist])
        lim = self.limit(t, pos, speed, max(v_eff, raw))
        if lim is None:
            # released once the path has been clear for `release` (the cart dropped out of sight
            # for single scans while braking: stop-and-go, ctrack_123_i)
            if self.braking:
                self.clear_since = t if self.clear_since is None else self.clear_since
                self.braking = t - self.clear_since < self.release
            out = 0.0 if self.braking else raw
        else:
            self.clear_since = None
            # stopped for a static block: creep up to the standoff (SUPER replans around it); for a
            # moving one wait until it has gone
            if self.braking and speed < self.v_stopped and lim[2].startswith('static'):
                self.braking = False
            if not self.braking and v_eff > lim[0] and lim[0] < raw:
                self.braking = True
            out = 0.0 if self.braking else min(raw, lim[0])
        self.cmd_hist.append((t, out))
        return out, lim


def make_node():
    """the ROS node (rclpy must be initialised); separate from main() so a test can drive its
    callbacks directly"""
    import sensor_msgs_py.point_cloud2 as pc2
    from custom_msgs.msg import LocalWaypointSetpoint
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Float32

    class SpeedGuard(Node):
        def __init__(self):
            super().__init__('super_speed_guard')

            def p(name, default):
                return self.declare_parameter(name, default).value
            self.core = GuardCore(
                block_radius=float(p('block_radius_m', 0.8)), standoff=float(p('standoff_m', 2.0)),
                brake_accel=float(p('brake_accel', 2.5)), delay=float(p('delay_s', 0.6)),
                look=float(p('look_m', 15.0)), nominal_speed=float(p('nominal_speed', 5.0)),
                react=float(p('react_s', 0.5)),
                dz=(float(p('dz_below_m', 0.7)), float(p('dz_above_m', 0.5))),
                cell=float(p('cell_m', 0.3)), cluster_range=float(p('cluster_range_m', 20.0)),
                gate=float(p('gate_m', 1.5)), vel_window=float(p('vel_window_s', 1.5)),
                moving_speed_min=float(p('moving_speed_min', 0.5)), fit_rms_max=float(p('fit_rms_max', 0.2)),
                moving_hold=float(p('moving_hold_s', 1.5)), block_hold=float(p('block_hold_s', 0.8)),
                moving_speed_assumed=float(p('moving_speed_assumed', 1.0)), coast=float(p('coast_s', 2.5)),
                release=float(p('release_s', 0.5)), region_hold=float(p('mover_region_hold_s', 30.0)),
                region_ahead=float(p('mover_region_ahead_s', 10.0)))
            # the scans without moving objects' returns, for SUPER: its map (raycasting off) never
            # clears a cell, so the cart's track stayed a wall across the lane and SUPER led the
            # vehicle round its end - where the cart turns (ctrack_123_l: 1.1 m from the panel).
            # Moving objects are this node's job. '' = off.
            self.static_topic = str(p('static_cloud_topic', ''))
            self.static_range = float(p('static_cloud_max_range_m', 25.0))
            # Scans shot while the vehicle tilts hard are mis-registered (ground ~1 m up, phantoms
            # SUPER never clears: ctrack_123_j); the lidar adapter gates them out of its gated_topic
            # by each scan's own sweep. The guard tracks with every scan on /cloud - gaps from
            # dropping them there kept restarting the RC cart's track (ctrack_s900_c).
            # static_cloud_input: take /cloud_static from this topic instead of /cloud - the lidar
            # adapter's gated_topic (an odom-based gate here saw the attitude 0.1-0.2 s late and let
            # mis-registered ground through: ctrack_s900_d stalled on it). '' = from /cloud.
            self.static_input = str(p('static_cloud_input', ''))
            # static_cloud_min_z: returns below this height (PX4-local z) are not passed on - at a
            # held height nothing that low can reach the vehicle, and mis-registered ground lands
            # there (0.7-0.9 m above ground in ctrack_s900_d). Default: no limit.
            self.static_min_z = float(p('static_cloud_min_z', -1.0e9))

            self.static_pub = (self.create_publisher(PointCloud2, self.static_topic, qos_profile_sensor_data)
                               if self.static_topic else None)
            self.n_dropped = 0
            dump = str(p('dump_path', ''))
            self.dump = open(dump, 'ab') if dump else None
            self.pos = None
            self.speed = 0.0
            self.last_stamp = None
            self.n_cut = 0
            self.n_err = 0
            self.n_blind = 0
            self.last_scan_rx = None
            self.last_scan_ok = -1e9
            self.limiting = False
            self.speed_pub = self.create_publisher(Float32, '/desired_speed', 10)
            self.create_subscription(Float32, '/desired_speed_raw', self.speed_cb, 10)
            self.create_subscription(LocalWaypointSetpoint, '/local_waypoint_setpoint_to_PF', self.path_cb, 10)
            self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
            self.create_subscription(PointCloud2, '/cloud', self.cloud_cb, qos_profile_sensor_data)
            if self.static_pub is not None and self.static_input:
                self.create_subscription(PointCloud2, self.static_input, self.static_cb, qos_profile_sensor_data)
            c = self.core
            self.get_logger().info(
                f'[super_speed_guard] block {c.r_block} m (moving objects: >= {c.v_move} m/s over {c.vel_window} s, '
                f'carried at >= {c.u_min} m/s for {c.look / c.v_nom + c.t_react:.1f} s), stop {c.standoff} m short, '
                f'brake {c.a} m/s^2 after {c.tau} s, look {c.look} m')

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def odom_cb(self, m):
            q = m.pose.pose.position
            self.pos = np.array([q.x, q.y, q.z])
            self.speed = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)


        def path_cb(self, m):
            if len(m.waypoint_x) >= 2:
                self.core.set_path(np.column_stack([m.waypoint_y, m.waypoint_x]))   # NED -> ENU (e, n)

        def cloud_cb(self, m):
            stamp = (m.header.stamp.sec, m.header.stamp.nanosec)
            if self.pos is None or stamp == self.last_stamp:
                if self.static_pub is not None and stamp != self.last_stamp:
                    self.static_pub.publish(m)
                return
            self.last_stamp = stamp
            t = self.now()
            pts = pc2.read_points(m, skip_nans=False)
            xyz = np.column_stack([pts['x'], pts['y'], pts['z']]).astype(np.float64)
            ok = np.isfinite(xyz).all(axis=1)
            drop = np.zeros(len(xyz), dtype=bool)
            self.last_scan_rx = t
            try:
                self.core.add_scan(t, xyz[ok], self.pos.copy())
                drop[np.flatnonzero(ok)] = self.core.moving_mask(xyz[ok])
                self.last_scan_ok = t
            except Exception as e:
                self.n_err += 1
                if self.n_err % 50 == 1:
                    self.get_logger().error(f'[super_speed_guard] scan: {type(e).__name__}: {e}')
            if self.static_pub is not None and not self.static_input:
                # and nothing beyond static_max_range: grazing ground returns 30-40 m out, a
                # degree or two off in pitch while the vehicle speeds up or brakes, landed
                # 0.5-1 m up next to the goal - SUPER never clears a cell and could not plan
                # into it for 6 min (ctrack_123_p)
                n_moving = int(drop.sum())
                if self.static_range > 0.0:
                    far = np.ones(len(xyz), dtype=bool)
                    far[ok] = np.einsum('ij,ij->i', xyz[ok] - self.pos, xyz[ok] - self.pos) > self.static_range ** 2
                    drop |= far
                drop |= ~(xyz[:, 2] >= self.static_min_z)
                if drop.any():
                    self.static_pub.publish(pc2.create_cloud(m.header, m.fields, pts[~drop]))
                    self.n_dropped += 1
                    if n_moving and self.n_dropped % 50 == 1:
                        self.get_logger().info(f'[super_speed_guard] {n_moving} returns of moving objects '
                                               f'kept out of {self.static_topic} ({self.n_dropped} scans so far)')
                else:
                    self.static_pub.publish(m)
            xyz = xyz[ok]
            if self.dump is not None:
                rel = xyz - self.pos
                band = (rel[:, 2] > -1.5) & (rel[:, 2] < 1.5) & (rel[:, 0] ** 2 + rel[:, 1] ** 2 <= 30.0 ** 2)
                pickle.dump({'t': t, 'pos': self.pos.copy(), 'xyz': xyz[band].astype(np.float32),
                             'path': self.core.path}, self.dump)
            for e in self.core.events:
                self.get_logger().info(f'[super_speed_guard] {e}')
            self.core.events.clear()

        def static_cb(self, m):
            """/cloud_static from the gated scans: without the returns of moving objects (the tracks
            of the latest /cloud scan), beyond static_max_range or below static_min_z"""
            if self.pos is None:
                return
            pts = pc2.read_points(m, skip_nans=False)
            xyz = np.column_stack([pts['x'], pts['y'], pts['z']]).astype(np.float64)
            ok = np.isfinite(xyz).all(axis=1)
            drop = ~ok
            try:
                drop[ok] |= self.core.moving_mask(xyz[ok]) if self.core.scan_t is not None else False
            except Exception as e:
                self.n_err += 1
                if self.n_err % 50 == 1:
                    self.get_logger().error(f'[super_speed_guard] static: {type(e).__name__}: {e}')
            n_moving = int(drop[ok].sum())
            if self.static_range > 0.0:
                d2 = np.full(len(xyz), np.inf)
                d2[ok] = np.einsum('ij,ij->i', xyz[ok] - self.pos, xyz[ok] - self.pos)
                drop |= d2 > self.static_range ** 2
            drop |= ~(xyz[:, 2] >= self.static_min_z)
            self.static_pub.publish(pc2.create_cloud(m.header, m.fields, pts[~drop]) if drop.any() else m)
            if n_moving:
                self.n_dropped += 1
                if self.n_dropped % 50 == 1:
                    self.get_logger().info(f'[super_speed_guard] {n_moving} returns of moving objects kept out of '
                                           f'{self.static_topic} ({self.n_dropped} scans so far)')

        def speed_cb(self, m):
            raw = float(m.data)
            out, lim = raw, None
            now = self.now()
            try:
                if math.isfinite(raw) and self.pos is not None:
                    out, lim = self.core.command(now, self.pos, self.speed, raw)
            except Exception as e:           # a guard that cannot judge stops the vehicle
                out, lim = 0.0, None
                self.n_err += 1
                if self.n_err % 50 == 1:
                    self.get_logger().error(f'[super_speed_guard] {type(e).__name__}: {e} - STOP')
            # blind (scans arrive but are not processed - a numpy 2 incompatibility made every scan
            # fail and the guard let the vehicle into the cart, ctrack_123_n): stop too
            if self.last_scan_rx is not None and self.last_scan_rx - self.last_scan_ok > 0.5:
                out = 0.0
                self.n_blind += 1
                if self.n_blind % 50 == 1:
                    self.get_logger().error('[super_speed_guard] scans arriving but not processed for '
                                            f'{self.last_scan_rx - self.last_scan_ok:.1f} s - STOP')
            if out < raw:
                self.n_cut += 1
                if not self.limiting or self.n_cut % 20 == 1:
                    why = (f'{lim[2]} block {lim[1]:.1f} m ahead, {lim[3]:.2f} m off the path' if lim is not None
                           else 'path clear, brake held')
                    self.get_logger().info(
                        f'[super_speed_guard] {raw:.2f} -> {out:.2f} m/s at {self.speed:.2f}: {why}'
                        f'{" - BRAKE" if self.core.braking else ""}')
                self.limiting = True
            elif self.limiting:
                self.get_logger().info(f'[super_speed_guard] path clear ({raw:.2f} m/s)')
                self.limiting = False
            o = Float32()
            o.data = float(out)
            self.speed_pub.publish(o)

    return SpeedGuard()


def main():
    import rclpy
    rclpy.init()
    n = make_node()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        if n.dump is not None:
            n.dump.close()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
