#!/usr/bin/env python3
"""Flight speed on the AirSim (Unreal) screen, top left: horizontal speed, vertical speed and height,
from the vehicle pose GazeboDrone mirrors into Unreal (= Gazebo's pose), polled over the AirSim
RPC (host port 41451). Also prints how often that pose actually changes - a jerky-looking vehicle
with a smooth PX4 attitude means the mirror (or Unreal's frame rate) is the bottleneck.

No packages needed: a minimal msgpack / msgpack-rpc client is built in (neither the host nor the
ROS2 image has msgpack).

usage: airsim_hud.py [--host 127.0.0.1] [--port 41451] [--rate 10] [--vehicle ''] [--rest-height 0.44] [--fps]
                     [--text-scale 3] [--text-dist 100] [--text-up 0.42] [--text-left 0.1] [--lidar RPLIDAR_A3] [--lidar-rate 1]
       --rate: pose polls per second (the screen line is redrawn at 2 Hz) - kept low: with 50 Hz polls in
       ctrack_super.sh runs the land command got lost and GazeboDrone froze (shares AirSim's RPC server); --rest-height: height of the
       vehicle centre above the ground when it stands on it (x8 0.44 m), so Height reads above ground
       --fps also turns on Unreal's frame-rate overlay (console command 'stat fps')
       --text-scale: the speed also drawn large and red at the top centre of the screen (simPlotStrings in
       front of the viewport camera; the top-left log line has a fixed small font); 0 = off
       --lidar: the farthest and nearest return of the latest scan (the sensor's local frame: NaN = no return),
       in the top-left line, --lidar-rate per second ('' = off; a scan
       is ~20k points, so kept slow)
       Ctrl+C to stop (the last line stays on screen).
"""
import argparse
import math
import socket
import struct
import time


# ---------------- minimal msgpack (the subset AirSim's RPC uses) ----------------
def pack(o):
    if o is None:
        return b'\xc0'
    if o is True:
        return b'\xc3'
    if o is False:
        return b'\xc2'
    if isinstance(o, int):
        if 0 <= o < 128:
            return struct.pack('B', o)
        if -32 <= o < 0:
            return struct.pack('b', o)
        return b'\xd3' + struct.pack('>q', o)
    if isinstance(o, float):
        return b'\xcb' + struct.pack('>d', o)
    if isinstance(o, str):
        b = o.encode('utf-8')
        if len(b) < 32:
            return struct.pack('B', 0xa0 | len(b)) + b
        return b'\xdb' + struct.pack('>I', len(b)) + b
    if isinstance(o, (list, tuple)):
        head = struct.pack('B', 0x90 | len(o)) if len(o) < 16 else b'\xdd' + struct.pack('>I', len(o))
        return head + b''.join(pack(x) for x in o)
    if isinstance(o, dict):
        head = struct.pack('B', 0x80 | len(o)) if len(o) < 16 else b'\xdf' + struct.pack('>I', len(o))
        return head + b''.join(pack(k) + pack(v) for k, v in o.items())
    raise TypeError(type(o))


class Reader:
    def __init__(self, sock):
        self.sock, self.buf, self.pos = sock, bytearray(), 0

    def take(self, n):
        # an offset into one buffer: slicing off the front per item made a lidar reply (60k floats) quadratic
        while len(self.buf) - self.pos < n:
            chunk = self.sock.recv(1 << 20)
            if not chunk:
                raise ConnectionError('AirSim closed the connection')
            self.buf += chunk
        out = bytes(self.buf[self.pos:self.pos + n])
        self.pos += n
        if self.pos > (1 << 20):
            del self.buf[:self.pos]
            self.pos = 0
        return out

    def obj(self):
        t = self.take(1)[0]
        if t < 0x80:
            return t
        if t >= 0xe0:
            return t - 256
        if 0x80 <= t <= 0x8f:
            return {self.obj(): self.obj() for _ in range(t & 0x0f)}
        if 0x90 <= t <= 0x9f:
            return [self.obj() for _ in range(t & 0x0f)]
        if 0xa0 <= t <= 0xbf:
            return self.take(t & 0x1f).decode('utf-8', 'replace')
        simple = {0xc0: None, 0xc2: False, 0xc3: True}
        if t in simple:
            return simple[t]
        fmt = {0xca: ('>f', 4), 0xcb: ('>d', 8), 0xcc: ('>B', 1), 0xcd: ('>H', 2), 0xce: ('>I', 4), 0xcf: ('>Q', 8),
               0xd0: ('>b', 1), 0xd1: ('>h', 2), 0xd2: ('>i', 4), 0xd3: ('>q', 8)}
        if t in fmt:
            f, n = fmt[t]
            return struct.unpack(f, self.take(n))[0]
        if t in (0xd9, 0xda, 0xdb):
            n = struct.unpack({0xd9: '>B', 0xda: '>H', 0xdb: '>I'}[t], self.take({0xd9: 1, 0xda: 2, 0xdb: 4}[t]))[0]
            return self.take(n).decode('utf-8', 'replace')
        if t in (0xc4, 0xc5, 0xc6):
            n = struct.unpack({0xc4: '>B', 0xc5: '>H', 0xc6: '>I'}[t], self.take({0xc4: 1, 0xc5: 2, 0xc6: 4}[t]))[0]
            return self.take(n)
        if t in (0xdc, 0xdd):
            n = struct.unpack('>H' if t == 0xdc else '>I', self.take(2 if t == 0xdc else 4))[0]
            return [self.obj() for _ in range(n)]
        if t in (0xde, 0xdf):
            n = struct.unpack('>H' if t == 0xde else '>I', self.take(2 if t == 0xde else 4))[0]
            return {self.obj(): self.obj() for _ in range(n)}
        raise ValueError(f'msgpack type 0x{t:02x} not handled')


class Rpc:
    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=5.0)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.rd, self.msgid = Reader(self.sock), 0

    def call(self, method, *params):
        self.msgid += 1
        self.sock.sendall(pack([0, self.msgid, method, list(params)]))
        while True:
            msg = self.rd.obj()
            if msg[0] == 1 and msg[1] == self.msgid:
                if msg[2] is not None:
                    raise RuntimeError(f'{method}: {msg[2]}')
                return msg[3]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=41451)
    ap.add_argument('--rate', type=float, default=10.0)
    ap.add_argument('--rest-height', type=float, default=0.44)
    ap.add_argument('--vehicle', default='')
    ap.add_argument('--fps', action='store_true')
    ap.add_argument('--text-scale', type=float, default=3.0)
    ap.add_argument('--text-dist', type=float, default=100.0)
    ap.add_argument('--text-up', type=float, default=0.42)
    ap.add_argument('--text-left', type=float, default=0.1)
    ap.add_argument('--lidar', default='RPLIDAR_A3')
    ap.add_argument('--lidar-rate', type=float, default=1.0)
    a = ap.parse_args()
    rpc = Rpc(a.host, a.port)
    print('AirSim:', rpc.call('ping'))
    if a.fps:
        try:
            rpc.call('simRunConsoleCommand', 'stat fps')
        except RuntimeError as e:
            print('stat fps:', e)
    hist = []                    # (t, x, y, z) of pose changes
    t_text, text_dt, last_cam = 0.0, 0.1, None   # the big speed: redrawn at 10 Hz, each lasting a bit longer
    t_lidar, lidar_txt = 0.0, ''
    last_xyz, n_upd, gaps, t_upd, t_print, t_screen = None, 0, [], None, time.time(), 0.0
    z_ground = None
    dt = 1.0 / a.rate
    while True:
        t0 = time.time()
        pose = rpc.call('simGetVehiclePose', a.vehicle)
        p = pose['position']
        xyz = (p['x_val'], p['y_val'], p['z_val'])                   # NED
        if xyz != last_xyz:                                            # a new pose from the mirror
            if t_upd is not None:
                gaps.append(t0 - t_upd)
            t_upd, last_xyz = t0, xyz
            n_upd += 1
            hist.append((t0, *xyz))
        hist = [h for h in hist if t0 - h[0] <= 0.5]
        # the ground = the lowest the vehicle has been (NED z is largest standing on it), so the
        # tool can be started at any time, also in flight
        z_ground = xyz[2] if z_ground is None else max(z_ground, xyz[2])
        if len(hist) >= 2 and hist[-1][0] - hist[0][0] > 0.1:
            (ta, xa, ya, za), (tb, xb, yb, zb) = hist[0], hist[-1]
            vh = math.hypot(xb - xa, yb - ya) / (tb - ta)
            vz = -(zb - za) / (tb - ta)
        else:
            vh = vz = 0.0
        alt = -(xyz[2] - z_ground) + a.rest_height
        if a.lidar and a.lidar_rate > 0.0 and t0 - t_lidar >= 1.0 / a.lidar_rate:
            t_lidar = t0
            try:
                pc = rpc.call('getLidarData', a.lidar, a.vehicle)['point_cloud']
                r = [math.sqrt(pc[i] ** 2 + pc[i + 1] ** 2 + pc[i + 2] ** 2) for i in range(0, len(pc) - 2, 3)]
                r = [d for d in r if d == d and d > 0.05]
                lidar_txt = f'LiDAR max {max(r):.1f} m  min {min(r):.1f} m' if r else 'LiDAR no returns'
            except (RuntimeError, KeyError, TypeError) as e:
                lidar_txt = f'LiDAR ? ({e})'[:60]
        if a.text_scale > 0.0 and t0 - t_text >= text_dt:
            # screen top centre: a point --text-dist ahead of the viewport camera (PlayerCameraManager_0,
            # far: the string keeps its screen size, and the camera moving between redraws shifts it less;
            # same frame as the plots), raised --text-up x that distance, shifted left by --text-left x it
            # (the string is drawn from its start); moved on by the camera's own motion over the redraw
            cam = rpc.call('simGetObjectPose', 'PlayerCameraManager_0', False)
            c, q = cam['position'], cam['orientation']
            if c['x_val'] == c['x_val']:
                w, x, y, z = q['w_val'], q['x_val'], q['y_val'], q['z_val']
                fwd = (1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y))
                right = (2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x))
                down = (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y))
                cp = (c['x_val'], c['y_val'], c['z_val'])
                vc = tuple((cp[i] - last_cam[i]) / max(t0 - t_text, 1e-3) for i in range(3)) if last_cam else (0, 0, 0)
                last_cam = cp
                d = a.text_dist
                pt = [cp[i] + vc[i] * text_dt + d * fwd[i] - a.text_up * d * down[i] - a.text_left * d * right[i]
                      for i in range(3)]
                rpc.call('simPlotStrings', [f'{vh:.1f} m/s'], [{'x_val': pt[0], 'y_val': pt[1], 'z_val': pt[2]}],
                         a.text_scale, [1.0, 0.0, 0.0, 1.0], 1.05 * text_dt)
            t_text = t0
        if t0 - t_screen >= 0.5:
            rpc.call('simPrintLogMessage', 'Speed', f'{vh:4.1f} m/s   Vz {vz:+4.1f} m/s   Height {alt:4.2f} m   {lidar_txt}', 0)
            t_screen = t0
        if t0 - t_print >= 2.0:
            g = sorted(gaps)
            print(f'pose updates {n_upd / (t0 - t_print):5.1f}/s, gap median {1e3 * g[len(g) // 2] if g else float("nan"):.0f} ms, '
                  f'max {1e3 * g[-1] if g else float("nan"):.0f} ms | {vh:.1f} m/s, height {alt:.2f} m', flush=True)
            n_upd, gaps, t_print = 0, [], t0
        time.sleep(max(0.0, dt - (time.time() - t0)))


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
