#!/usr/bin/env python3
"""ROS 2 topics over one TCP connection - PILS over a routed link (the onboard computer reachable only through
Tailscale / 5G, ~40 ms round trip): the AirSim bridge runs next to Unreal on the simulator PC and its topics are
carried here and republished (DDS discovery does not cross Tailscale; the bridge's RPC polling of AirSim over
that link starved - its start-up segmentation call timed out after 60 s).

  topic_relay.py send HOST:PORT TOPIC [TOPIC ...] [--dedupe]   subscribe (serialized), forward to HOST:PORT
  topic_relay.py recv PORT                                     accept a sender, republish its topics here

--dedupe: forward a message only when its header stamp changes - the AirSim bridge republishes the latest lidar
scan on every 10 ms poll, 10x the data for nothing. A large message (> 16 kB: the lidar) waits for the link only
as the newest unsent one of its topic: when the link is slower than the data (the Mid-360 pattern at 20k points x
10 Hz is ~19 Mbit/s; the Jetson's 5G tether gave ~12) older frames are replaced, so the delay stays one frame long
instead of growing. Small ones (odometry) all go, in order - the adapter registers each scan with the poses around
its time. Messages
keep their own stamps (the receiver does not touch them). The sender reconnects every second until the receiver
is up.
Clouds (x y z float32 PointCloud2) go as int16 centimetres, zlib'd, every point in its place (NaN kept): the
Mid-360 pattern's 252 kB frames (80 % empty shots on the ground) shrink several times.
Frames: 'T' idx(u16) name type (u16-length strings) = a topic; 'D' idx(u16) len(u32) CDR bytes = a message;
'P' idx(u16) len(u32) cloud-without-data CDR, len(u32) zlib(int16 cm) = a cloud."""
import array
import socket
import struct
import sys
import threading
import time
import zlib

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2

CLOUD = 'sensor_msgs/msg/PointCloud2'
NAN_Q = -32768


def encode_cloud(data):
    """x y z float32 PointCloud2 (the AirSim lidar: 21k points, NaN for the empty shots - their order is the
    shot index the adapter de-skews by) -> (the message without its data, zlib of int16 centimetres; NaN and
    out-of-range -> NAN_Q). Every point keeps its place; 1 cm against image bins >= 0.2 m. None: other layout."""
    m = deserialize_message(data, PointCloud2)
    f = m.fields
    if (len(f) != 3 or [x.name for x in f] != ['x', 'y', 'z'] or any(x.datatype != 7 for x in f)
            or m.point_step != 12 or m.is_bigendian):
        return None
    xyz = np.frombuffer(bytes(m.data), dtype=np.float32).reshape(-1, 3)
    with np.errstate(invalid='ignore'):
        q = np.round(xyz * 100.0)
        bad = ~np.isfinite(q).all(axis=1) | (np.abs(q) > 32767).any(axis=1)
    q[bad] = NAN_Q
    m.data = array.array('B')
    return serialize_message(m), zlib.compress(q.astype(np.int16).tobytes(), 1)


def decode_cloud(meta, blob):
    m = deserialize_message(meta, PointCloud2)
    q = np.frombuffer(zlib.decompress(blob), dtype=np.int16).reshape(-1, 3)
    xyz = q.astype(np.float32) / 100.0
    xyz[(q == NAN_Q).any(axis=1)] = np.nan
    m.data = array.array('B', xyz.tobytes())
    return m


def pack_str(s):
    b = s.encode()
    return struct.pack('<H', len(b)) + b


def recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError('closed')
        buf += chunk
    return bytes(buf)


class Sender(Node):
    def __init__(self, host, port, topics, dedupe):
        super().__init__('topic_relay_send')
        self.addr, self.topics, self.dedupe = (host, port), topics, dedupe
        self.types = {}
        while len(self.types) < len(topics):                 # wait for the publishers to appear
            for name, types in self.get_topic_names_and_types():
                if name in topics and name not in self.types:
                    self.types[name] = types[0]
                    self.get_logger().info(f'{name} [{types[0]}]')
            if len(self.types) < len(topics):
                rclpy.spin_once(self, timeout_sec=0.5)
        self.last_stamp = {}
        self.pending = {i: [] for i in range(len(topics))}   # unsent messages per topic (large: only the newest)
        self.cv = threading.Condition()
        self.sent = [0] * len(topics)
        self.replaced = [0] * len(topics)
        self.dropped_dup = 0
        self.raw_bytes = self.link_bytes = 0
        for i, name in enumerate(topics):
            self.create_subscription(get_message(self.types[name]), name,
                                     lambda data, i=i: self.cb(i, data), qos_profile_sensor_data, raw=True)
        threading.Thread(target=self.link, daemon=True).start()
        self.create_timer(5.0, self.report)

    def cb(self, i, data):
        if self.dedupe:
            stamp = data[4:12]
            if self.last_stamp.get(i) == stamp:
                self.dropped_dup += 1
                return
            self.last_stamp[i] = stamp
        frame = None
        if self.types[self.topics[i]] == CLOUD:
            enc = encode_cloud(data)
            if enc is not None:
                meta, blob = enc
                frame = b'P' + struct.pack('<HI', i, len(meta)) + meta + struct.pack('<I', len(blob)) + blob
                self.raw_bytes += len(data)
        if frame is None:
            frame = b'D' + struct.pack('<HI', i, len(data)) + data
            self.raw_bytes += len(data)
        with self.cv:
            q = self.pending[i]
            if q and (len(frame) > 16384 or len(q) >= 200):
                self.replaced[i] += 1
                q.pop(0)
            q.append(frame)
            self.cv.notify()

    def link(self):
        while rclpy.ok():
            try:
                s = socket.create_connection(self.addr, timeout=5.0)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.settimeout(None)
                s.sendall(b''.join(b'T' + struct.pack('<H', i) + pack_str(n) + pack_str(self.types[n])
                                   for i, n in enumerate(self.topics)))
                self.get_logger().info(f'connected to {self.addr[0]}:{self.addr[1]}')
                with self.cv:
                    for q in self.pending.values():
                        q.clear()
                while rclpy.ok():
                    with self.cv:
                        while not any(self.pending.values()):
                            self.cv.wait(1.0)
                        batch = [(i, fr) for i, q in self.pending.items() for fr in q]
                        for q in self.pending.values():
                            q.clear()
                    s.sendall(b''.join(fr for _, fr in batch))
                    for i, fr in batch:
                        self.sent[i] += 1
                        self.link_bytes += len(fr)
            except (OSError, ConnectionError) as e:
                self.get_logger().warn(f'link to {self.addr[0]}:{self.addr[1]}: {e} - retrying')
                time.sleep(1.0)

    def report(self):
        self.get_logger().info('sent per 5 s: ' + ', '.join(f'{n.split("/")[-1]} {c} (replaced {r})' for n, c, r in
                                                           zip(self.topics, self.sent, self.replaced))
                               + f'; duplicates dropped {self.dropped_dup}; {self.link_bytes * 8 / 5e6:.1f} Mbit/s on the link'
                               + f' ({self.raw_bytes * 8 / 5e6:.1f} raw)')
        self.raw_bytes = self.link_bytes = 0
        self.sent = [0] * len(self.topics)
        self.replaced = [0] * len(self.topics)
        self.dropped_dup = 0


class Receiver(Node):
    def __init__(self, port):
        super().__init__('topic_relay_recv')
        self.port = port
        self.pubs = {}
        self.count = {}
        threading.Thread(target=self.serve, daemon=True).start()
        self.create_timer(5.0, self.report)

    def serve(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('0.0.0.0', self.port))
        srv.listen(1)
        self.get_logger().info(f'listening on {self.port}')
        qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE)   # matches best-effort readers too
        while rclpy.ok():
            c, addr = srv.accept()
            c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.get_logger().info(f'sender {addr[0]} connected')
            try:
                while rclpy.ok():
                    kind = recv_exact(c, 1)
                    if kind == b'T':
                        i, = struct.unpack('<H', recv_exact(c, 2))
                        n = recv_exact(c, struct.unpack('<H', recv_exact(c, 2))[0]).decode()
                        t = recv_exact(c, struct.unpack('<H', recv_exact(c, 2))[0]).decode()
                        if i not in self.pubs:
                            self.pubs[i] = (n, self.create_publisher(get_message(t), n, qos))
                            self.count[i] = 0
                            self.get_logger().info(f'republishing {n} [{t}]')
                    elif kind == b'P':
                        i, lm = struct.unpack('<HI', recv_exact(c, 6))
                        meta = recv_exact(c, lm)
                        blob = recv_exact(c, struct.unpack('<I', recv_exact(c, 4))[0])
                        if i in self.pubs:
                            self.pubs[i][1].publish(decode_cloud(meta, blob))
                            self.count[i] += 1
                    elif kind == b'D':
                        i, ln = struct.unpack('<HI', recv_exact(c, 6))
                        data = recv_exact(c, ln)
                        if i in self.pubs:
                            self.pubs[i][1].publish(data)
                            self.count[i] += 1
                    else:
                        raise ConnectionError(f'bad frame {kind!r}')
            except (OSError, ConnectionError) as e:
                self.get_logger().warn(f'sender gone ({e})')
                c.close()

    def report(self):
        if self.pubs:
            self.get_logger().info('received per 5 s: ' + ', '.join(f'{n.split("/")[-1]} {self.count[i]}'
                                                                   for i, (n, _) in self.pubs.items()))
            self.count = {i: 0 for i in self.count}


def main():
    a = sys.argv[1:]
    rclpy.init()
    if a and a[0] == 'send' and len(a) >= 3:
        host, port = a[1].rsplit(':', 1)
        topics = [t for t in a[2:] if not t.startswith('--')]
        node = Sender(host, int(port), topics, '--dedupe' in a)
    elif a and a[0] == 'recv' and len(a) == 2:
        node = Receiver(int(a[1]))
    else:
        print(__doc__)
        sys.exit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
