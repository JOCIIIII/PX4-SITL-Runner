#!/usr/bin/env python3
"""Minimal AirSim (Cosys-AirSim) msgpack-RPC client without packages (neither the host nor the ROS2 image has
msgpack) - the subset of msgpack AirSim's RPC uses, as in scripts/dev/airsim_hud.py - plus a camera capture.

    from airsim_rpc import Rpc, capture_png
    png = capture_png('172.70.0.1')          # bytes of a PNG from the vehicle's front_center camera
"""
import socket
import struct


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
    def __init__(self, host, port=41451, timeout=5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
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

    def close(self):
        self.sock.close()


def capture_png(host, port=41451, camera='front_center', vehicle='', timeout=10.0):
    """one compressed Scene image (PNG bytes) from a vehicle camera; raises when AirSim has no such camera"""
    rpc = Rpc(host, port, timeout)
    try:
        # Cosys-AirSim 3.5: simGetImages(requests, vehicle_name) - two arguments (rpclib's arity error message
        # names the counts the other way round)
        req = {'camera_name': camera, 'image_type': 0, 'pixels_as_float': False, 'compress': True}
        res = rpc.call('simGetImages', [req], vehicle)
    finally:
        rpc.close()
    if not res:
        raise RuntimeError('no image returned')
    data = res[0].get('image_data_uint8') if isinstance(res[0], dict) else None
    if isinstance(data, list):
        data = bytes(data)
    if not data:
        raise RuntimeError(f'empty image from camera {camera!r}')
    return data
