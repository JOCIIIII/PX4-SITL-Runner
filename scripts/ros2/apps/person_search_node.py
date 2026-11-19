#!/usr/bin/env python3
"""Person search: while a search mission flies (llm_mission_node.py task=search, /search/enable), the camera
'search_cam' (AirSim, scripts/dev/unreal_settings/*_cam.json) is read a few times a second, YOLOX finds people
(tools/person_detector.py), and each image's camera pose puts them on the ground (NED = the C-Track layout X, Y).
A person seen in confirm_n frames within confirm_s s, away from the ones already reported, is inspected:

  /flight_driver/pause true -> the vehicle holds, then moves (/flight_driver/hold_at) to where the person is in the
  middle of the camera -> a new image; the person found again near that spot, else the best frame of the pass
  -> photo + crop saved, the vision LLM (Ollama, image input) says what it sees ("not a person" = false alarm)
  -> report on /llm/reply (where, how sure, what) -> /flight_driver/pause false -> the search goes on

At the end (flight_driver DONE) search_results.json in log_dir lists them (scripts/dev/search_report.py draws it).
ROS parameters: model, camera (search_cam), fov_deg (90), rate_hz (3), score (0.3: tracked), confirm_score (0.5: the
best frame needed to stop for it), confirm_n (2), confirm_s (3), merge_m (3), dedupe_m (6), view_dist_m (5: the camera
looks 50 deg down from 6 m), airsim_ip, llm_url, llm_model, vlm (true), origin_xy, places, log_dir.
"""
import base64
import json
import math
import os
import re
import sys
import threading
import time
import urllib.request

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, String

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
from airsim_rpc import Rpc                                          # noqa: E402
from person_detector import PersonDetector, ground_point            # noqa: E402


class PersonSearch(Node):
    def __init__(self):
        super().__init__('person_search')
        p = lambda n, d: self.declare_parameter(n, d).value
        self.model = str(p('model', '/home/user/workspace/ros2/models/yolox_s.onnx'))
        self.camera = str(p('camera', 'search_cam'))
        self.fov = float(p('fov_deg', 90.0))
        self.period = 1.0 / float(p('rate_hz', 3.0))
        self.score = float(p('score', 0.3))
        self.confirm_score = float(p('confirm_score', 0.5))
        self.view_dist = float(p('view_dist_m', 5.0))
        self.confirm_n = int(p('confirm_n', 2))
        self.confirm_s = float(p('confirm_s', 3.0))
        self.merge_m = float(p('merge_m', 3.0))
        self.dedupe_m = float(p('dedupe_m', 6.0))
        self.airsim_ip = str(p('airsim_ip', os.environ.get('AIRSIM_IP', '127.0.0.1')))
        self.llm_url = str(p('llm_url', 'http://127.0.0.1:11434')).rstrip('/')
        self.llm_model = str(p('llm_model', 'gemma4:12b'))
        self.vlm = bool(p('vlm', True))
        ox, oy = (float(v) for v in str(p('origin_xy', '0,0')).split(','))
        self.origin = (ox, oy)
        places = str(p('places', ''))
        self.places = json.load(open(places))['places'] if places and os.path.exists(places) else {}
        self.log_dir = str(p('log_dir', '.'))
        self.photo_dir = os.path.join(self.log_dir, 'search')
        os.makedirs(self.photo_dir, exist_ok=True)

        self.reply_pub = self.create_publisher(String, '/llm/reply', 10)
        self.pause_pub = self.create_publisher(Bool, '/flight_driver/pause', 10)
        self.hold_pub = self.create_publisher(PointStamped, '/flight_driver/hold_at', 10)
        self.det_pub = self.create_publisher(String, '/search/detections', 10)
        self.create_subscription(Bool, '/search/enable', self.on_enable, 10)
        self.create_subscription(String, '/flight_driver/state', self.on_state, 10)

        self.enabled = False
        self.phase = ''
        self.cands = []           # [{'xy', 'hits': [t...], 'best'}]
        self.found = []           # reported people
        self.false_alarms = []
        self.frames = 0
        self.t_frames = []
        self.det = None
        self.log = open(os.path.join(self.log_dir, 'person_search.jsonl'), 'a')
        threading.Thread(target=self.loop, daemon=True).start()

    # ---- io ----
    def say(self, text):
        self.get_logger().info(text)
        self.reply_pub.publish(String(data=text))

    def record(self, kind, **kw):
        self.log.write(json.dumps({'t': time.time(), 'kind': kind, **kw}, ensure_ascii=False) + '\n')
        self.log.flush()

    def on_enable(self, m):
        if bool(m.data) != self.enabled:
            self.enabled = bool(m.data)
            self.record('enable', on=self.enabled)
            if self.enabled:
                self.say(f'수색 시작: 카메라 {self.camera}, 감지 {os.path.basename(self.model)}')

    def on_state(self, m):
        ph = re.search(r'phase=(\S+)', m.data)
        phase = ph.group(1) if ph else ''
        if phase == 'DONE' and self.phase != 'DONE':
            self.finish()
        self.phase = phase

    # ---- camera ----
    def capture(self):
        rpc = Rpc(self.airsim_ip, timeout=10.0)
        try:
            req = {'camera_name': self.camera, 'image_type': 0, 'pixels_as_float': False, 'compress': True}
            res = rpc.call('simGetImages', [req], '')[0]
        finally:
            rpc.close()
        data = res.get('image_data_uint8')
        img = cv2.imdecode(np.frombuffer(bytes(data), np.uint8), cv2.IMREAD_COLOR) if data else None
        if img is None:
            raise RuntimeError(f'no image from camera {self.camera!r}')
        cp, cq = res['camera_position'], res['camera_orientation']
        pos = np.array([cp['x_val'], cp['y_val'], cp['z_val']], dtype=float)
        q = np.array([cq['w_val'], cq['x_val'], cq['y_val'], cq['z_val']], dtype=float)
        return img, pos, q

    def people_in(self, img, pos, q):
        h, w = img.shape[:2]
        out = []
        for x1, y1, x2, y2, s, rot in self.det.detect_multi(img):
            # upright: the feet touch the ground; found turned (lying): the middle of the body does
            v = y2 if rot == 0 else (y1 + y2) / 2.0
            g = ground_point((x1 + x2) / 2.0, v, w, h, self.fov, pos, q)
            if g is not None and np.hypot(g[0] - pos[0], g[1] - pos[1]) < 60.0:
                out.append({'box': (x1, y1, x2, y2), 'score': s, 'rot': rot, 'xy': (float(g[0]), float(g[1]))})
        return out

    # ---- main loop ----
    def loop(self):
        while rclpy.ok():
            if not (self.enabled and self.phase == 'FOLLOW'):
                time.sleep(0.2)
                continue
            if self.det is None:
                try:
                    self.det = PersonDetector(self.model, score_thr=self.score)
                    self.say(f'감지 모델 준비 ({self.det.w}x{self.det.h})')
                except Exception as e:
                    self.say(f'감지 모델을 못 읽었습니다 ({e}) - 수색 없이 비행만 합니다.')
                    self.enabled = False
                    continue
            t0 = time.time()
            try:
                img, pos, q = self.capture()
                hits = self.people_in(img, pos, q)
            except Exception as e:
                self.record('capture_error', error=str(e))
                time.sleep(1.0)
                continue
            self.frames += 1
            self.t_frames.append(time.time() - t0)
            for d in hits:
                self.det_pub.publish(String(data=json.dumps({'t': t0, 'xy': d['xy'], 'score': d['score']})))
            self.record('frame', n=len(hits), cam=pos.tolist(),
                        hits=[{'xy': d['xy'], 'score': round(d['score'], 3)} for d in hits], secs=round(time.time() - t0, 3))
            c = self.update(t0, hits, img, pos, q)
            if c is not None:
                self.inspect(c)
            time.sleep(max(0.0, self.period - (time.time() - t0)))

    def update(self, t, hits, img, pos, q):
        """candidates on the ground; one seen confirm_n times within confirm_s s, once with confirm_score or more,
        and not yet reported -> inspect it (with the frame it scored best in)"""
        self.cands = [c for c in self.cands if t - c['hits'][-1] < 10.0]
        for d in hits:
            if any(math.dist(d['xy'], f['xy']) < self.dedupe_m for f in self.found + self.false_alarms):
                continue
            c = next((c for c in self.cands if math.dist(d['xy'], c['xy']) < self.merge_m), None)
            if c is None:
                c = {'xy': d['xy'], 'hits': [], 'best': 0.0}
                self.cands.append(c)
            n = len(c['hits'])
            c['xy'] = ((c['xy'][0] * n + d['xy'][0]) / (n + 1), (c['xy'][1] * n + d['xy'][1]) / (n + 1))
            c['hits'].append(t)
            if d['score'] > c['best']:
                c['best'], c['frame'] = d['score'], (img, d, pos, q)
            recent = [h for h in c['hits'] if t - h <= self.confirm_s]
            if len(recent) >= self.confirm_n and c['best'] >= self.confirm_score:
                self.cands.remove(c)
                return c
        return None

    # ---- a closer look ----
    def pause(self, on):
        for _ in range(3):
            self.pause_pub.publish(Bool(data=on))
            time.sleep(0.05)

    def inspect(self, c):
        k = len(self.found) + 1
        self.pause(True)
        self.say(f'사람으로 보이는 것 발견 (시험장 {c["xy"][0]:.1f}, {c["xy"][1]:.1f}, 감지 {c["best"]:.2f}) - 멈춰서 확인합니다.')
        _, _, cam_pos, cam_q = c['frame']
        yaw = math.atan2(2 * (cam_q[0] * cam_q[3] + cam_q[1] * cam_q[2]), 1 - 2 * (cam_q[2] ** 2 + cam_q[3] ** 2))
        vx, vy = c['xy'][0] - self.view_dist * math.cos(yaw), c['xy'][1] - self.view_dist * math.sin(yaw)
        time.sleep(1.5)                              # braking
        hp = PointStamped()
        hp.header.frame_id = 'takeoff_enu'
        hp.point.x, hp.point.y = vx - self.origin[0], -(vy - self.origin[1])   # east, north from the take-off point
        for _ in range(3):
            self.hold_pub.publish(hp)
            time.sleep(0.05)
        time.sleep(4.0)                              # there and steady
        try:
            img, pos, q = self.capture()
            hits = self.people_in(img, pos, q)
        except Exception as e:
            self.record('capture_error', error=str(e))
            hits = []
        near = sorted(hits, key=lambda d: math.dist(d['xy'], c['xy']))
        if near and math.dist(near[0]['xy'], c['xy']) <= 4.0:
            d = near[0]
            self.record('seen_again', xy=d['xy'], score=d['score'])
        else:                                        # gone from view: judge the best frame of the pass
            img, d, pos, q = c['frame']
            self.record('not_seen_again', xy=c['xy'], best=c['best'])
        stamp = time.strftime('%H%M%S')
        full = img.copy()
        x1, y1, x2, y2 = (int(v) for v in d['box'])
        cv2.rectangle(full, (x1, y1), (x2, y2), (0, 0, 255), 3)
        cv2.putText(full, f'#{k} {d["score"]:.2f}', (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        f_full = os.path.join(self.photo_dir, f'person_{k}_{stamp}.jpg')
        cv2.imwrite(f_full, full)
        pw, ph = int((x2 - x1) * 0.6) + 20, int((y2 - y1) * 0.6) + 20
        crop = img[max(0, y1 - ph):min(img.shape[0], y2 + ph), max(0, x1 - pw):min(img.shape[1], x2 + pw)]
        f_crop = os.path.join(self.photo_dir, f'person_{k}_{stamp}_crop.jpg')
        cv2.imwrite(f_crop, crop)
        desc = self.describe(crop) if self.vlm else ''
        if desc.startswith('사람 아님'):
            self.false_alarms.append({'xy': d['xy'], 'why': desc, 'photo': f_full})
            self.record('false_alarm', xy=d['xy'], why=desc, photo=f_full)
            self.say(f'확인 결과 사람이 아닙니다 ({desc}) - 수색을 이어갑니다.')
            self.pause(False)
            return
        x, y = d['xy']
        north, east = -(y - self.origin[1]), x - self.origin[0]
        place, dist = self.nearest_place(x, y)
        person = {'id': k, 'xy': [round(x, 2), round(y, 2)], 'from_takeoff_ne': [round(north, 1), round(east, 1)],
                  'score': round(d['score'], 3), 'description': desc, 'photo': f_full, 'crop': f_crop,
                  'vehicle_ned': [round(v, 2) for v in pos], 't': time.time()}
        self.found.append(person)
        self.record('person', **person)
        self.say(f'사람 #{k} 확인: 시험장 ({x:.1f}, {y:.1f}) - 출발점에서 북 {north:+.0f} m, 동 {east:+.0f} m'
                 + (f', {place} 근처 {dist:.0f} m' if place else '') + f', 감지 {d["score"]:.2f}'
                 + (f'. {desc}' if desc else '') + f' (사진 {os.path.basename(f_full)}) - 수색을 이어갑니다.')
        self.pause(False)

    def describe(self, crop):
        ok, jpg = cv2.imencode('.jpg', crop, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            return ''
        body = {'model': self.llm_model, 'stream': False, 'think': False, 'options': {'temperature': 0},
                'messages': [{'role': 'user', 'images': [base64.b64encode(jpg.tobytes()).decode()],
                              'content': '드론 카메라가 위에서 찍은 사진입니다. 사진 속 사람이 서 있는지, 누워 있는지(쓰러져 '
                                         '있는지)와 옷 색 같은 특징을 한국어 한 문장으로 짧게 말하세요. 사람이 없으면 '
                                         '"사람 아님: (보이는 것)"이라고만 답하세요.'}]}
        try:
            t0 = time.time()
            req = urllib.request.Request(self.llm_url + '/api/chat', json.dumps(body).encode(),
                                         {'Content-Type': 'application/json'})
            r = json.load(urllib.request.urlopen(req, timeout=90))
            text = r['message']['content'].strip().replace('\n', ' ')
            self.record('vlm', text=text, seconds=round(time.time() - t0, 1))
            return text
        except Exception as e:
            self.record('vlm_error', error=str(e))
            return ''

    def nearest_place(self, x, y):
        best, bd = None, 1e9
        for n, v in self.places.items():
            xy = self.origin if v.get('take_off') else v.get('xy')
            if xy is None:
                continue
            d = math.dist((x, y), xy)
            if d < bd:
                best, bd = n, d
        return (best, bd) if best is not None and bd < 25.0 else (None, 0.0)

    def finish(self):
        res = {'people': self.found, 'false_alarms': self.false_alarms, 'frames': self.frames,
               'frame_s_median': round(float(np.median(self.t_frames)), 3) if self.t_frames else None,
               'camera': self.camera, 'model': os.path.basename(self.model)}
        with open(os.path.join(self.log_dir, 'search_results.json'), 'w') as f:
            json.dump(res, f, ensure_ascii=False, indent=2)
        if self.frames:
            self.say(f'수색 끝: 사람 {len(self.found)}명 확인, 오탐 {len(self.false_alarms)}건, 영상 {self.frames}장 '
                     f'(한 장 {res["frame_s_median"]:.2f} s).')


def main():
    rclpy.init()
    node = PersonSearch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
