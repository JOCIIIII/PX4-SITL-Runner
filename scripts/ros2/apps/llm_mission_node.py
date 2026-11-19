#!/usr/bin/env python3
"""Mission LLM: a Korean command -> a checked mission for flight_driver's legs (noclimb_ca.sh with LLM_MISSION=1).

The LLM only interprets: it picks places from a fixed table (llm_places_*.json), the order, photos and the return;
everything that flies stays where it was - the policy avoids, PathFollowing follows, flight_driver arms, lands and
guards. Flow:

  /llm/command (String, the operator's words)  -> the LLM (Ollama, JSON schema) -> checks -> /llm/reply (the plan)
  /llm/command "확인" / "go"                    -> /flight_driver/mission (legs), /flight_driver/go
  /llm/command "취소"                           -> the plan is dropped
  /llm/command "착륙" / "멈춰" (any time)        -> /flight_driver/land (lands in place - no LLM in that path)
  /flight_driver/leg "arrived i/N"             -> a photo there when the plan asks for one (AirSim camera)
  /flight_driver/state                         -> progress lines on /llm/reply
  task=search (an area of the table's 'areas'): a lawnmower over it (lanes search_lane_m apart, pass-through points,
  then home), /search/enable for person_search_node.py, which pauses the flight to look at what it finds

Legs are metres from the take-off point (north = -(Y - Y0), east = X - X0 in the test map layout frame, as
scenario_super.sh's goal). One mission per session: flight_driver arms once.

ROS parameters: llm_url (Ollama, default http://127.0.0.1:11434; set LLM_URL for a remote one), llm_model (gemma4:12b), places (json),
origin_xy "X,Y" (the take-off point in the places frame), flight_alt (m above ground, fixed for the session),
v_max (the policy's top speed), photo_hold_s (5), photo_camera (front_left), airsim_ip, log_dir.
"""
import json
import math
import os
import re
import sys
import threading
import time
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
try:
    from airsim_rpc import capture_png
except ImportError:                      # the camera is optional
    capture_png = None

STOP_WORDS = ('착륙', '멈춰', '정지', '중지', '그만', 'stop', 'land', 'abort')
CONFIRM_WORDS = ('확인', '좋아', '출발', '진행', 'go', 'yes', 'ok')
CANCEL_WORDS = ('취소', '아니', 'cancel', 'no')


class LlmMission(Node):
    def __init__(self):
        super().__init__('llm_mission')
        p = lambda n, d: self.declare_parameter(n, d).value
        self.url = str(p('llm_url', 'http://127.0.0.1:11434')).rstrip('/')
        self.model = str(p('llm_model', 'gemma4:12b'))
        places_file = str(p('places', ''))
        ox, oy = (float(v) for v in str(p('origin_xy', '0,0')).split(','))
        self.origin = (ox, oy)
        self.alt = float(p('flight_alt', 1.7))
        self.v_max = float(p('v_max', 3.0))
        self.photo_hold = float(p('photo_hold_s', 5.0))
        self.airsim_ip = str(p('airsim_ip', os.environ.get('AIRSIM_IP', '127.0.0.1')))
        # the AirSim pawn's front_center camera sits inside the vehicle's body (the image is its own frame):
        # front_left looks ahead past it
        self.photo_camera = str(p('photo_camera', 'front_left'))
        self.log_dir = str(p('log_dir', '.'))
        self.max_radius = float(p('max_radius_m', 150.0))
        data = json.load(open(places_file))
        self.places = data['places']
        self.names = list(self.places)
        self.home = next((n for n, v in self.places.items() if v.get('take_off')), None)
        self.areas = data.get('areas', {})
        self.lane = float(p('search_lane_m', 10.0))
        self.search_speed = float(p('search_speed', 2.0))

        self.reply_pub = self.create_publisher(String, '/llm/reply', 10)
        self.mission_pub = self.create_publisher(String, '/flight_driver/mission', 10)
        self.go_pub = self.create_publisher(Bool, '/flight_driver/go', 10)
        self.land_pub = self.create_publisher(Bool, '/flight_driver/land', 10)
        self.cap_pub = self.create_publisher(Float32, '/noclimb/speed_cap', 10)
        self.search_pub = self.create_publisher(Bool, '/search/enable', 10)
        self.create_subscription(String, '/llm/command', self.on_command, 10)
        self.create_subscription(String, '/flight_driver/state', self.on_state, 10)
        self.create_subscription(String, '/flight_driver/leg', self.on_leg, 10)

        self.plan = None             # the checked plan waiting for the operator's go
        self.flying = None           # the plan being flown
        self.busy = False            # an LLM call is running
        self.phase = ''
        self.state_line = ''
        self.go_until = 0.0
        self.log = open(os.path.join(self.log_dir, 'llm_mission.jsonl'), 'a')
        self.create_timer(0.5, self.tick)
        self.say(f'임무 LLM 준비: 모델 {self.model}, 장소 {len(self.names)}곳 ({", ".join(self.names)}), '
                 f'이번 비행 고도 {self.alt:.1f} m, 최대 {self.v_max:.1f} m/s. 명령을 입력하세요.')

    # ---- output ----
    def say(self, text):
        self.get_logger().info(text)
        self.reply_pub.publish(String(data=text))
        self.record('reply', text=text)

    def record(self, kind, **kw):
        self.log.write(json.dumps({'t': time.time(), 'kind': kind, **kw}, ensure_ascii=False) + '\n')
        self.log.flush()

    # ---- operator ----
    def on_command(self, m):
        text = m.data.strip()
        if not text:
            return
        self.record('command', text=text)
        low = text.lower()
        if any(w in low for w in STOP_WORDS) and self.flying is not None:
            self.land_pub.publish(Bool(data=True))
            self.say('착륙 명령을 보냈습니다 - 그 자리에 착륙합니다.')
            return
        if self.plan is not None and any(low.startswith(w) or low == w for w in CONFIRM_WORDS):
            self.start(self.plan)
            return
        if self.plan is not None and any(low.startswith(w) for w in CANCEL_WORDS):
            self.plan = None
            self.say('계획을 취소했습니다.')
            return
        if self.flying is not None:
            self.say('비행 중에는 새 임무를 받지 않습니다 ("착륙"만 가능).')
            return
        if self.busy:
            self.say('앞 명령을 해석하는 중입니다.')
            return
        self.busy = True
        threading.Thread(target=self.interpret, args=(text,), daemon=True).start()

    # ---- LLM ----
    def schema(self):
        return {'type': 'object', 'properties': {
            'valid': {'type': 'boolean'},
            'reason': {'type': 'string'},
            'task': {'type': 'string', 'enum': ['goto', 'search']},
            'area': {'type': 'string', 'enum': list(self.areas) + ['none']},
            'legs': {'type': 'array', 'items': {'type': 'object', 'properties': {
                'place': {'type': 'string', 'enum': self.names},
                'photo': {'type': 'boolean'}}, 'required': ['place', 'photo']}},
            'return_home': {'type': 'boolean'},
            'speed_mps': {'type': 'number'},
            'altitude_m': {'type': 'number'},
            'summary': {'type': 'string'}},
            'required': ['valid', 'reason', 'task', 'area', 'legs', 'return_home', 'speed_mps', 'altitude_m', 'summary']}

    def system_prompt(self):
        table = '\n'.join(f'- {n}: {v.get("desc", "")}' for n, v in self.places.items())
        return (
            '너는 드론 임무 해석기다. 운용자의 한국어 명령을 비행 계획 JSON 하나로 바꾼다.\n'
            f'장소는 반드시 아래 목록의 이름 그대로만 쓴다:\n{table}\n'
            '규칙:\n'
            '- legs: 들를 곳을 순서대로. 각 곳에서 사진을 찍으라고 하면 photo=true.\n'
            '- "돌아와", "복귀"처럼 마지막에 이륙한 곳으로 오라는 말이 있으면 return_home=true (legs 에 출발점을 넣지 않는다).\n'
            '- "출발점으로 가"처럼 출발점 자체가 목적지면 legs=[출발점].\n'
            f'- 속도를 말하지 않으면 3, "천천히" 1, "빨리" {self.v_max:g}. 고도를 말하지 않으면 {self.alt:g}.\n'
            '- 목록에 없는 장소, 5곳 넘게 들르기, 이해할 수 없거나 위험한 명령은 valid=false, legs=[] 로 하고 reason 에 이유를 쓴다.\n'
            '- summary: 운용자에게 보여 줄 짧은 한 문장.\n'
            '- 사람을 찾으라는 명령(수색, 실종자, 사람 찾아)은 task=search, area 에 아래 수색 구역 이름, legs=[] (경로는 코드가 만든다). '
            '구역을 말하지 않으면 "' + (next(iter(self.areas)) if self.areas else 'none') + '". 그 밖의 명령은 task=goto, area=none.\n'
            '수색 구역:\n' + '\n'.join(f'- {n}: {v.get("desc", "")}' for n, v in self.areas.items()))

    def interpret(self, text):
        try:
            body = {'model': self.model, 'stream': False, 'think': False, 'format': self.schema(),
                    'options': {'temperature': 0},
                    'messages': [{'role': 'system', 'content': self.system_prompt()},
                                 {'role': 'user', 'content': text}]}
            req = urllib.request.Request(self.url + '/api/chat', json.dumps(body).encode(),
                                         {'Content-Type': 'application/json'})
            t0 = time.time()
            r = json.load(urllib.request.urlopen(req, timeout=120))
            out = json.loads(r['message']['content'])
            self.record('llm', text=text, out=out, seconds=round(time.time() - t0, 1))
            self.check(text, out, time.time() - t0)
        except Exception as e:      # the LLM is down / unreachable: say so, never guess a mission
            self.say(f'LLM 호출 실패 ({type(e).__name__}: {e}) - 명령을 해석하지 못했습니다.')
        finally:
            self.busy = False

    # ---- checks (plain code, after the LLM) ----
    def xy(self, name):
        v = self.places[name]
        return self.origin if v.get('take_off') else tuple(v['xy'])

    def check(self, text, out, secs):
        if out.get('valid') and out.get('task') == 'search':
            self.check_search(text, out, secs)
            return
        if not out.get('valid') or not out.get('legs'):
            self.say(f'수행할 수 없는 명령입니다: {out.get("reason") or "이유 없음"} ({secs:.1f} s)')
            return
        legs = [dict(l) for l in out['legs']]
        if out.get('return_home') and self.home and legs[-1]['place'] != self.home:
            legs.append({'place': self.home, 'photo': False})
        notes = []
        speed = float(out.get('speed_mps') or 3.0)
        if speed > self.v_max:
            notes.append(f'속도 {speed:g} m/s 는 최대 {self.v_max:g} 로 낮춤')
            speed = self.v_max
        if speed < 0.5:
            notes.append(f'속도 {speed:g} m/s 는 0.5 로 올림')
            speed = 0.5
        alt = float(out.get('altitude_m') or self.alt)
        if abs(alt - self.alt) > 0.05:
            notes.append(f'고도 {alt:g} m 요청 - 이번 비행은 {self.alt:g} m 고정')
        if len(legs) > 6:
            self.say('들를 곳이 너무 많습니다 (최대 5곳 + 복귀).')
            return
        x0, y0 = self.origin
        prev = (x0, y0)
        parts = []
        for i, l in enumerate(legs):
            x, y = self.xy(l['place'])
            north, east = -(y - y0), x - x0
            if math.hypot(east, north) > self.max_radius:
                self.say(f'{l["place"]} 는 이륙 지점에서 {math.hypot(east, north):.0f} m 로 한계({self.max_radius:.0f} m) 밖입니다.')
                return
            if math.hypot(x - prev[0], y - prev[1]) < 5.0:
                self.say(f'{l["place"]} 가 바로 앞 지점과 5 m 안이라 구간이 될 수 없습니다.')
                return
            l.update(north=round(north, 2), east=round(east, 2), hold=self.photo_hold if l.get('photo') else 0.0)
            prev = (x, y)
            parts.append(l)
        mission = ';'.join(f'{l["north"]:.2f},{l["east"]:.2f},{l["hold"]:.1f}' for l in parts)
        route = ' → '.join(l['place'] + (' (사진)' if l.get('photo') else '') for l in parts)
        self.plan = {'text': text, 'legs': parts, 'speed': speed, 'mission': mission, 'summary': out.get('summary', '')}
        self.record('plan', plan=self.plan, notes=notes)
        self.say(f'계획: 출발점 → {route} → 착륙, {speed:g} m/s, 고도 {self.alt:g} m'
                 + (f' ({"; ".join(notes)})' if notes else '') + f'. 해석 {secs:.1f} s.'
                 + ' 비행하려면 "확인", 그만두려면 "취소".')

    def lawnmower(self, poly):
        """pass-through points over an axis-aligned area: lanes along the long side, search_lane_m apart, starting
        at the corner nearest to the take-off point"""
        xs, ys = [q[0] for q in poly], [q[1] for q in poly]
        x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
        along_x = (x1 - x0) >= (y1 - y0)
        a0, a1, b0, b1 = (x0, x1, y0, y1) if along_x else (y0, y1, x0, x1)
        n = max(1, int(math.ceil((b1 - b0) / self.lane)))
        lanes = [b0 + (i + 0.5) * (b1 - b0) / n for i in range(n)]
        ox, oy = self.origin
        oa, ob = (ox, oy) if along_x else (oy, ox)
        if abs(ob - lanes[-1]) < abs(ob - lanes[0]):
            lanes.reverse()
        start_hi = abs(oa - a1) < abs(oa - a0)
        pts = []
        for i, b in enumerate(lanes):
            ends = (a1, a0) if (i % 2 == 0) == start_hi else (a0, a1)
            for a in ends:
                pts.append((a, b) if along_x else (b, a))
        return pts

    def check_search(self, text, out, secs):
        area = out.get('area')
        if area not in self.areas:
            self.say(f'수색 구역을 모르겠습니다 ({area}). 구역: {", ".join(self.areas)}')
            return
        pts = self.lawnmower(self.areas[area]['polygon'])
        x0, y0 = self.origin
        legs, prev, length = [], (x0, y0), 0.0
        for x, y in pts:
            if math.hypot(x - prev[0], y - prev[1]) < 5.0:
                continue
            north, east = -(y - y0), x - x0
            if math.hypot(east, north) > self.max_radius:
                self.say(f'{area} 는 이륙 지점에서 {math.hypot(east, north):.0f} m 로 한계({self.max_radius:.0f} m) 밖입니다.')
                return
            length += math.hypot(x - prev[0], y - prev[1])
            legs.append({'place': f'수색점 {len(legs) + 1}', 'photo': False, 'north': round(north, 2),
                         'east': round(east, 2), 'hold': -1.0})
            prev = (x, y)
        length += math.hypot(prev[0] - x0, prev[1] - y0)
        legs.append({'place': self.home or '출발점', 'photo': False, 'north': 0.0, 'east': 0.0, 'hold': 0.0})
        speed = min(float(out.get('speed_mps') or self.search_speed), self.search_speed, self.v_max)
        mission = ';'.join(f'{l["north"]:.2f},{l["east"]:.2f},{l["hold"]:.1f}' for l in legs)
        self.plan = {'text': text, 'legs': legs, 'speed': speed, 'mission': mission, 'search': area,
                     'summary': out.get('summary', '')}
        self.record('plan', plan=self.plan)
        lanes = (len(legs) - 1) // 2
        self.say(f'수색 계획: {area} 를 {lanes}줄로 지그재그(간격 {self.lane:g} m, {len(legs) - 1}점), 고도 {self.alt:g} m, '
                 f'{speed:g} m/s, 약 {length:.0f} m / {length / speed / 60:.1f}분. 사람을 찾으면 멈춰서 확인하고 보고한 뒤 '
                 f'이어갑니다. 끝나면 출발점에 착륙. 해석 {secs:.1f} s. 비행하려면 "확인", 그만두려면 "취소".')

    # ---- flying ----
    def start(self, plan):
        self.plan = None
        self.flying = plan
        self.cap_pub.publish(Float32(data=float(plan['speed'])))
        self.mission_pub.publish(String(data=plan['mission']))
        if plan.get('search'):
            self.search_pub.publish(Bool(data=True))
        self.go_until = time.time() + 30.0            # go once flight_driver is READY again with this mission
        self.record('start', plan=plan)
        self.say(f'임무를 비행 드라이버에 보냈습니다 ({len(plan["legs"])}구간). 준비되면 이륙합니다.')

    def tick(self):
        if self.flying is not None and self.flying.get('search') and int(time.time() * 2) % 4 == 0:
            self.search_pub.publish(Bool(data=True))
        if self.flying is not None and time.time() < self.go_until:
            self.cap_pub.publish(Float32(data=float(self.flying['speed'])))
            if 'WAIT_GO' in self.phase:
                self.go_pub.publish(Bool(data=True))
            elif self.phase in ('PREFLIGHT', '') and int(time.time() * 2) % 6 == 0:
                self.mission_pub.publish(String(data=self.flying['mission']))   # in case the first one was lost

    def on_state(self, m):
        self.state_line = m.data
        ph = re.search(r'phase=(\S+)', m.data)
        phase = ph.group(1) if ph else ''
        if phase != self.phase:
            if self.flying is not None and phase in ('TAKEOFF', 'LANDING', 'DONE', 'YIELDED'):
                why = re.search(r'reason="([^"]*)"', m.data)
                names = {'TAKEOFF': '이륙', 'LANDING': '착륙 시작', 'DONE': '비행 끝', 'YIELDED': '조종사에게 넘김'}
                self.say(f'{names[phase]}' + (f' ({why.group(1)})' if why and phase != 'TAKEOFF' else ''))
            if phase == 'DONE':
                if self.flying is not None and self.flying.get('search'):
                    self.search_pub.publish(Bool(data=False))
                self.flying = None
            self.phase = phase

    def on_leg(self, m):
        if self.flying is None:
            return
        g = re.match(r'arrived (\d+)/(\d+)', m.data)
        if not g:
            return
        i = int(g.group(1)) - 1
        legs = self.flying['legs']
        if i >= len(legs):
            return
        leg = legs[i]
        self.say(f'{leg["place"]} 도착 ({i + 1}/{len(legs)})')
        if leg.get('photo'):
            threading.Thread(target=self.photo, args=(i, leg['place']), daemon=True).start()

    def photo(self, i, place):
        time.sleep(1.0)                                 # settled at the hold
        if capture_png is None:
            self.say('사진: 카메라 모듈이 없습니다.')
            return
        try:
            png = capture_png(self.airsim_ip, camera=self.photo_camera)
            path = os.path.join(self.log_dir, f'photo_{i + 1}_{time.strftime("%H%M%S")}.png')
            with open(path, 'wb') as f:
                f.write(png)
            self.record('photo', leg=i + 1, place=place, path=path, bytes=len(png))
            self.say(f'사진 저장: {os.path.basename(path)} ({len(png) // 1024} KB, {place})')
        except Exception as e:
            self.record('photo_failed', leg=i + 1, place=place, error=str(e))
            self.say(f'사진 실패 ({place}): {e}')


def main():
    rclpy.init()
    node = LlmMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
