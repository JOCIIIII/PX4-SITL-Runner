#!/usr/bin/env python3
"""Operator console for the mission LLM (scripts/ros2/apps/llm_mission_node.py): type a Korean command, read the
plan, answer "확인" to fly or "취소"; "착륙" lands in place at any time. Ctrl+D / Ctrl+C quits (the flight goes on).
Runs where the apps run (SILS: docker exec -it ros2-env, PILS: the onboard container) - scripts/ctrack.sh llm.
"""
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Console(Node):
    def __init__(self):
        super().__init__('llm_console')
        self.pub = self.create_publisher(String, '/llm/command', 10)
        self.create_subscription(String, '/llm/reply', lambda m: print(f'\n[LLM] {m.data}\n> ', end='', flush=True), 10)


def main():
    rclpy.init()
    node = Console()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    print('명령을 입력하세요 (예: "골까지 가서 사진 찍고 돌아와"). "확인" 비행 / "취소" / "착륙". 끝내려면 Ctrl+D.')
    try:
        while True:
            line = input('> ').strip()
            if line:
                node.pub.publish(String(data=line))
    except (EOFError, KeyboardInterrupt):
        print()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
