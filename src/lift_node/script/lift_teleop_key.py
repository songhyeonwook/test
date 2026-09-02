#!/usr/bin/env python3
"""키보드를 누르는 동안 리프트를 움직이고, 떼면 정지하는 텔레옵 (lift_node 용).

  ros2 run lift_node lift_teleop_key.py
    ↑ / w  상승 UP        ↓ / s  하강 DOWN
    → / d  전진 EXTEND    ← / a  후진 RETRACT
    space  전부 정지 (lift_node/stop)      q / Ctrl-C  정지 후 종료

터미널은 "키를 뗐다" 는 이벤트를 주지 않는다. 대신 키를 누르고 있으면 OS 자동반복으로
같은 문자가 계속 들어오므로, 그 흐름이 끊기는 것을 뗀 것으로 본다.
  - 첫 누름 뒤 자동반복이 시작되기까지는 OS 지연(보통 0.25~0.5 s) 이 있다. 그동안은
    --tap-timeout (0.6 s) 만큼 기다린다. 그 안에 떼면 최대 0.6 s 뒤에 선다.
  - 자동반복이 시작된 뒤에는 문자가 ~30 Hz 로 오므로 --release-timeout (0.15 s) 안에
    다음 문자가 없으면 바로 정지한다.
  누르는 동안은 --rate (20 Hz) 로 지령을 계속 발행한다 (lift_node 의 cmd_timeout_s 갱신).
  시작하자마자 리프트가 움찔거리면(반복 지연 > tap-timeout) --tap-timeout 을 늘리고,
  더 빨리 세우고 싶으면 OS 키 반복 지연을 줄인다 (X11: xset r rate 200 40).
"""

import argparse
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger

VERTICAL, HORIZONTAL = 0, 1
AXIS_TOPIC = {VERTICAL: 'lift/vertical', HORIZONTAL: 'lift/horizontal'}
VALUE_NAME = {
    (VERTICAL, 1): 'UP', (VERTICAL, 2): 'DOWN',
    (HORIZONTAL, 1): 'EXTEND', (HORIZONTAL, 2): 'RETRACT',
}
# 키 -> (축, 값)
KEYMAP = {
    'w': (VERTICAL, 1), '\x1b[A': (VERTICAL, 1),      # ↑
    's': (VERTICAL, 2), '\x1b[B': (VERTICAL, 2),      # ↓
    'd': (HORIZONTAL, 1), '\x1b[C': (HORIZONTAL, 1),  # →
    'a': (HORIZONTAL, 2), '\x1b[D': (HORIZONTAL, 2),  # ←
}
STOP_KEYS = (' ',)
QUIT_KEYS = ('q', '\x03', '\x04')  # q, Ctrl-C, Ctrl-D


def parse_keys(data: str):
    """stdin 바이트를 키 단위로 자른다. 화살표는 ESC [ A~D 시퀀스."""
    keys = []
    i = 0
    while i < len(data):
        ch = data[i]
        if ch == '\x1b' and i + 2 < len(data) and data[i + 1] == '[' and data[i + 2] in 'ABCD':
            keys.append(data[i:i + 3])
            i += 3
        elif ch == '\x1b' and i + 2 < len(data) and data[i + 1] == 'O' and data[i + 2] in 'ABCD':
            keys.append('\x1b[' + data[i + 2])  # 일부 터미널의 application 모드
            i += 3
        else:
            keys.append(ch.lower() if ch.isalpha() else ch)
            i += 1
    return keys


class LiftTeleop(Node):
    def __init__(self, args):
        super().__init__('lift_teleop_key')
        self.args = args
        self.pub = {axis: self.create_publisher(Int32, topic, 10) for axis, topic in AXIS_TOPIC.items()}
        self.stop_client = self.create_client(Trigger, 'lift_node/stop')
        self.create_subscription(String, 'lift_node/status', self._status_cb,
                                 rclpy.qos.QoSProfile(depth=1,
                                                      reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                                                      durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
        self.node_status = '?'
        # 축별 상태
        self.value = {VERTICAL: 0, HORIZONTAL: 0}
        self.last_key = {VERTICAL: 0.0, HORIZONTAL: 0.0}
        self.repeating = {VERTICAL: False, HORIZONTAL: False}
        self.key_count = {VERTICAL: 0, HORIZONTAL: 0}
        self.last_line = ''

    def _status_cb(self, msg):
        self.node_status = msg.data
        self.show()

    def publish(self, axis, value, times=1):
        msg = Int32()
        msg.data = value
        for _ in range(times):
            self.pub[axis].publish(msg)

    def press(self, axis, value, now):
        if self.value[axis] == value:
            # 같은 키 반복: 첫 누름 뒤 tap-timeout 안에 또 오면 자동반복 시작으로 본다
            self.key_count[axis] += 1
            if self.key_count[axis] >= 2:
                self.repeating[axis] = True
        else:
            self.value[axis] = value
            self.key_count[axis] = 1
            self.repeating[axis] = False
            self.publish(axis, value)
        self.last_key[axis] = now
        self.show()

    def release(self, axis):
        if self.value[axis] != 0:
            self.value[axis] = 0
            self.repeating[axis] = False
            self.key_count[axis] = 0
            self.publish(axis, 0, times=2)
            self.show()

    def stop_all(self):
        for axis in (VERTICAL, HORIZONTAL):
            self.value[axis] = 0
            self.repeating[axis] = False
            self.key_count[axis] = 0
            self.publish(axis, 0, times=2)
        if self.stop_client.service_is_ready():
            self.stop_client.call_async(Trigger.Request())
        self.show('STOP ALL')

    def tick(self, now):
        for axis in (VERTICAL, HORIZONTAL):
            if self.value[axis] == 0:
                continue
            timeout = self.args.release_timeout if self.repeating[axis] else self.args.tap_timeout
            if now - self.last_key[axis] > timeout:
                self.release(axis)
            else:
                self.publish(axis, self.value[axis])  # 누르는 동안 계속 발행 (워치독 갱신)

    def show(self, note=''):
        v = VALUE_NAME.get((VERTICAL, self.value[VERTICAL]), '-')
        h = VALUE_NAME.get((HORIZONTAL, self.value[HORIZONTAL]), '-')
        line = 'vertical %-7s horizontal %-7s | lift_node %s %s' % (v, h, self.node_status, note)
        if line != self.last_line:
            sys.stdout.write('\r\x1b[K' + line)
            sys.stdout.flush()
            self.last_line = line


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--tap-timeout', type=float, default=0.6,
                        help='첫 누름 뒤 자동반복을 기다리는 시간 [s] (OS 반복 지연보다 길게)')
    parser.add_argument('--release-timeout', type=float, default=0.15,
                        help='자동반복 중 문자가 끊기면 정지까지 [s]')
    parser.add_argument('--rate', type=float, default=20.0, help='누르는 동안 발행 주기 [Hz]')
    args, ros_args = parser.parse_known_args()

    if not sys.stdin.isatty():
        print('stdin 이 터미널이 아닙니다', file=sys.stderr)
        return 1

    rclpy.init(args=ros_args)
    node = LiftTeleop(args)
    print(__doc__.split('\n\n')[1])
    print('tap-timeout %.2fs, release-timeout %.2fs, %.0f Hz' %
          (args.tap_timeout, args.release_timeout, args.rate))

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    # Ctrl-C 를 문자로 받아 정지 후 종료하도록 ISIG 를 끈다
    attrs = termios.tcgetattr(fd)
    attrs[3] &= ~(termios.ISIG | termios.ECHO)
    termios.tcsetattr(fd, termios.TCSANOW, attrs)

    period = 1.0 / max(1.0, args.rate)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            ready, _, _ = select.select([fd], [], [], period)
            now = time.monotonic()
            if ready:
                data = os.read(fd, 64).decode('utf-8', 'ignore')
                for key in parse_keys(data):
                    if key in QUIT_KEYS:
                        node.stop_all()
                        return 0
                    if key in STOP_KEYS:
                        node.stop_all()
                        continue
                    if key in KEYMAP:
                        axis, value = KEYMAP[key]
                        node.press(axis, value, now)
            node.tick(time.monotonic())
        return 0
    finally:
        try:
            node.stop_all()
            rclpy.spin_once(node, timeout_sec=0.1)
        except Exception:
            pass
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        sys.stdout.write('\n')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
