#!/usr/bin/env python3
"""리프트 보드에 ROS 없이 직접 프레임을 보내는 벤치 스크립트 (lift_node 와 같은 프로토콜).

  ros2 run lift_node lift_set.py probe                    # 0x40 왕복으로 링크 확인
  ros2 run lift_node lift_set.py up --hold 1.0            # 1 초 상승 후 정지
  ros2 run lift_node lift_set.py down|extend|retract --hold 0.5
  ros2 run lift_node lift_set.py stop                     # 전부 정지 (리프트 2축 + 호이스트 4)
  ros2 run lift_node lift_set.py listen --seconds 5       # 수신 프레임만 덤프
  옵션: --port /dev/ttyTHS1 --baud 115200

구동 명령은 --hold 초 뒤 반드시 정지를 보낸다 (기본 1.0 s, 최대 10 s).
lift_node 가 같은 포트를 잡고 있으면 먼저 내려야 한다.
"""

import argparse
import fcntl
import os
import select
import sys
import termios
import time

DEVICE_ID = 0x3E
PROTO_VERTICAL = 0x10
PROTO_HORIZONTAL = 0x11
PROTO_HOIST = 0x20
PROTO_ARM_ERROR_READ = 0x40
MAX_PAYLOAD = 64

COMMANDS = {
    # name: (protocol, value)
    'up': (PROTO_VERTICAL, 1),
    'down': (PROTO_VERTICAL, 2),
    'extend': (PROTO_HORIZONTAL, 1),
    'retract': (PROTO_HORIZONTAL, 2),
}


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def make_frame(protocol: int, payload: bytes) -> bytes:
    frame = bytes([DEVICE_ID, protocol, len(payload)]) + payload
    return frame + bytes([crc8(frame)])


def stop_all_frames() -> bytes:
    out = make_frame(PROTO_VERTICAL, b'\x00') + make_frame(PROTO_HORIZONTAL, b'\x00')
    for motor in range(4):
        out += make_frame(PROTO_HOIST, bytes([motor, 0]))
    return out


def open_serial(port: str, baud: int) -> int:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    speed = getattr(termios, 'B%d' % baud)
    iflag, oflag, cflag, lflag, _, _, cc = attrs
    iflag = 0
    oflag = 0
    lflag = 0
    cflag = termios.CLOCAL | termios.CREAD | termios.CS8
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, speed, speed, cc])
    termios.tcflush(fd, termios.TCIOFLUSH)
    flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
    return fd


def hexs(data: bytes) -> str:
    return ' '.join('%02X' % b for b in data)


def send(fd: int, data: bytes) -> None:
    os.write(fd, data)
    # 프레임 단위로 로그
    offset = 0
    while offset + 4 <= len(data):
        length = data[offset + 2]
        frame = data[offset:offset + 4 + length]
        print('TX', hexs(frame))
        offset += 4 + length


def listen(fd: int, seconds: float, want_probe: bool = False) -> bool:
    """seconds 동안 프레임을 수신해 출력. probe 응답을 보면 True."""
    buf = bytearray()
    deadline = time.monotonic() + seconds
    got = False
    while time.monotonic() < deadline:
        ready, _, _ = select.select([fd], [], [], 0.05)
        if not ready:
            continue
        try:
            chunk = os.read(fd, 256)
        except BlockingIOError:
            continue
        if not chunk:
            continue
        buf += chunk
        while True:
            start = buf.find(bytes([DEVICE_ID]))
            if start < 0:
                buf.clear()
                break
            del buf[:start]
            if len(buf) < 4:
                break
            length = buf[2]
            if length > MAX_PAYLOAD:
                del buf[0]
                continue
            total = length + 4
            if len(buf) < total:
                break
            frame = bytes(buf[:total])
            if crc8(frame[:-1]) != frame[-1]:
                del buf[0]
                continue
            del buf[:total]
            proto, payload = frame[1], frame[3:-1]
            print('RX', hexs(frame))
            if proto == PROTO_ARM_ERROR_READ and payload == b'\x00':
                got = True
            if proto in (0x41, 0x51) and len(payload) == 2:
                print('   arm motor %d error state %d' % (payload[0], payload[1]))
                got = True
            if want_probe and got:
                return True
    return got


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', choices=['probe', 'stop', 'listen'] + list(COMMANDS))
    parser.add_argument('--port', default=os.environ.get('LIFT_SERIAL_PORT', '/dev/ttyTHS1'))
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--hold', type=float, default=1.0, help='구동 유지 시간 [s] (최대 10)')
    parser.add_argument('--seconds', type=float, default=5.0, help='listen/probe 대기 시간 [s]')
    args = parser.parse_args()

    try:
        fd = open_serial(args.port, args.baud)
    except PermissionError:
        print('%s: permission denied — sudo usermod -aG dialout $USER 후 재로그인' % args.port,
              file=sys.stderr)
        return 1
    except OSError as exc:
        print('%s: %s' % (args.port, exc), file=sys.stderr)
        return 1

    try:
        if args.command == 'listen':
            listen(fd, args.seconds)
            return 0
        if args.command == 'stop':
            send(fd, stop_all_frames())
            time.sleep(0.075)
            send(fd, stop_all_frames())
            listen(fd, 0.3)
            return 0
        if args.command == 'probe':
            send(fd, make_frame(PROTO_ARM_ERROR_READ, b'\x00'))
            if listen(fd, args.seconds, want_probe=True):
                print('probe OK: 보드 응답 수신')
                return 0
            print('probe FAIL: %.1f s 안에 응답 없음' % args.seconds)
            return 2

        proto, value = COMMANDS[args.command]
        hold = max(0.0, min(10.0, args.hold))
        send(fd, make_frame(proto, bytes([value])))
        try:
            listen(fd, hold)
        finally:
            for _ in range(3):
                send(fd, make_frame(proto, b'\x00'))
                time.sleep(0.075)
            listen(fd, 0.2)
        return 0
    finally:
        try:
            termios.tcdrain(fd)
        except OSError:
            pass
        os.close(fd)


if __name__ == '__main__':
    sys.exit(main())
