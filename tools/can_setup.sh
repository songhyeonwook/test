#!/usr/bin/env bash
# Jetson 내장 CAN(mttcan) 으로 can0 를 올린다. hw motor_node / drive_set.py 용.
#
#   sudo tools/can_setup.sh up            # 모듈 로드 + can0 설정 + up
#   sudo tools/can_setup.sh down
#        tools/can_setup.sh status        # 링크 상태, 카운터, 버스 수신 테스트
#   sudo tools/can_setup.sh install       # 부팅 시 자동 실행 (can0_setup.service)
#   sudo tools/can_setup.sh selftest      # 루프백 자체진단 (버스 케이블 분리하고)
#        tools/can_setup.sh errors        # 에러프레임 종류 확인 (5초)
#   sudo tools/can_setup.sh listen        # listen-only 로 엿듣기 (ACK 안 보냄)
#
# 비트레이트: 환경변수 CAN_BITRATE 또는 --bitrate N. 기본 1000000 (drive_set.py 와 동일).
#   rbio TransferRobot 의 transfer-robot-can.service 는 250000 이다. 드라이버 설정과
#   맞아야 하며, 틀리면 `status` 의 bus-error / error-warning 카운터가 올라간다.
set -euo pipefail

IFACE="${CAN_IFACE:-can0}"
BITRATE="${CAN_BITRATE:-250000}"
RESTART_MS="${CAN_RESTART_MS:-100}"      # bus-off 자동 복구 지연
TXQUEUELEN="${CAN_TXQUEUELEN:-1000}"     # 4축 PDO 50 Hz 송신 큐
UNIT_NAME="can0_setup.service"
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"

ACTION="${1:-status}"
shift || true
while [[ $# -gt 0 ]]; do
  case "$1" in
    --bitrate) BITRATE="$2"; shift 2 ;;
    --iface)   IFACE="$2";   shift 2 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

need_root() {
  if [[ "$(id -u)" -ne 0 ]]; then
    echo "root 권한이 필요합니다: sudo $SCRIPT_PATH $ACTION" >&2
    exit 1
  fi
}

load_modules() {
  # Jetson: mttcan (Tegra MTT CAN 컨트롤러). 다른 보드는 해당 드라이버로 바꾼다.
  for m in can can_raw can_dev mttcan; do
    if ! lsmod | grep -q "^${m}\b"; then
      modprobe "$m" 2>/dev/null || echo "warn: modprobe $m 실패 (내장 커널이면 무시)" >&2
    fi
  done
  if [[ ! -e "/sys/class/net/${IFACE}" ]]; then
    echo "${IFACE} 장치가 없습니다. mttcan 드라이버/핀먹스(jetson-io) 를 확인하세요." >&2
    exit 1
  fi
}

can_up() {
  need_root
  load_modules
  ip link set "$IFACE" down 2>/dev/null || true
  ip link set "$IFACE" type can bitrate "$BITRATE" restart-ms "$RESTART_MS"
  ip link set "$IFACE" txqueuelen "$TXQUEUELEN"
  ip link set "$IFACE" up
  echo "${IFACE} up: bitrate=${BITRATE} restart-ms=${RESTART_MS} txqueuelen=${TXQUEUELEN}"
  ip -d link show "$IFACE" | sed -n 1,3p
}

can_down() {
  need_root
  ip link set "$IFACE" down 2>/dev/null && echo "${IFACE} down" || echo "${IFACE} 이미 down"
}

can_status() {
  if [[ ! -e "/sys/class/net/${IFACE}" ]]; then
    echo "${IFACE}: 장치 없음"; exit 1
  fi
  ip -d -s link show "$IFACE"
  echo
  local state
  state="$(cat "/sys/class/net/${IFACE}/operstate" 2>/dev/null || echo unknown)"
  echo "operstate: ${state}"
  if command -v candump >/dev/null 2>&1 && [[ "$state" == "up" ]]; then
    echo "2초간 버스 수신 (모터 드라이버가 켜져 있으면 TPDO 0x181~0x184 가 보여야 한다):"
    timeout 2 candump -n 8 "$IFACE" 2>/dev/null || echo "  (수신 없음)"
  else
    echo "candump 없음 또는 링크 down. sudo apt install can-utils"
  fi
}

# 컨트롤러 + 커널 경로만 검사한다. 트랜시버/배선은 보지 않으므로
# 버스 케이블을 분리하고 돌리는 것이 안전하다.
can_selftest() {
  need_root
  load_modules
  command -v cansend >/dev/null 2>&1 || { echo "can-utils 필요: sudo apt install can-utils" >&2; exit 1; }
  ip link set "$IFACE" down 2>/dev/null || true
  ip link set "$IFACE" type can bitrate "$BITRATE" loopback on
  ip link set "$IFACE" up
  echo "loopback on (${BITRATE} bps). 자기 프레임이 되돌아오면 컨트롤러는 정상이다."
  local out
  out="$( { timeout 2 candump -n 1 "$IFACE" & sleep 0.3; cansend "$IFACE" 123#DEADBEEF; wait; } 2>&1 || true )"
  echo "${out:-  (되돌아온 프레임 없음)}"
  ip link set "$IFACE" down
  ip link set "$IFACE" type can bitrate "$BITRATE" loopback off restart-ms "$RESTART_MS"
  ip link set "$IFACE" up
  echo "loopback off, ${IFACE} 정상 모드로 복귀"
}

# 에러프레임의 "종류" 를 본다. 원인 분류에 가장 유용하다.
#   stuff/form/crc  -> 비트레이트 불일치, 노이즈, 종단 저항
#   bit0/bit1       -> 배선/트랜시버 (도미넌트 고착 등)
#   ack             -> 우리는 보내는데 아무도 응답 안 함 (상대 없음/속도 불일치)
can_errors() {
  command -v candump >/dev/null 2>&1 || { echo "can-utils 필요: sudo apt install can-utils" >&2; exit 1; }
  echo "5초간 에러프레임 수집 (${IFACE}):"
  timeout 5 candump -e "${IFACE},#FFFFFFFF" 2>/dev/null || echo "  (에러프레임 없음)"
  echo
  echo "카운터 변화:"; ip -s -d link show "$IFACE" | grep -A2 "re-started"
}

# listen-only: ACK/에러프레임을 버스에 내보내지 않는다. 남의 통신을 방해하지 않고
# 엿들을 때, 그리고 우리 송신이 문제인지 가릴 때 쓴다.
can_listen() {
  need_root
  load_modules
  ip link set "$IFACE" down 2>/dev/null || true
  ip link set "$IFACE" type can bitrate "$BITRATE" listen-only on
  ip link set "$IFACE" up
  echo "listen-only on (${BITRATE} bps). Ctrl-C 로 종료 후 'up' 으로 복귀."
  candump "$IFACE" || true
}

can_install() {
  need_root
  cat > "/etc/systemd/system/${UNIT_NAME}" <<UNIT
[Unit]
Description=hw can0 setup (mttcan, ${BITRATE} bps)
After=sys-subsystem-net-devices-${IFACE}.device
Wants=sys-subsystem-net-devices-${IFACE}.device

[Service]
Type=oneshot
Environment=CAN_IFACE=${IFACE}
Environment=CAN_BITRATE=${BITRATE}
Environment=CAN_RESTART_MS=${RESTART_MS}
Environment=CAN_TXQUEUELEN=${TXQUEUELEN}
ExecStart=${SCRIPT_PATH} up
ExecStop=${SCRIPT_PATH} down
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
UNIT
  systemctl daemon-reload
  systemctl enable --now "${UNIT_NAME}"
  systemctl --no-pager status "${UNIT_NAME}" | sed -n 1,5p
  echo "설치됨: /etc/systemd/system/${UNIT_NAME} (bitrate ${BITRATE}). 변경은 다시 install."
}

case "$ACTION" in
  up)       can_up ;;
  down)     can_down ;;
  status)   can_status ;;
  install)  can_install ;;
  selftest) can_selftest ;;
  errors)   can_errors ;;
  listen)   can_listen ;;
  *) echo "usage: $0 {up|down|status|install|selftest|errors|listen} [--bitrate N] [--iface can0]" >&2; exit 2 ;;
esac
