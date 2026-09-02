#!/usr/bin/env bash
# MID-360 3대의 유선 포트 설정/점검. rbio 실차(TransferRobot-Hanyang Jetson)와 같은 배치:
#   물리 포트 하나에 라이다 하나, 포트마다 서브넷 하나, host_ip 는 포트별로 하나씩 고정.
# Livox SDK 는 multi_MID360_config.json 의 host_ip 를 글자 그대로 bind 하므로, 그 주소가
# 이 PC 의 어느 포트에든 실제로 붙어 있어야 라이다가 붙는다.
#
#        tools/lidar_net.sh status                       # 포트<->host_ip<->라이다 표, 링크/ping, JSON 대조
#   sudo tools/lidar_net.sh install                      # 포트마다 고정 IP (NetworkManager, 재부팅 후 유지)
#   sudo tools/lidar_net.sh install top=enxAAAA front=enP8p1s0 rear=enxBBBB   # 포트 이름이 다를 때
#
# 기본 포트 이름은 rbio 실차 값이다. enx... 는 USB 이더넷 어댑터의 MAC 기반 이름이라
# 어댑터를 바꾸면 달라진다. `ip -br link` 로 확인해서 넘긴다.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
CONFIG_JSON="${LIVOX_CONFIG_JSON:-${SCRIPT_DIR}/../src/livox_ros_driver2/config/multi_MID360_config.json}"
SDK_SRC="${SCRIPT_DIR}/../third_party/Livox-SDK2/sdk_core/device_manager.cpp"

# 역할  lidar_ip        host_ip        기본 포트           설명
ROLES=(top front rear)
declare -A LIDAR_IP=( [top]=192.168.1.135 [front]=192.168.2.102 [rear]=192.168.3.144 )
declare -A HOST_IP=(  [top]=192.168.1.50  [front]=192.168.2.50  [rear]=192.168.3.50 )
declare -A IFACE=(    [top]=enxb0386cf17bd0 [front]=enP8p1s0 [rear]=enxb0386cf1873c )
declare -A NOTE=(     [top]="livox_top  IMU 공급, 앱 LiDAR 1" [front]="livox_front 앱 LiDAR 2 (rbio /livox/lidar)" [rear]="livox_rear  앱 LiDAR 3" )

ACTION="${1:-status}"
shift || true
for arg in "$@"; do
  case "$arg" in
    top=*|front=*|rear=*) IFACE[${arg%%=*}]="${arg#*=}" ;;
    *) echo "알 수 없는 인자: $arg  (top=IF front=IF rear=IF 만 받는다)" >&2; exit 2 ;;
  esac
done

need_root() {
  if [[ "$(id -u)" -ne 0 ]]; then
    echo "root 권한이 필요합니다: sudo $0 $ACTION $*" >&2
    exit 1
  fi
}

# host_ip 를 가진 인터페이스 이름 (없으면 빈 문자열)
iface_of_ip() { ip -4 -o addr show 2>/dev/null | awk -v ip="$1" '$4 ~ "^"ip"/" {print $2; exit}'; }
link_state()  { cat "/sys/class/net/$1/operstate" 2>/dev/null || echo "-"; }

# JSON 의 (lidar_ip, host_ip) 쌍이 이 스크립트의 표와 같은지 대조
check_json() {
  [[ -f "$CONFIG_JSON" ]] || { echo "WARN  JSON 없음: $CONFIG_JSON"; return 1; }
  local expect=""
  for r in "${ROLES[@]}"; do expect+="${LIDAR_IP[$r]}=${HOST_IP[$r]} "; done
  python3 - "$CONFIG_JSON" $expect <<'PY'
import json, sys
cfg = json.load(open(sys.argv[1]))
expect = dict(a.split("=") for a in sys.argv[2:])
got = {}
for h in cfg["MID360"]["host_net_info"]:
    for lip in h["lidar_ip"]:
        got[lip] = h["host_ip"]
ok = True
for lip, hip in expect.items():
    if got.get(lip) != hip:
        print(f"WARN  JSON 불일치: lidar {lip} -> host {got.get(lip)} (스크립트 표는 {hip})"); ok = False
extra = set(got) - set(expect)
if extra:
    print(f"WARN  JSON 에만 있는 라이다: {sorted(extra)}"); ok = False
cfg_ips = {c["ip"] for c in cfg["lidar_configs"]}
if cfg_ips != set(expect):
    print(f"WARN  lidar_configs 의 ip {sorted(cfg_ips)} 가 host_net_info 와 다름"); ok = False
for c in cfg["lidar_configs"]:
    e = c.get("extrinsic_parameter", {})
    if any(float(e.get(k, 0)) != 0.0 for k in ("roll", "pitch", "yaw", "x", "y", "z")):
        print(f"WARN  {c['ip']} extrinsic_parameter 가 0 이 아님 (장착 기하는 URDF 에만 둔다)"); ok = False
print("OK    JSON host_ip/lidar_ip 표와 일치" if ok else "FAIL  JSON 을 확인할 것")
sys.exit(0 if ok else 1)
PY
}

do_status() {
  local fail=0
  printf "%-6s %-17s %-15s %-14s %-8s %-6s %s\n" 역할 포트 lidar_ip host_ip 링크 ping 비고
  for r in "${ROLES[@]}"; do
    local hip="${HOST_IP[$r]}" lip="${LIDAR_IP[$r]}"
    local ifc; ifc="$(iface_of_ip "$hip")"
    local shown="${ifc:-(host_ip 없음)}" lnk="-" png="-"
    if [[ -n "$ifc" ]]; then
      lnk="$(link_state "$ifc")"
      if ping -c1 -W1 -I "$ifc" "$lip" >/dev/null 2>&1; then png=OK; else png=NO; fail=1; fi
    else
      fail=1
    fi
    if [[ -n "$ifc" && "$ifc" != "${IFACE[$r]}" ]]; then shown="$ifc (기본 ${IFACE[$r]})"; fi
    printf "%-6s %-17s %-15s %-14s %-8s %-6s %s\n" "$r" "$shown" "$lip" "$hip" "$lnk" "$png" "${NOTE[$r]}"
  done
  echo
  # 같은 포트에 host_ip 가 둘 이상이면 (옛 "스위치 한 포트" 방식) 동작은 하지만 rbio 배치가 아니다
  local dup; dup="$(for r in "${ROLES[@]}"; do iface_of_ip "${HOST_IP[$r]}"; done | grep -v '^$' | sort | uniq -d || true)"
  [[ -n "$dup" ]] && echo "WARN  한 포트에 host_ip 여러 개: $dup  (rbio 배치는 포트 하나에 라이다 하나)"
  for r in "${ROLES[@]}"; do
    local ifc; ifc="$(iface_of_ip "${HOST_IP[$r]}")"
    [[ -z "$ifc" ]] && echo "FAIL  ${HOST_IP[$r]} 가 이 PC 에 없다 -> 드라이버 'bind failed' / 'Init lds lidar fail!'.  sudo $0 install"
    [[ -n "$ifc" && "$(link_state "$ifc")" != "up" ]] && echo "FAIL  $ifc 링크 다운: 케이블/라이다 전원 확인 (${LIDAR_IP[$r]})"
  done
  if grep -q detection_host_ips_ "$SDK_SRC" 2>/dev/null; then
    echo "OK    Livox-SDK2 multi-NIC 패치 적용됨 (third_party 소스). 설치본은 빌드 시점 기준."
  else
    echo "WARN  third_party/Livox-SDK2 에 multi-NIC 패치가 없다 - 포트 3개면 라이다 1대만 붙는다"
  fi
  check_json || fail=1
  echo
  if [[ $fail -eq 0 ]]; then
    echo "라이다 3대 모두 자기 포트에서 응답. ping 은 되는데 드라이버가 못 붙으면 README '라이다 네트워크' 참고."
  else
    echo "host_ip 는 있는데 ping 이 안 되면 케이블이 다른 포트에 꽂힌 것이다 - 라이다는 자기 서브넷 포트로만 응답한다."
  fi
  return $fail
}

# 인터페이스에 묶인 NetworkManager 연결 이름. 없으면 빈 문자열.
con_of_iface() {
  local ifc="$1" name
  name="$(nmcli -t -f NAME,DEVICE con show --active 2>/dev/null | awk -F: -v d="$ifc" '$2==d {print $1; exit}')"
  [[ -n "$name" ]] && { echo "$name"; return; }
  while IFS= read -r name; do
    [[ "$(nmcli -g connection.interface-name con show "$name" 2>/dev/null)" == "$ifc" ]] && { echo "$name"; return; }
  done < <(nmcli -t -f NAME con show 2>/dev/null)
}

do_install() {
  need_root "$@"
  command -v nmcli >/dev/null || { echo "nmcli 가 없다 (NetworkManager 전제)" >&2; exit 1; }
  for r in "${ROLES[@]}"; do
    local ifc="${IFACE[$r]}" hip="${HOST_IP[$r]}"
    if [[ ! -d "/sys/class/net/$ifc" ]]; then
      echo "FAIL  포트 $ifc 없음 ($r). 있는 포트: $(ls /sys/class/net | tr '\n' ' ')" >&2
      echo "      sudo $0 install $r=<포트이름> 으로 지정" >&2
      exit 1
    fi
    # 다른 포트가 이 host_ip 를 이미 갖고 있으면 (옛 스위치 방식) 거기서 빼지 않으면 라우팅이 꼬인다
    local holder; holder="$(iface_of_ip "$hip")"
    if [[ -n "$holder" && "$holder" != "$ifc" ]]; then
      echo "FAIL  $hip 가 이미 $holder 에 있다. 그 연결에서 먼저 빼고 다시 실행:" >&2
      echo "      nmcli -t -f NAME,DEVICE con show --active | grep $holder  ->  sudo nmcli con mod <이름> -ipv4.addresses $hip/24; sudo nmcli con up <이름>" >&2
      exit 1
    fi
    local con; con="$(con_of_iface "$ifc")"
    if [[ -z "$con" ]]; then
      con="livox-$r"
      nmcli con add type ethernet ifname "$ifc" con-name "$con" >/dev/null
      echo "새 연결 $con ($ifc)"
    fi
    nmcli con mod "$con" \
      connection.interface-name "$ifc" connection.autoconnect yes \
      ipv4.method manual ipv4.addresses "$hip/24" ipv4.gateway "" ipv4.dns "" \
      ipv4.never-default yes ipv6.method ignore
    nmcli con up "$con" >/dev/null
    echo "설정  $r: $ifc  $hip/24  (연결 '$con', never-default)"
  done
  echo
  do_status
}

case "$ACTION" in
  status)  do_status ;;
  install) do_install "$@" ;;
  *) sed -n '2,12p' "$0"; exit 2 ;;
esac
