#!/usr/bin/env bash
# MID-360 3대를 유선 NIC 3개로 하나씩 받는다 (랜허브 1개 + IP 별칭 3개 구성을 대체).
#
# 주소 체계는 rbio TransferRobot 과 같다 - 라이다마다 서브넷이 다르다. 달라진 건
# 그 서브넷을 한 NIC 에 별칭으로 얹지 않고 물리 NIC 를 하나씩 전용으로 쓴다는 것뿐이다.
# 드라이버 설정(multi_MID360_config.json)은 host_ip 만 보므로 바꿀 게 없다.
#
#   sudo tools/lidar_net_setup.sh apply <top_if> <front_if> <rear_if>
#        tools/lidar_net_setup.sh status
#   sudo tools/lidar_net_setup.sh down     # 연결만 내린다
#   sudo tools/lidar_net_setup.sh clean    # 만든 연결 프로필까지 지운다
#
# 인터페이스 이름은 장비마다 다르다. `ip -br link` 로 먼저 확인한다.
# 어느 포트에 어느 라이다가 꽂혔는지 모르면 한 대씩 꽂아가며 `status` 로 본다.
set -euo pipefail

# 이름:host_ip:lidar_ip  (apply 인자 순서와 같다)
LIDARS=(
  "top:192.168.1.50:192.168.1.135"
  "front:192.168.2.50:192.168.2.102"
  "rear:192.168.3.50:192.168.3.144"
)
PREFIX=24
CON_PREFIX="livox-"
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"

need_root() {
  if [[ "$(id -u)" -ne 0 ]]; then
    echo "root 권한이 필요합니다: sudo ${SCRIPT_PATH} $*" >&2
    exit 1
  fi
}

usage() {
  sed -n '2,15p' "${SCRIPT_PATH}" | sed 's/^# \{0,1\}//'
  exit 2
}

do_apply() {
  need_root apply "$@"
  [[ $# -eq 3 ]] || usage
  local ifaces=("$@")

  # 같은 NIC 를 두 번 지정하면 별칭 구성으로 되돌아간다. 그걸 막는 게 이 스크립트의 목적이다.
  local uniq
  uniq="$(printf '%s\n' "${ifaces[@]}" | sort -u | wc -l)"
  if [[ "${uniq}" -ne 3 ]]; then
    echo "인터페이스 3개가 서로 달라야 합니다: ${ifaces[*]}" >&2
    exit 1
  fi

  local i name host lidar iface con
  for i in "${!LIDARS[@]}"; do
    IFS=: read -r name host lidar <<<"${LIDARS[$i]}"
    iface="${ifaces[$i]}"
    if [[ ! -e "/sys/class/net/${iface}" ]]; then
      echo "그런 인터페이스가 없습니다: ${iface} (ip -br link 로 확인)" >&2
      exit 1
    fi
    con="${CON_PREFIX}${name}"

    if nmcli -t -f NAME con show | grep -qx "${con}"; then
      nmcli con mod "${con}" connection.interface-name "${iface}"
    else
      nmcli con add type ethernet con-name "${con}" ifname "${iface}" >/dev/null
    fi
    # never-default: 유선이 기본 게이트웨이를 가져가 Wi-Fi 인터넷을 끊는 것을 막는다.
    nmcli con mod "${con}" \
      ipv4.method manual \
      ipv4.addresses "${host}/${PREFIX}" \
      ipv4.gateway "" \
      ipv4.dns "" \
      ipv4.never-default yes \
      ipv6.method disabled \
      connection.autoconnect yes
    nmcli con up "${con}" >/dev/null
    echo "  ${con}  ${iface}  ${host}/${PREFIX}  -> lidar ${lidar}"
  done

  echo
  echo "부팅 시 자동 적용된다(autoconnect). 확인: ${SCRIPT_PATH} status"
}

do_status() {
  local name host lidar spec owner rp
  printf '%-7s %-16s %-16s %-12s %-8s %s\n' 라이다 host_ip lidar_ip 인터페이스 ping rp_filter
  for spec in "${LIDARS[@]}"; do
    IFS=: read -r name host lidar <<<"${spec}"
    owner="$(ip -4 -o addr show 2>/dev/null | awk -v ip="${host}/" '$4 ~ "^"ip {print $2; exit}')"
    owner="${owner:--}"
    rp="-"
    if [[ "${owner}" != "-" && -r "/proc/sys/net/ipv4/conf/${owner}/rp_filter" ]]; then
      rp="$(cat "/proc/sys/net/ipv4/conf/${owner}/rp_filter")"
    fi
    if ping -c1 -W1 "${lidar}" >/dev/null 2>&1; then
      printf '%-7s %-16s %-16s %-12s %-8s %s\n' "${name}" "${host}" "${lidar}" "${owner}" OK "${rp}"
    else
      printf '%-7s %-16s %-16s %-12s %-8s %s\n' "${name}" "${host}" "${lidar}" "${owner}" FAIL "${rp}"
    fi
  done

  echo
  echo "각 라이다로 나가는 경로:"
  for spec in "${LIDARS[@]}"; do
    IFS=: read -r name host lidar <<<"${spec}"
    echo "  ${name}: $(ip route get "${lidar}" 2>&1 | head -1)"
  done

  echo
  echo "인터페이스 세 개가 서로 달라야 한다. 같으면 아직 랜허브(별칭) 구성이다."
  echo "host_ip 가 하나라도 '-' 면 드라이버가 bind failed -> Init lds lidar fail! 로 죽는다."
}

do_down() {
  need_root down
  local spec name rest
  for spec in "${LIDARS[@]}"; do
    IFS=: read -r name rest <<<"${spec}"
    nmcli con down "${CON_PREFIX}${name}" 2>/dev/null || true
  done
}

do_clean() {
  need_root clean
  local spec name rest
  for spec in "${LIDARS[@]}"; do
    IFS=: read -r name rest <<<"${spec}"
    nmcli con delete "${CON_PREFIX}${name}" 2>/dev/null || true
  done
}

case "${1:-status}" in
  apply) shift; do_apply "$@" ;;
  status) do_status ;;
  down)  do_down ;;
  clean) do_clean ;;
  *) usage ;;
esac
