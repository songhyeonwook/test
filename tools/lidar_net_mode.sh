#!/usr/bin/env bash
# MID-360 3대의 유선 네트워크 구성을 두 모드 사이에서 전환한다.
#
#   unicon : 랜허브 1대에 라이다 3대를 물리고, NIC 하나에 host_ip 3개를 별칭으로 얹는 예전 구성
#            (기본 NIC enP8p1s0). 라이다 케이블은 전부 허브에, 허브는 그 NIC 하나에 꽂는다.
#   rbio   : 라이다마다 NIC 하나를 직결하고 포트별로 host_ip 하나씩 두는 rbio 실차 배치
#            (기본 top=enxb0386cf17bd0 front=enP8p1s0 rear=enxb0386cf1873c).
#
#        tools/lidar_net_mode.sh status                         # 지금 어느 모드인지 + 링크/ping
#   sudo tools/lidar_net_mode.sh unicon [if=enP8p1s0]           # 랜허브 모드로 전환
#   sudo tools/lidar_net_mode.sh rbio   [top=IF front=IF rear=IF]   # NIC 3개 직결 모드로 전환
#   sudo tools/lidar_net_mode.sh -n unicon|rbio ...             # 실행할 nmcli 만 출력 (dry-run)
#
# 두 모드 모두 host_ip/lidar_ip 표는 같으므로 드라이버 설정(multi_MID360_config.json)은 바꿀 게 없다.
# SDK 는 host_ip 를 글자 그대로 bind 하므로 전환 뒤 livox 드라이버(bringup)는 다시 띄워야 한다.
#
# NetworkManager 프로필은 이 스크립트가 소유하는 것만 쓴다:
#   livox-hub                         unicon 용 (NIC 하나, 주소 3개)
#   livox-top / livox-front / livox-rear   rbio 용 (NIC 하나에 주소 하나)
# 전환 시 반대 모드 프로필은 내리고 autoconnect 를 끈다. host_ip 를 들고 있는 다른 프로필
# ("Wired connection N" 등)이 있으면 autoconnect 를 끄고 내린 뒤 이름을 출력한다 - 그대로 두면
# 재부팅 때 같은 서브넷이 두 포트에 올라와 라우팅이 꼬인다.
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"

ROLES=(top front rear)
declare -A LIDAR_IP=( [top]=192.168.1.135   [front]=192.168.2.102 [rear]=192.168.3.144 )
declare -A HOST_IP=(  [top]=192.168.1.50    [front]=192.168.2.50  [rear]=192.168.3.50 )
declare -A IFACE=(    [top]=enxb0386cf17bd0 [front]=enP8p1s0      [rear]=enxb0386cf1873c )
declare -A NOTE=(     [top]="livox_top  IMU 공급, 앱 LiDAR 1" [front]="livox_front 앱 LiDAR 2" [rear]="livox_rear  앱 LiDAR 3" )
HUB_IFACE=enP8p1s0
PREFIX=24
CON_HUB=livox-hub
OUR_CONS=("$CON_HUB" livox-top livox-front livox-rear)
DRY=0

usage() { sed -n '2,14p' "$SCRIPT_PATH" | sed 's/^# \{0,1\}//'; exit 2; }

# ── 공통 ─────────────────────────────────────────────────────────
run() {
  if [[ $DRY -eq 1 ]]; then printf '+'; printf ' %q' "$@"; echo; else "$@"; fi
}
need_root() {
  [[ $DRY -eq 1 ]] && return 0
  if [[ "$(id -u)" -ne 0 ]]; then
    echo "root 권한이 필요합니다: sudo $SCRIPT_PATH $*" >&2; exit 1
  fi
  command -v nmcli >/dev/null || { echo "nmcli 가 없다 (NetworkManager 전제)" >&2; exit 1; }
}
iface_exists() { [[ -d "/sys/class/net/$1" ]]; }
link_state()   { cat "/sys/class/net/$1/operstate" 2>/dev/null || echo "-"; }
# host_ip 를 가진 인터페이스 (없으면 빈 문자열)
iface_of_ip()  { ip -4 -o addr show 2>/dev/null | awk -v ip="$1" '$4 ~ "^"ip"/" {print $2; exit}'; }
# 인터페이스에서 활성인 NM 연결 이름 (없으면 빈 문자열)
active_con_of_iface() {
  nmcli -t -f NAME,DEVICE con show --active 2>/dev/null | awk -F: -v d="$1" '$2==d {print $1; exit}'
}
con_exists() { nmcli -t -f NAME con show 2>/dev/null | grep -qx -- "$1"; }
is_our_con() { local c; for c in "${OUR_CONS[@]}"; do [[ "$c" == "$1" ]] && return 0; done; return 1; }

# 프로필이 있으면 인터페이스만 다시 묶고, 없으면 만든다.
ensure_con() {
  local con="$1" ifc="$2"
  if con_exists "$con"; then
    run nmcli con mod "$con" connection.interface-name "$ifc"
  else
    run nmcli con add type ethernet con-name "$con" ifname "$ifc"
  fi
}

# 프로필에 고정 주소를 넣고 올린다. never-default: 유선이 기본 게이트웨이를 가져가
# Wi-Fi 인터넷을 끊는 것을 막는다. autoconnect-priority 로 같은 포트의 다른 프로필보다 앞선다.
configure_up() {
  local con="$1" addrs="$2"
  run nmcli con mod "$con" \
    ipv4.method manual ipv4.addresses "$addrs" ipv4.gateway "" ipv4.dns "" \
    ipv4.never-default yes ipv6.method disabled \
    connection.autoconnect yes connection.autoconnect-priority 100
  run nmcli con up "$con"
}

# 프로필을 내리고 autoconnect 를 끈다 (없으면 무시).
disable_con() {
  local con="$1"
  con_exists "$con" || return 0
  run nmcli con mod "$con" connection.autoconnect no
  if [[ $DRY -eq 1 ]]; then run nmcli con down "$con"; else nmcli con down "$con" >/dev/null 2>&1 || true; fi
}

# host_ip 를 들고 있는데 이 스크립트 소유가 아닌 프로필을 내리고 autoconnect 를 끈다.
release_foreign_holders() {
  local r hip ifc con
  for r in "${ROLES[@]}"; do
    hip="${HOST_IP[$r]}"
    ifc="$(iface_of_ip "$hip")"; [[ -n "$ifc" ]] || continue
    con="$(active_con_of_iface "$ifc")"; [[ -n "$con" ]] || continue
    is_our_con "$con" && continue
    echo "인계  '$con' ($ifc) 가 $hip 를 갖고 있어 내리고 autoconnect 를 끈다"
    disable_con "$con"
  done
}

# ── 모드 판정 / 상태 ───────────────────────────────────────────────
# 출력: unicon <iface> | rbio | none | mixed
detect_mode() {
  local r ifc holders=() uniq
  for r in "${ROLES[@]}"; do
    ifc="$(iface_of_ip "${HOST_IP[$r]}")"
    holders+=("${ifc:-NONE}")
  done
  uniq="$(printf '%s\n' "${holders[@]}" | sort -u)"
  if [[ "$(wc -l <<<"$uniq")" -eq 1 ]]; then
    [[ "$uniq" == "NONE" ]] && { echo none; return; }
    echo "unicon $uniq"; return
  fi
  if [[ "$(wc -l <<<"$uniq")" -eq 3 && "$uniq" != *NONE* ]]; then echo rbio; return; fi
  echo mixed
}

do_status() {
  local fail=0 r hip lip ifc lnk png mode
  mode="$(detect_mode)"
  case "$mode" in
    unicon*) echo "모드  unicon (랜허브: ${mode#unicon } 하나에 host_ip 3개)";;
    rbio)    echo "모드  rbio (NIC 3개 직결, 포트별 host_ip 하나)";;
    none)    echo "모드  없음 - host_ip 가 하나도 없다.  sudo $SCRIPT_PATH unicon|rbio";;
    *)       echo "모드  섞임 - 일부만 설정돼 있다. 원하는 모드로 다시 실행하면 정리된다";;
  esac
  echo
  printf "%-6s %-17s %-15s %-14s %-8s %-5s %s\n" 역할 포트 lidar_ip host_ip 링크 ping 비고
  for r in "${ROLES[@]}"; do
    hip="${HOST_IP[$r]}"; lip="${LIDAR_IP[$r]}"
    ifc="$(iface_of_ip "$hip")"; lnk="-"; png="-"
    if [[ -n "$ifc" ]]; then
      lnk="$(link_state "$ifc")"
      if ping -c1 -W1 -I "$ifc" "$lip" >/dev/null 2>&1; then png=OK; else png=NO; fail=1; fi
    else
      fail=1
    fi
    printf "%-6s %-17s %-15s %-14s %-8s %-5s %s\n" "$r" "${ifc:-(host_ip 없음)}" "$lip" "$hip" "$lnk" "$png" "${NOTE[$r]}"
  done
  echo
  echo "NM 프로필:"
  local c act
  for c in "${OUR_CONS[@]}"; do
    if con_exists "$c"; then
      act="$(nmcli -g GENERAL.STATE con show "$c" 2>/dev/null || true)"
      printf "  %-12s iface=%-17s autoconnect=%-4s %s\n" "$c" \
        "$(nmcli -g connection.interface-name con show "$c" 2>/dev/null)" \
        "$(nmcli -g connection.autoconnect con show "$c" 2>/dev/null)" "${act:-inactive}"
    else
      printf "  %-12s (없음)\n" "$c"
    fi
  done
  echo
  if [[ $fail -eq 0 ]]; then
    echo "라이다 3대 모두 응답. 드라이버가 못 붙으면 bringup 을 다시 띄운다 (SDK 는 시작 시 host_ip 를 bind)."
  else
    case "$mode" in
      unicon*) echo "unicon: 라이다 3대 케이블이 모두 랜허브에, 허브가 ${mode#unicon } 에 꽂혀 있는지 확인.";;
      rbio)    echo "rbio: host_ip 는 있는데 ping 이 NO 면 케이블이 다른 포트에 꽂힌 것 - 라이다는 자기 서브넷 포트로만 응답.";;
    esac
  fi
  return $fail
}

# ── unicon: 랜허브, NIC 하나에 별칭 3개 ───────────────────────────────
do_unicon() {
  local arg
  for arg in "$@"; do
    case "$arg" in
      if=*) HUB_IFACE="${arg#*=}" ;;
      *) echo "unicon 은 if=<NIC> 만 받는다: $arg" >&2; exit 2 ;;
    esac
  done
  need_root unicon "$@"
  if ! iface_exists "$HUB_IFACE"; then
    echo "FAIL  포트 $HUB_IFACE 없음. 있는 포트: $(ls /sys/class/net | tr '\n' ' ')" >&2
    echo "      sudo $SCRIPT_PATH unicon if=<포트이름>" >&2; exit 1
  fi
  local addrs="" r
  for r in "${ROLES[@]}"; do addrs+="${addrs:+,}${HOST_IP[$r]}/$PREFIX"; done

  echo "== unicon: $HUB_IFACE 에 $addrs"
  release_foreign_holders
  for r in "${ROLES[@]}"; do disable_con "livox-$r"; done
  ensure_con "$CON_HUB" "$HUB_IFACE"
  configure_up "$CON_HUB" "$addrs"
  echo
  [[ $DRY -eq 1 ]] || do_status || true
}

# ── rbio: NIC 3개 직결, 포트별 host_ip 하나 ────────────────────────────
do_rbio() {
  local arg
  for arg in "$@"; do
    case "$arg" in
      top=*|front=*|rear=*) IFACE[${arg%%=*}]="${arg#*=}" ;;
      *) echo "rbio 는 top=IF front=IF rear=IF 만 받는다: $arg" >&2; exit 2 ;;
    esac
  done
  need_root rbio "$@"
  local r
  for r in "${ROLES[@]}"; do
    if ! iface_exists "${IFACE[$r]}"; then
      echo "FAIL  포트 ${IFACE[$r]} 없음 ($r). 있는 포트: $(ls /sys/class/net | tr '\n' ' ')" >&2
      echo "      sudo $SCRIPT_PATH rbio $r=<포트이름>   (enx… 는 USB 어댑터 MAC 기반 이름, ip -br link 로 확인)" >&2
      exit 1
    fi
  done
  # 같은 NIC 를 두 역할에 주면 별칭 구성으로 되돌아간다. 그건 unicon 이다.
  if [[ "$(printf '%s\n' "${IFACE[@]}" | sort -u | wc -l)" -ne 3 ]]; then
    echo "FAIL  포트 3개가 서로 달라야 한다: top=${IFACE[top]} front=${IFACE[front]} rear=${IFACE[rear]}" >&2; exit 1
  fi

  echo "== rbio: top=${IFACE[top]} front=${IFACE[front]} rear=${IFACE[rear]}"
  release_foreign_holders
  disable_con "$CON_HUB"
  for r in "${ROLES[@]}"; do
    ensure_con "livox-$r" "${IFACE[$r]}"
    configure_up "livox-$r" "${HOST_IP[$r]}/$PREFIX"
    echo "설정  $r: ${IFACE[$r]}  ${HOST_IP[$r]}/$PREFIX  -> lidar ${LIDAR_IP[$r]}"
  done
  echo
  [[ $DRY -eq 1 ]] || do_status || true
}

# ── 진입 ─────────────────────────────────────────────────────────
[[ "${1:-}" == "-n" || "${1:-}" == "--dry-run" ]] && { DRY=1; shift; }
ACTION="${1:-status}"; shift || true
case "$ACTION" in
  status) do_status ;;
  unicon) do_unicon "$@" ;;
  rbio)   do_rbio "$@" ;;
  *) usage ;;
esac
