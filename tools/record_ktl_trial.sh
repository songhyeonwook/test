#!/usr/bin/env bash
# KTL 자율주행 시험 1회분을 bag + 메타데이터 + 정렬오차 결과로 묶어 저장한다.

set -uo pipefail

usage() {
  cat <<'EOF'
사용법:
  ./tools/record_ktl_trial.sh <시험번호 01~10> <static|dynamic|dynamic2> "<상황 설명>" [--raw]

예시:
  ./tools/record_ktl_trial.sh 01 static "통로 중앙 의자 1개"
  ./tools/record_ktl_trial.sh 06 dynamic "전방 좌→우 보행자 진입 후 이탈"
  ./tools/record_ktl_trial.sh 01 dynamic2 "동적장애물 시나리오 2"

--raw: 병합 점군 외에 라이다 3대의 원본 PointCloud2도 기록한다(용량 크게 증가).
EOF
}

if [[ $# -lt 3 || $# -gt 4 ]]; then
  usage
  exit 2
fi

trial_input="$1"
scenario_type="$2"
scenario_description="$3"
raw_mode="${4:-}"

if ! [[ "$trial_input" =~ ^[0-9]{1,2}$ ]]; then
  echo "오류: 시험번호는 01~10의 숫자여야 합니다." >&2
  exit 2
fi
trial_number=$((10#$trial_input))
if (( trial_number < 1 || trial_number > 10 )); then
  echo "오류: 시험번호 범위는 01~10입니다." >&2
  exit 2
fi
trial_id=$(printf '%02d' "$trial_number")

if [[ "$scenario_type" != "static" \
      && "$scenario_type" != "dynamic" \
      && "$scenario_type" != "dynamic2" ]]; then
  echo "오류: 시험 종류는 static, dynamic 또는 dynamic2여야 합니다." >&2
  exit 2
fi
if [[ -n "$raw_mode" && "$raw_mode" != "--raw" ]]; then
  echo "오류: 네 번째 인자는 --raw만 사용할 수 있습니다." >&2
  exit 2
fi

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd -- "$script_dir/.." && pwd)
setup_file="$repo_root/install/setup.bash"
if [[ ! -f "$setup_file" ]]; then
  echo "오류: $setup_file 이 없습니다. 워크스페이스 경로를 확인하세요." >&2
  exit 1
fi
# ROS/colcon setup 스크립트는 일부 환경변수가 없을 수 있으므로 source 동안만
# nounset 검사를 끈다.
set +u
# shellcheck disable=SC1090
source "$setup_file"
set -u

run_date=$(date +%Y%m%d)
run_stamp=$(date +%Y%m%d_%H%M%S)
output_root="$repo_root/bag/ktl_$run_date"
bag_dir="$output_root/trial_${trial_id}_${scenario_type}_$run_stamp"
manifest_path="$output_root/trials.csv"
goal_log_dir="${GOAL_LOG_DIR:-/home/test/nav_goal_results}"
latest_goal_result="$goal_log_dir/latest_result.txt"
cumulative_goal_results="$goal_log_dir/goal_results.csv"

mkdir -p "$output_root"
if [[ -e "$bag_dir" ]]; then
  echo "오류: 출력 경로가 이미 존재합니다: $bag_dir" >&2
  exit 1
fi

start_epoch=$(date +%s)
start_iso=$(date --iso-8601=seconds)
latest_hash_before=""
if [[ -f "$latest_goal_result" ]]; then
  latest_hash_before=$(sha256sum "$latest_goal_result" | awk '{print $1}')
fi

topics=(
  /tf
  /tf_static
  /rosout
  /goal_pose
  /map
  /map_metadata
  /localization
  /Odometry
  /odom
  /mode
  /steer_angle_deg
  /navigate_to_pose/_action/status
  /navigate_to_pose/_action/feedback
  /plan
  /local_plan
  /global_costmap/costmap
  /global_costmap/costmap_updates
  /local_costmap/costmap
  /local_costmap/costmap_updates
  /global_costmap/published_footprint
  /local_costmap/published_footprint
  /cmd_vel_nav
  /cmd_vel_nav_raw
  /cmd_vel_collision_monitor_in
  /cmd_vel_dynamic_monitor_in
  /cmd_vel
  /livox_merge/merged_pointcloud_sliced
  /dynamic_obstacles_HY
  /dynamic_obstacle_tracker_HY/status
  /directional_collision_gate_HY/status
  /collision_monitor/surround_dynamic_stop_zone
)

if [[ "$raw_mode" == "--raw" ]]; then
  topics+=(
    /livox/lidar_192_168_2_102
    /livox/lidar_192_168_3_144
    /livox/lidar_192_168_1_135
  )
fi

echo
echo "===== KTL 시험 $trial_id 기록 시작 ====="
echo "종류      : $scenario_type"
echo "상황      : $scenario_description"
echo "저장 위치 : $bag_dir"
echo "남은 용량 : $(df -h --output=avail "$repo_root/bag" | tail -n 1 | xargs)"
echo "종료 방법 : Goal 결과가 나온 뒤 2~3초 기다리고 Ctrl+C"
echo

record_rc=0
ros2 bag record \
  --include-hidden-topics \
  --compression-mode file \
  --compression-format zstd \
  --compression-threads 2 \
  --max-cache-size 134217728 \
  -o "$bag_dir" \
  "${topics[@]}" || record_rc=$?

end_epoch=$(date +%s)
end_iso=$(date --iso-8601=seconds)

# rosbag2는 Ctrl+C(SIGINT) 종료를 비정상 코드로 돌려줄 수 있지만,
# metadata.yaml이 있으면 정상적으로 마감된 bag으로 취급한다.
if [[ ! -f "$bag_dir/metadata.yaml" ]]; then
  echo "오류: metadata.yaml이 없어 bag 저장이 완료되지 않았습니다." >&2
  if (( record_rc == 0 )); then
    record_rc=1
  fi
  exit "$record_rc"
fi

goal_result_status="MISSING"
goal_result=""
position_error_mm=""
measurement_valid=""
pass_70mm=""
latest_hash_after=""
if [[ -f "$latest_goal_result" ]]; then
  latest_hash_after=$(sha256sum "$latest_goal_result" | awk '{print $1}')
fi
if [[ -f "$latest_goal_result" \
      && -n "$latest_hash_after" \
      && "$latest_hash_after" != "$latest_hash_before" ]]; then
  cp -a "$latest_goal_result" "$bag_dir/goal_result.txt"
  goal_result_status="SAVED"
  goal_result=$(sed -n 's/^result: //p' "$latest_goal_result" | tail -n 1)
  position_error_mm=$(sed -n 's/^position_error_mm: //p' "$latest_goal_result" | tail -n 1)
  measurement_valid=$(sed -n 's/^measurement_valid: //p' "$latest_goal_result" | tail -n 1)
  pass_70mm=$(sed -n 's/^pass_70mm: //p' "$latest_goal_result" | tail -n 1)
else
  printf '%s\n' \
    "이 시험 기록 시작 이후 생성된 latest_result.txt가 없습니다." \
    "Goal SUCCEEDED/ABORTED 결과와 goal_error_recorder 실행 여부를 확인하세요." \
    > "$bag_dir/goal_result_missing.txt"
fi

if [[ -f "$cumulative_goal_results" ]]; then
  cp -a "$cumulative_goal_results" "$bag_dir/goal_results_cumulative_snapshot.csv"
fi

# 시험 당시 설정을 함께 보존하여 나중에 파라미터 변경 여부를 추적한다.
cp -a "$repo_root/src/navigation/params/nav2_params.yaml" "$bag_dir/" 2>/dev/null || true
cp -a "$repo_root/src/fast_lio_localization/config/mid360.yaml" "$bag_dir/" 2>/dev/null || true
cp -a "$repo_root/src/bringup/launch/bringup.launch.py" "$bag_dir/" 2>/dev/null || true

external_error_mm=""
operator_note=""
if [[ -t 0 ]]; then
  read -r -p "외부 레이저 실측 오차(mm, 미측정이면 Enter): " external_error_mm || true
  read -r -p "시험 메모(없으면 Enter): " operator_note || true
fi

{
  printf 'trial_id: %s\n' "$trial_id"
  printf 'scenario_type: %s\n' "$scenario_type"
  printf 'scenario_description: %s\n' "$scenario_description"
  printf 'started_at: %s\n' "$start_iso"
  printf 'ended_at: %s\n' "$end_iso"
  printf 'duration_sec: %s\n' "$((end_epoch - start_epoch))"
  printf 'bag_directory: %s\n' "$bag_dir"
  printf 'raw_lidar_recorded: %s\n' "$([[ "$raw_mode" == "--raw" ]] && echo YES || echo NO)"
  printf 'goal_result_copy: %s\n' "$goal_result_status"
  printf 'nav_result: %s\n' "$goal_result"
  printf 'internal_position_error_mm: %s\n' "$position_error_mm"
  printf 'measurement_valid: %s\n' "$measurement_valid"
  printf 'pass_70mm: %s\n' "$pass_70mm"
  printf 'external_laser_error_mm: %s\n' "$external_error_mm"
  printf 'operator_note: %s\n' "$operator_note"
  printf 'topics:\n'
  printf '  - %s\n' "${topics[@]}"
} > "$bag_dir/trial_info.txt"

csv_quote() {
  local value=${1//\"/\"\"}
  printf '"%s"' "$value"
}

if [[ ! -s "$manifest_path" ]]; then
  printf '%s\n' \
    'trial_id,scenario_type,scenario_description,started_at,ended_at,duration_sec,bag_directory,nav_result,internal_position_error_mm,measurement_valid,pass_70mm,external_laser_error_mm,operator_note' \
    > "$manifest_path"
fi
{
  csv_quote "$trial_id"; printf ','
  csv_quote "$scenario_type"; printf ','
  csv_quote "$scenario_description"; printf ','
  csv_quote "$start_iso"; printf ','
  csv_quote "$end_iso"; printf ','
  csv_quote "$((end_epoch - start_epoch))"; printf ','
  csv_quote "$bag_dir"; printf ','
  csv_quote "$goal_result"; printf ','
  csv_quote "$position_error_mm"; printf ','
  csv_quote "$measurement_valid"; printf ','
  csv_quote "$pass_70mm"; printf ','
  csv_quote "$external_error_mm"; printf ','
  csv_quote "$operator_note"; printf '\n'
} >> "$manifest_path"

echo
echo "===== KTL 시험 $trial_id 저장 완료 ====="
echo "bag       : $bag_dir"
echo "시험 요약 : $bag_dir/trial_info.txt"
echo "전체 누적 : $manifest_path"
if [[ "$goal_result_status" == "SAVED" ]]; then
  echo "Nav 결과  : ${goal_result:-UNKNOWN}, 내부오차=${position_error_mm:-UNKNOWN} mm, 70mm=${pass_70mm:-UNKNOWN}"
else
  echo "주의      : 이번 시험의 정렬오차 결과가 생성되지 않았습니다."
fi
