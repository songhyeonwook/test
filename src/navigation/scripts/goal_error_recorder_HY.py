#!/usr/bin/env python3
"""HY 플랫폼 Nav2 목표 도달 오차 자동 기록기.

RViz가 발행한 /goal_pose를 보관하고 NavigateToPose 액션이 종료되는 순간의
/localization을 CSV에 기록한다. 일회성 /goal_pose를 DDS가 놓친 경우에는 Nav2
global plan의 마지막 pose를 동일 Goal로 자동 복구한다. 위치 오차는 두 메시지가
모두 map 프레임일 때 유효하며, KTL의 외부 실측값을 대신하는 값이 아니라 ROS
내부 성능 확인용이다.
"""

import csv
from datetime import datetime
import math
import os

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time


CSV_FIELDS = [
    'recorded_at',
    'result',
    'goal_id',
    'goal_stamp_sec',
    'goal_stamp_nanosec',
    'success_stamp_sec',
    'success_stamp_nanosec',
    'localization_stamp_sec',
    'localization_stamp_nanosec',
    'localization_sample_age_ms',
    'measurement_valid',
    'validation_note',
    'goal_frame',
    'localization_frame',
    'goal_x_m',
    'goal_y_m',
    'goal_yaw_deg',
    'actual_x_m',
    'actual_y_m',
    'actual_yaw_deg',
    'error_x_mm',
    'error_y_mm',
    'position_error_mm',
    'yaw_error_deg',
    'pass_70mm',
]


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalized_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class GoalErrorRecorderHY(Node):
    def __init__(self):
        super().__init__('goal_error_recorder_HY')

        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('localization_topic', '/localization')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter(
            'status_topic', '/navigate_to_pose/_action/status')
        self.declare_parameter('output_directory', '~/nav_goal_results_HY')
        self.declare_parameter('position_limit_mm', 70.0)
        # ros2 topic pub에서 header.stamp를 생략하면 0으로 발행된다. 이 경우
        # bt_navigator와 기록기의 콜백 순서는 보장되지 않으므로 액션 접수
        # 시각이 기록기 수신 시각보다 조금 앞서도 같은 Goal로 인정한다.
        self.declare_parameter('zero_stamp_match_slop_ms', 1000.0)
        # FAST-LIO 처리 지연은 실차에서 약 0.65초였다. 단순 수신 age가 아니라
        # 성공 시각 이후 stamp를 가진 첫 localization인지도 함께 검증한다.
        self.declare_parameter('max_localization_age_ms', 1500.0)
        self.declare_parameter('fresh_sample_wait_ms', 2500.0)
        # 출장 플랫폼에서는 RViz/CLI 발행자의 QoS와 관계없이 일회성
        # /goal_pose를 받도록 BEST_EFFORT subscriber를 선택할 수 있다.
        # 기본값은 false라 HY의 기존 동작에는 변화가 없다.
        self.declare_parameter('goal_best_effort_qos', False)
        # plan endpoint는 costmap grid에 맞춰 원 Goal에서 수 cm 이동할 수 있다.
        # 출장 기록에서는 fallback 좌표로 PASS/FAIL을 확정하지 않는다.
        self.declare_parameter('allow_plan_fallback_measurement', True)
        # 고정된 KTL 반복시험에서는 plan 끝점이 costmap grid에 스냅되더라도
        # 원래 기준 좌표를 복원할 수 있다. HY 기본값은 비활성화다.
        self.declare_parameter('reference_goal_enabled', False)
        self.declare_parameter('reference_goal_x', 0.0)
        self.declare_parameter('reference_goal_y', 0.0)
        self.declare_parameter('reference_goal_yaw_deg', 0.0)
        self.declare_parameter('reference_goal_match_radius', 0.20)

        goal_topic = self.get_parameter('goal_topic').value
        localization_topic = self.get_parameter('localization_topic').value
        plan_topic = self.get_parameter('plan_topic').value
        status_topic = self.get_parameter('status_topic').value
        output_directory = os.path.abspath(os.path.expanduser(
            self.get_parameter('output_directory').value))
        self.position_limit_mm = float(
            self.get_parameter('position_limit_mm').value)
        self.zero_stamp_match_slop_ns = int(float(
            self.get_parameter('zero_stamp_match_slop_ms').value)
            * 1_000_000.0)
        self.max_localization_age_ms = float(
            self.get_parameter('max_localization_age_ms').value)
        self.fresh_sample_wait_ms = float(
            self.get_parameter('fresh_sample_wait_ms').value)
        self.goal_best_effort_qos = bool(
            self.get_parameter('goal_best_effort_qos').value)
        self.allow_plan_fallback_measurement = bool(
            self.get_parameter('allow_plan_fallback_measurement').value)
        self.reference_goal_enabled = bool(
            self.get_parameter('reference_goal_enabled').value)
        self.reference_goal_x = float(
            self.get_parameter('reference_goal_x').value)
        self.reference_goal_y = float(
            self.get_parameter('reference_goal_y').value)
        self.reference_goal_yaw_deg = float(
            self.get_parameter('reference_goal_yaw_deg').value)
        self.reference_goal_match_radius = float(
            self.get_parameter('reference_goal_match_radius').value)

        os.makedirs(output_directory, exist_ok=True)
        self.csv_path = os.path.join(output_directory, 'goal_results.csv')
        self.latest_path = os.path.join(output_directory, 'latest_result.txt')

        self.latest_goal = None
        self.latest_localization = None
        self.latest_goal_reference_ns = 0
        self.latest_goal_has_zero_stamp = False
        self.latest_goal_from_plan = False
        self.latest_goal_action_time_unbound = False
        self.goal_pending = False
        self.pending_terminal = None
        self.seen_terminal_goal_ids = set()

        goal_qos = 10
        if self.goal_best_effort_qos:
            goal_qos = QoSProfile(depth=10)
            goal_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(PoseStamped, goal_topic, self.on_goal, goal_qos)
        self.create_subscription(Path, plan_topic, self.on_plan, 10)
        self.create_subscription(
            Odometry, localization_topic, self.on_localization, 20)
        self.create_subscription(
            GoalStatusArray, status_topic, self.on_status, 10)
        self.create_timer(0.05, self.check_pending_terminal)

        self.get_logger().info(
            f'HY goal error recorder started; CSV: {self.csv_path}')

    def on_goal(self, msg):
        self.capture_goal(msg, recovered_from_plan=False)

    def on_plan(self, msg):
        """Recover the intended Goal when a one-shot /goal_pose was missed.

        SmacPlanner paths terminate at the NavigateToPose goal. Only the first
        plan of a new action is used; later replans must not reset Goal state.
        """
        if self.goal_pending or not msg.poses:
            return
        endpoint = msg.poses[-1]
        recovered = PoseStamped()
        recovered.header = endpoint.header
        if not recovered.header.frame_id:
            recovered.header.frame_id = msg.header.frame_id
        recovered.pose = endpoint.pose
        if self.reference_goal_enabled:
            distance = math.hypot(
                recovered.pose.position.x - self.reference_goal_x,
                recovered.pose.position.y - self.reference_goal_y)
            if distance <= self.reference_goal_match_radius:
                recovered.header.stamp.sec = 0
                recovered.header.stamp.nanosec = 0
                recovered.pose.position.x = self.reference_goal_x
                recovered.pose.position.y = self.reference_goal_y
                yaw = math.radians(self.reference_goal_yaw_deg)
                recovered.pose.orientation.x = 0.0
                recovered.pose.orientation.y = 0.0
                recovered.pose.orientation.z = math.sin(yaw * 0.5)
                recovered.pose.orientation.w = math.cos(yaw * 0.5)
                self.capture_goal(
                    # 설정된 KTL 기준점으로 복구했으므로 측정 기준은
                    # 유효하다. 다만 /plan은 action accepted 뒤에 오므로
                    # accepted 시각 매칭은 하지 않는다.
                    recovered, recovered_from_plan=False,
                    action_time_unbound=True,
                    source_override='configured KTL reference via /plan')
                return
        self.capture_goal(recovered, recovered_from_plan=True)

    def capture_goal(
            self, msg, recovered_from_plan, source_override=None,
            action_time_unbound=False):
        self.latest_goal = msg
        goal_stamp = stamp_ns(msg.header.stamp)
        self.latest_goal_has_zero_stamp = goal_stamp == 0
        self.latest_goal_from_plan = recovered_from_plan
        self.latest_goal_action_time_unbound = (
            action_time_unbound or recovered_from_plan)
        self.latest_goal_reference_ns = (
            goal_stamp if goal_stamp > 0 else self.get_clock().now().nanoseconds)
        self.goal_pending = True
        self.pending_terminal = None
        yaw_deg = math.degrees(yaw_from_quaternion(msg.pose.orientation))
        source = source_override or (
            '/plan endpoint fallback' if recovered_from_plan else '/goal_pose')
        self.get_logger().info(
            f'Goal captured from {source}: frame={msg.header.frame_id}, '
            f'x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, '
            f'yaw={yaw_deg:.2f} deg')

    def on_localization(self, msg):
        self.latest_localization = msg
        if self.pending_terminal is None:
            return
        terminal_ns = self.pending_terminal['terminal_ns']
        if stamp_ns(msg.header.stamp) >= terminal_ns:
            pending = self.pending_terminal
            self.pending_terminal = None
            self.record(
                pending['result'], pending['goal_id'], terminal_ns,
                require_post_result_sample=True)

    def check_pending_terminal(self):
        if self.pending_terminal is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns < self.pending_terminal['deadline_ns']:
            return
        pending = self.pending_terminal
        self.pending_terminal = None
        self.get_logger().error(
            'Timed out waiting for a localization sample stamped after Goal success')
        self.record(
            pending['result'], pending['goal_id'], pending['terminal_ns'],
            require_post_result_sample=True)

    def on_status(self, msg):
        terminal_names = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }

        # 같은 목표를 다시 전송하면 status 배열에는 종료된 이전 goal과 실행 중인
        # 새 goal이 함께 들어온다. 이때 이전 ABORTED/CANCELED를 시험 결과로
        # 저장하면 실제로 뒤에서 성공한 새 goal을 영구히 놓치게 된다.
        active_goal_ids = {
            bytes(item.goal_info.goal_id.uuid).hex()
            for item in msg.status_list
            if item.status in (
                GoalStatus.STATUS_ACCEPTED,
                GoalStatus.STATUS_EXECUTING,
                GoalStatus.STATUS_CANCELING)
        }

        for status in msg.status_list:
            if status.status not in terminal_names:
                continue

            goal_id = bytes(status.goal_info.goal_id.uuid).hex()
            if goal_id in self.seen_terminal_goal_ids:
                continue

            if active_goal_ids and goal_id not in active_goal_ids:
                self.seen_terminal_goal_ids.add(goal_id)
                self.get_logger().info(
                    f'Ignoring terminal result of superseded goal {goal_id}; '
                    'a replacement goal is still active')
                continue

            # 액션 상태 토픽은 transient history에 이전 목표도 담을 수 있다.
            # 새 /goal_pose가 들어온 뒤 처음 관측된 종료 상태만 그 목표와 묶는다.
            if not self.goal_pending:
                # Goal 대기 전에 관측한 과거 종료 ID는 이후 새 Goal에 붙지
                # 않도록 여기서 소비한다.
                self.seen_terminal_goal_ids.add(goal_id)
                continue
            accepted_ns = stamp_ns(status.goal_info.stamp)
            match_slop_ns = (
                self.zero_stamp_match_slop_ns
                if self.latest_goal_has_zero_stamp else 0)
            if (not self.latest_goal_action_time_unbound
                    and accepted_ns + match_slop_ns
                    < self.latest_goal_reference_ns):
                # 현재 Goal보다 확실히 오래된 종료 상태다. 시간 검증 후에만
                # seen 처리하여 zero-stamp 콜백 순서 역전으로 현재 결과를
                # 영구적으로 놓치지 않게 한다.
                self.seen_terminal_goal_ids.add(goal_id)
                continue

            # 현재 /goal_pose와 시간상 매칭된 종료 결과만 완료 처리한다.
            self.seen_terminal_goal_ids.add(goal_id)

            result = terminal_names[status.status]
            terminal_ns = self.get_clock().now().nanoseconds
            self.goal_pending = False
            if result == 'SUCCEEDED':
                if (self.latest_localization is not None
                        and stamp_ns(self.latest_localization.header.stamp)
                        >= terminal_ns):
                    self.record(
                        result, goal_id, terminal_ns,
                        require_post_result_sample=True)
                else:
                    self.pending_terminal = {
                        'result': result,
                        'goal_id': goal_id,
                        'terminal_ns': terminal_ns,
                        'deadline_ns': terminal_ns
                        + int(self.fresh_sample_wait_ms * 1_000_000.0),
                    }
                    self.get_logger().info(
                        'Goal succeeded; waiting for the first localization '
                        'sample stamped after success')
            else:
                self.record(result, goal_id, terminal_ns)

    def record(
            self, result, goal_id, terminal_ns,
            require_post_result_sample=False):
        if self.latest_goal is None:
            self.get_logger().warning(
                f'{result} received but no /goal_pose has been captured')
            return
        if self.latest_localization is None:
            self.get_logger().error(
                f'{result} received but no /localization is available; not recorded')
            return

        goal = self.latest_goal
        localization = self.latest_localization
        actual = localization.pose.pose

        dx_m = actual.position.x - goal.pose.position.x
        dy_m = actual.position.y - goal.pose.position.y
        error_mm = math.hypot(dx_m, dy_m) * 1000.0
        goal_yaw = yaw_from_quaternion(goal.pose.orientation)
        actual_yaw = yaw_from_quaternion(actual.orientation)
        yaw_error_deg = math.degrees(normalized_angle(actual_yaw - goal_yaw))

        now = self.get_clock().now().to_msg()
        terminal_stamp = Time(nanoseconds=terminal_ns).to_msg()
        localization_stamp_ns = stamp_ns(localization.header.stamp)
        localization_age_ms = (
            stamp_ns(now) - localization_stamp_ns) / 1_000_000.0
        frames_match = (
            bool(goal.header.frame_id)
            and goal.header.frame_id == localization.header.frame_id)
        localization_is_fresh = (
            -50.0 <= localization_age_ms <= self.max_localization_age_ms)
        post_result_sample = (
            not require_post_result_sample
            or localization_stamp_ns >= terminal_ns)
        goal_reference_valid = (
            self.allow_plan_fallback_measurement
            or not self.latest_goal_from_plan)
        measurement_valid = (
            frames_match and localization_is_fresh and post_result_sample
            and goal_reference_valid)
        validation_notes = []
        if not frames_match:
            validation_notes.append('FRAME_MISMATCH')
        if not localization_is_fresh:
            validation_notes.append('STALE_LOCALIZATION')
        if not post_result_sample:
            validation_notes.append('PRE_RESULT_LOCALIZATION')
        if not goal_reference_valid:
            validation_notes.append('GOAL_FROM_PLAN_FALLBACK')
        validation_note = 'OK' if not validation_notes else '|'.join(validation_notes)
        passed = (
            measurement_valid
            and result == 'SUCCEEDED'
            and error_mm <= self.position_limit_mm)
        if not measurement_valid:
            pass_text = 'INVALID'
        else:
            pass_text = 'PASS' if passed else 'FAIL'

        row = {
            'recorded_at': datetime.now().astimezone().isoformat(timespec='milliseconds'),
            'result': result,
            'goal_id': goal_id,
            'goal_stamp_sec': goal.header.stamp.sec,
            'goal_stamp_nanosec': goal.header.stamp.nanosec,
            'success_stamp_sec': terminal_stamp.sec,
            'success_stamp_nanosec': terminal_stamp.nanosec,
            'localization_stamp_sec': localization.header.stamp.sec,
            'localization_stamp_nanosec': localization.header.stamp.nanosec,
            'localization_sample_age_ms': f'{localization_age_ms:.3f}',
            'measurement_valid': 'YES' if measurement_valid else 'NO',
            'validation_note': validation_note,
            'goal_frame': goal.header.frame_id,
            'localization_frame': localization.header.frame_id,
            'goal_x_m': f'{goal.pose.position.x:.9f}',
            'goal_y_m': f'{goal.pose.position.y:.9f}',
            'goal_yaw_deg': f'{math.degrees(goal_yaw):.6f}',
            'actual_x_m': f'{actual.position.x:.9f}',
            'actual_y_m': f'{actual.position.y:.9f}',
            'actual_yaw_deg': f'{math.degrees(actual_yaw):.6f}',
            'error_x_mm': f'{dx_m * 1000.0:.3f}',
            'error_y_mm': f'{dy_m * 1000.0:.3f}',
            'position_error_mm': f'{error_mm:.3f}',
            'yaw_error_deg': f'{yaw_error_deg:.6f}',
            'pass_70mm': pass_text,
        }

        file_exists = os.path.exists(self.csv_path)
        with open(self.csv_path, 'a', newline='', encoding='utf-8') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            if not file_exists or os.path.getsize(self.csv_path) == 0:
                writer.writeheader()
            writer.writerow(row)
            csv_file.flush()
            os.fsync(csv_file.fileno())

        with open(self.latest_path, 'w', encoding='utf-8') as latest_file:
            for key in CSV_FIELDS:
                latest_file.write(f'{key}: {row[key]}\n')

        self.get_logger().info(
            f'Goal result saved: {result}, error={error_mm:.1f} mm, '
            f'yaw_error={yaw_error_deg:.2f} deg, '
            f'70 mm={row["pass_70mm"]} -> {self.csv_path}')


def main(args=None):
    rclpy.init(args=args)
    node = GoalErrorRecorderHY()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
