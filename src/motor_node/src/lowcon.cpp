// lowcon.cpp
//
//   Node 1 = FRONT_DRIVE   PV mode(3), 0x60FF
//   Node 2 = FRONT_STEER   PV mode(3), 0x60FF + 소프트웨어 각도 P 제어
//   Node 3 = REAR_DRIVE    PV mode(3), 0x60FF
//   Node 4 = REAR_STEER    PV mode(3), 0x60FF + 소프트웨어 각도 P 제어
//
// 토픽
//   /mode                          std_msgs/Int32  0=재정렬, 1=애커만, 2=디프
//   /cmd_vel                       geometry_msgs/Twist
//   /odom                          nav_msgs/Odometry           (발행)
//   /steer_angle_deg               std_msgs/Float32MultiArray  (발행)
//                                    [전륜실제, 후륜실제, 전륜지령, 후륜지령]
//   ~/diagnostics                  diagnostic_msgs/DiagnosticArray (발행)
//   ~/initialization_status        std_msgs/String  latched, "STATE|detail"
//
// 서비스
//   ~/initialize                   std_srvs/Trigger  4축 재초기화 + 영점 재계산
//   ~/reset_odom                   std_srvs/Trigger  오도메트리 원점 리셋
//
// ── 오도메트리 (모드 무관 단일 모델) ─────────────────────────────
//   전/후 축을 각각 조향 가능한 2축 플랫폼으로 보고, 측정된 조향각과
//   측정된 휠 속도로부터 몸체 속도를 직접 역산한다. 
//
//     접지점  앞 p_f = (+L/2, 0),  뒤 p_r = (-L/2, 0)
//     강체    v_wheel = v_body + omega x p
//
//       vx           = v_f cos(df)      vx           = v_r cos(dr)
//       vy + w*L/2   = v_f sin(df)      vy - w*L/2   = v_r sin(dr)
//
//     최소자승해 (앞뒤 두 축의 관측을 모두 사용)
//       vx = ( v_f cos(df) + v_r cos(dr) ) / 2
//       vy = ( v_f sin(df) + v_r sin(dr) ) / 2
//       w  = ( v_f sin(df) - v_r sin(dr) ) / L
//
// 감속비 보정
//   조향비가 틀리면 지령 각도와 실제 각도가 어긋납니다. 실측 후
//     steer_ratio_true = steer_ratio_used * (지령각 / 실제각)
//   으로 계산해 steer_ratio 파라미터에 넣으면 재빌드 없이 반영됩니다.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

#define CAN_INTERFACE   "can0"

// 모터 ID 정의
#define ID_FRONT_DRIVE  1
#define ID_FRONT_STEER  2
#define ID_REAR_DRIVE   3
#define ID_REAR_STEER   4

// 로봇 물리 정보 기본값 (파라미터로 덮어쓸 수 있다)
#define DEF_WHEEL_CIRCUM  0.47124   // 바퀴 둘레 [m] (0.15 * pi)
#define DEF_WHEELBASE     1.29      // 축간거리 [m]

// 조향각 제한 기본값
#define DEF_MAX_STEER_LEFT_DEG   45.0
#define DEF_MAX_STEER_RIGHT_DEG  45.0

// 속도 제한 기본값
#define DEF_MAX_LINEAR_VEL  2.0
#define DEF_MAX_ANGULAR_VEL 1.0

// 가감속 기본값
#define DRIVE_ACCEL     100000
#define DRIVE_DECEL     500000
#define STEER_ACCEL     200000
#define STEER_DECEL     1000000   // 조향만 더 크게

// CANopen Objects
#define NMT_START       0x01
#define OD_ERROR_CODE   0x603F
#define OD_CONTROL_WORD 0x6040
#define OD_STATUS_WORD  0x6041
#define OD_MODES_OF_OP  0x6060
#define OD_POS_INTERNAL 0x6063
#define OD_POS_ACTUAL   0x6064
#define OD_PROFILE_ACC  0x6083
#define OD_PROFILE_DEC  0x6084
#define OD_ENC_RES      0x608F
#define OD_FEED_CONST   0x6092
#define OD_TARGET_VEL   0x60FF

#define DEFAULT_ENC_RES 131072
#define DEFAULT_FEED    10000

namespace {

constexpr uint16_t SW_STATE_MASK        = 0x006F;
constexpr uint16_t SW_OPERATION_ENABLED = 0x0027;
constexpr uint16_t SW_FAULT_BIT         = 1U << 3;
constexpr uint16_t SW_INTERNAL_LIMIT    = 1U << 11;

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

// 복구 상태기계 타이밍 [ms]
constexpr int RECOVERY_ERROR_READ_MS        = 250;
constexpr int RECOVERY_STEP_MS              = 60;
constexpr int RECOVERY_FAULT_CLEAR_TIMEOUT  = 1500;
constexpr int RECOVERY_ENABLE_TIMEOUT       = 1500;

double normalize_angle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

} // namespace

enum class Mode : int { ALIGN = 0, ACKERMANN = 1, DIFF = 2 };

struct AxisCfg {
    int32_t enc  = DEFAULT_ENC_RES;   // 0x608F-01 엔코더 분해능 [cnt/rev]
    int32_t feed = DEFAULT_FEED;      // 각도/속도 환산에 쓰는 유효 pulse/rev (override 가능)
    int32_t feed_meas = DEFAULT_FEED;  // 드라이브가 실제 보고한 0x6092-01.
                                       // 6063 <-> 6064 단위 환산에는 반드시 이 값을 쓴다.
};

// 축 상태 (motor_node 의 MotorStatus 를 이식)
struct MotorState {
    uint16_t status_word = 0;
    uint16_t error_code  = 0;
    bool     error_code_valid   = false;
    bool     error_read_pending = false;
    bool     tpdo_received      = false;
    std::chrono::steady_clock::time_point last_tpdo{};
};

enum class Recovery {
    IDLE,
    WAIT_ERROR_CODE,
    WAIT_RESET_LOW,
    WAIT_FAULT_CLEAR,
    WAIT_SHUTDOWN,
    WAIT_SWITCH_ON,
    WAIT_ENABLE,
    REFRESH_ZERO,
    FAILED
};

class MotorNode : public rclcpp::Node {
public:
    MotorNode() : Node("lowcon"), can_sock_(-1) {
        declare_all_parameters();

        // ── ROS 인터페이스는 CAN 보다 먼저 만든다.
        //    CAN 초기화가 실패해도 상위에서 진단/상태를 볼 수 있어야 한다.
        sub_mode_ = this->create_subscription<std_msgs::msg::Int32>(
            "mode", 10, std::bind(&MotorNode::mode_cb, this, std::placeholders::_1));
        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&MotorNode::cmd_vel_cb, this, std::placeholders::_1));
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        pub_steer_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "steer_angle_deg", 10);

        auto latched = rclcpp::QoS(rclcpp::KeepLast(1));
        latched.reliable();
        latched.transient_local();
        pub_diag_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "~/diagnostics", latched);
        pub_init_status_ = this->create_publisher<std_msgs::msg::String>(
            "~/initialization_status", latched);

        srv_initialize_ = this->create_service<std_srvs::srv::Trigger>(
            "~/initialize",
            std::bind(&MotorNode::initialize_cb, this,
                      std::placeholders::_1, std::placeholders::_2));
        srv_reset_odom_ = this->create_service<std_srvs::srv::Trigger>(
            "~/reset_odom",
            std::bind(&MotorNode::reset_odom_cb, this,
                      std::placeholders::_1, std::placeholders::_2));

        if (publish_tf_)
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        diag_timer_ = this->create_wall_timer(
            250ms, std::bind(&MotorNode::publish_diagnostics, this));

        if (init_can_socket() < 0) {
            RCLCPP_ERROR(this->get_logger(), "CAN Socket Init Failed!");
            // 진단/상태는 계속 내보내 상위에서 실패를 알 수 있게 한다.
            set_init_status("FAILED", "CAN socket initialization failed");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Initializing Motors...");
        set_init_status("INITIALIZING", "Reading axis configuration");

        for (int id : {ID_FRONT_DRIVE, ID_REAR_DRIVE, ID_FRONT_STEER, ID_REAR_STEER}) {
            send_nmt_command(NMT_START, id);
            usleep(20000);
            AxisCfg c; int32_t v;
            if (sdo_read(id, OD_ENC_RES,    0x01, v) && v > 0) c.enc  = v;
            if (sdo_read(id, OD_FEED_CONST, 0x01, v) && v > 0) c.feed = c.feed_meas = v;

            // 파라미터로 강제 지정된 경우 덮어쓴다
            bool is_steer = is_steer_axis(id);
            int  ovr = is_steer ? pulse_per_rev_steer_ : pulse_per_rev_drive_;
            if (ovr > 0) {
                RCLCPP_WARN(this->get_logger(),
                    "Node %d pulse/rev 강제 지정: %d (실측 %d)", id, ovr, c.feed);
                c.feed = ovr;
            }
            cfg_[id] = c;
            RCLCPP_INFO(this->get_logger(),
                "Node %d: enc=%d cnt/rev, feed(실측)=%d, feed(적용)=%d pulse/rev%s",
                id, c.enc, c.feed_meas, c.feed,
                is_steer ? "" : "");
            if (is_steer)
                RCLCPP_INFO(this->get_logger(),
                    "  -> 조향 출력축 1회전 = %.0f pulse (feed x steer_ratio). "
                    "motor_node 기준값은 1310720 이다.",
                    (double)c.feed * steer_ratio_);
        }

        // 1. 초기화 (모든 모터를 속도 제어 모드 3으로 통일)
        for (int id : all_axes()) init_motor_sdo(id, 3);

        // 2. RPDO 매핑
        for (int id : all_axes()) setup_rpdo_mapping(id);

        // 3. TPDO 매핑 (주행: 실제 속도 0x606C, 조향: 실제 위치 0x6064)
        setup_tpdo_mapping(ID_FRONT_DRIVE, 0x606C0020);
        setup_tpdo_mapping(ID_REAR_DRIVE,  0x606C0020);
        setup_tpdo_mapping(ID_FRONT_STEER, 0x60640020);
        setup_tpdo_mapping(ID_REAR_STEER,  0x60640020);

        // 4. 모터 시작
        for (int id : all_axes()) send_nmt_command(NMT_START, id);

        // 5. 조향 영점 계산 (모터를 움직이지 않음)
        compute_steer_user_zero();

        // ── 제어 루프 기동
        last_cmd_time_ = this->now();
        last_time_     = this->now();
        timer_ = this->create_wall_timer(20ms, std::bind(&MotorNode::control_loop, this));

        // 런타임 파라미터 변경 반영 (ros2 param set)
        param_cb_ = this->add_on_set_parameters_callback(
            std::bind(&MotorNode::on_param_change, this, std::placeholders::_1));

        mode_ = Mode::ALIGN;
        if (zero_ready_)
            set_init_status("READY", "Zero reference acquired; waiting for mode command");
        else
            set_init_status("FAILED", "Steering zero reference unavailable");

        RCLCPP_INFO(this->get_logger(),
            "lowcon 시작. steer_ratio=%.1f:1, wheelbase=%.3fm, 조향 제한 -%.0f~+%.0f도, "
            "mode 0(정렬)부터 시작", steer_ratio_, wheelbase_,
            max_steer_right_deg_, max_steer_left_deg_);
    }

    ~MotorNode() override {
        if (can_sock_ >= 0) {
            for (int id : all_axes()) send_pdo_command(id, 0, 0x000F);
            usleep(50000);
            for (int id : all_axes()) send_sdo_write(id, OD_CONTROL_WORD, 0, 0x0006, 2);
            usleep(20000);
            send_nmt_command(0x81, 0);
            close(can_sock_);
        }
    }

private:
    // ───────────────────────────── 멤버
    int can_sock_;
    std::map<int, AxisCfg> cfg_;
    std::array<MotorState, 5> motor_{};   // CANopen node ID 1~4 사용

    // 조향
    double steer_ratio_ = 120.0, steer_gain_ = 1.3;
    double max_steer_rate_dps_ = 20.0, steer_deadband_deg_ = 0.3;
    double steer_settle_deg_ = 1.0, diff_steer_deg_ = 90.0, max_steer_abs_deg_ = 95.0;
    double steer_min_rate_dps_ = 2.0, deadband_exit_ratio_ = 3.0;
    double max_steer_left_deg_ = DEF_MAX_STEER_LEFT_DEG;
    double max_steer_right_deg_ = DEF_MAX_STEER_RIGHT_DEG;
    int    steer_accel_ = STEER_ACCEL, steer_decel_ = STEER_DECEL;

    // 주행
    double drive_ratio_ = 1.0;
    int    drive_accel_ = DRIVE_ACCEL, drive_decel_ = DRIVE_DECEL;
    double max_linear_vel_ = DEF_MAX_LINEAR_VEL, max_angular_vel_ = DEF_MAX_ANGULAR_VEL;

    // 기구
    double wheelbase_ = DEF_WHEELBASE, wheel_circum_ = DEF_WHEEL_CIRCUM;

    int    pulse_per_rev_steer_ = 0, pulse_per_rev_drive_ = 0;
    bool   in_deadband_[5] = {false, false, false, false, false};
    int64_t front_steer_zero_ = 0, rear_steer_zero_ = 0;
    bool have_zero_ = false, diff_lateral_ = true;
    bool hold_steer_on_timeout_ = true;
    double ack_stationary_gain_ = 30.0;
    bool   steer_hold_until_mode_cmd_ = false;   // 첫 /mode 수신 전까지 조향 정지
    bool   mode_cmd_received_ = false;
    double cmd_vel_timeout_s_ = 0.5;
    double diff_steer_front_deg_ = 0.0, diff_steer_rear_deg_ = 0.0;   // 0 = diff_steer_deg
    double diff_max_linear_vel_ = 0.0, diff_max_angular_vel_ = 0.0;   // 0 = 전역 제한 사용

    // 안전
    int    feedback_timeout_ms_ = 300;
    bool   require_motor_ready_ = true;
    bool   auto_recovery_enabled_ = true;

    // 오도메트리
    std::string odom_frame_ = "odom", base_frame_ = "base_link";
    bool   publish_tf_ = true;
    double odom_v_deadband_ = 0.005;   // [m/s] 이하 휠속도는 0 으로 본다

    Mode mode_ = Mode::ALIGN;
    bool zero_ready_ = false;
    bool await_settle_ = true;      // 모드 전환 직후 한 번만 조향 정착을 기다림
    double ack_delta_deg_ = 0.0;    // 애커만 전륜 조향각 유지값

    std::map<int, double> steer_user_zero_;      // 조향 0도일 때의 0x6064 (소수 허용)
    double steer_cmd_deg_[5]     = {0, 0, 0, 0, 0};
    double current_steer_deg_[5] = {0, 0, 0, 0, 0};

    double cmd_vx_ = 0.0, cmd_vy_ = 0.0, cmd_wz_ = 0.0;

    // 포즈 / 측정 트위스트
    double x_ = 0.0, y_ = 0.0, th_ = 0.0;
    double meas_vx_ = 0.0, meas_vy_ = 0.0, meas_wz_ = 0.0;
    bool   odom_valid_ = false;
    double drive_vel_[5] = {0, 0, 0, 0, 0};   // 축별 휠 접지속도 [m/s]

    // 진단 / 복구
    int  stall_cnt_[5] = {0, 0, 0, 0, 0};
    double last_steer_deg_[5] = {0, 0, 0, 0, 0};
    Recovery recovery_ = Recovery::IDLE;
    std::chrono::steady_clock::time_point recovery_deadline_{};
    bool recovery_requested_ = false;     // 서비스로 요청된 초기화
    bool recovery_all_axes_ = false;
    bool auto_recovery_armed_ = true;     // 1회성 자동 복구
    bool motion_inhibited_ = false;
    bool awaiting_neutral_cmd_ = false;
    std::string init_state_ = "PENDING", init_detail_ = "Waiting for initialization";

    rclcpp::Time last_cmd_time_, last_time_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_mode_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_steer_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_init_status_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_initialize_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_odom_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr diag_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    static const std::vector<int> &all_axes() {
        static const std::vector<int> v{ID_FRONT_DRIVE, ID_REAR_DRIVE,
                                        ID_FRONT_STEER, ID_REAR_STEER};
        return v;
    }
    static const std::vector<int> &steer_axes() {
        static const std::vector<int> v{ID_FRONT_STEER, ID_REAR_STEER};
        return v;
    }
    static const std::vector<int> &drive_axes() {
        static const std::vector<int> v{ID_FRONT_DRIVE, ID_REAR_DRIVE};
        return v;
    }
    static bool is_steer_axis(int id) {
        return id == ID_FRONT_STEER || id == ID_REAR_STEER;
    }

    // ───────────────────────────── 파라미터 선언
    void declare_all_parameters() {
        // 조향
        steer_ratio_        = this->declare_parameter<double>("steer_ratio", 120.0);
        steer_gain_         = this->declare_parameter<double>("steer_angle_tracking_gain", 1.3);
        max_steer_rate_dps_ = this->declare_parameter<double>("max_steer_rate_dps", 20.0);
        steer_deadband_deg_ = this->declare_parameter<double>("steer_angle_deadband_deg", 0.3);
        steer_settle_deg_   = this->declare_parameter<double>("steer_settle_deg", 1.0);
        diff_steer_deg_     = this->declare_parameter<double>("diff_steer_deg", 90.0);
        max_steer_abs_deg_  = this->declare_parameter<double>("max_steer_abs_deg", 95.0);
        steer_min_rate_dps_ = this->declare_parameter<double>("steer_min_rate_dps", 2.0);
        deadband_exit_ratio_ =
            this->declare_parameter<double>("steer_deadband_exit_ratio", 3.0);
        max_steer_left_deg_ =
            this->declare_parameter<double>("max_steer_left_deg", DEF_MAX_STEER_LEFT_DEG);
        max_steer_right_deg_ =
            this->declare_parameter<double>("max_steer_right_deg", DEF_MAX_STEER_RIGHT_DEG);
        steer_accel_        = this->declare_parameter<int>("steer_accel", STEER_ACCEL);
        steer_decel_        = this->declare_parameter<int>("steer_decel", STEER_DECEL);

        // 주행축은 조향축과 별개로 관리한다
        drive_ratio_        = this->declare_parameter<double>("drive_ratio", 1.0);
        drive_accel_        = this->declare_parameter<int>("drive_accel", DRIVE_ACCEL);
        drive_decel_        = this->declare_parameter<int>("drive_decel", DRIVE_DECEL);

        // 속도 상한 (기존에는 매크로였고 yaml 값이 무시되었다)
        max_linear_vel_  = this->declare_parameter<double>("max_linear_vel", DEF_MAX_LINEAR_VEL);
        max_angular_vel_ = this->declare_parameter<double>("max_angular_vel", DEF_MAX_ANGULAR_VEL);

        // 기구 치수
        wheelbase_    = this->declare_parameter<double>("wheelbase", DEF_WHEELBASE);
        wheel_circum_ = this->declare_parameter<double>("wheel_circumference", DEF_WHEEL_CIRCUM);

        // 0 이면 0x6092-01 실측값 사용. 0 이 아니면 그 값으로 강제
        pulse_per_rev_steer_ = this->declare_parameter<int>("pulse_per_rev_steer", 0);
        pulse_per_rev_drive_ = this->declare_parameter<int>("pulse_per_rev_drive", 0);

        front_steer_zero_ = this->declare_parameter<int64_t>("front_steer_zero", 0);
        rear_steer_zero_  = this->declare_parameter<int64_t>("rear_steer_zero", 0);
        have_zero_        = this->declare_parameter<bool>("have_steer_zero", false);
        diff_lateral_     = this->declare_parameter<bool>("diff_linear_x_is_lateral", true);
        hold_steer_on_timeout_ =
            this->declare_parameter<bool>("hold_steer_on_cmd_timeout", true);
        ack_stationary_gain_ =
            this->declare_parameter<double>("ackermann_stationary_steer_gain", 30.0);
        steer_hold_until_mode_cmd_ =
            this->declare_parameter<bool>("steer_hold_until_mode_cmd", false);
        cmd_vel_timeout_s_ = this->declare_parameter<double>("cmd_vel_timeout_s", 0.5);

        // 디프(도킹) 모드. motor_node 의 docking_front_pulse / docking_rear_pulse 를
        // 각도로 옮긴 값이다. 0 이면 diff_steer_deg 를 전후륜 공통으로 쓴다.
        diff_steer_front_deg_ = this->declare_parameter<double>("diff_steer_front_deg", 0.0);
        diff_steer_rear_deg_  = this->declare_parameter<double>("diff_steer_rear_deg", 0.0);
        diff_max_linear_vel_  = this->declare_parameter<double>("diff_max_linear_vel", 0.0);
        diff_max_angular_vel_ = this->declare_parameter<double>("diff_max_angular_vel", 0.0);

        // 안전
        feedback_timeout_ms_ = this->declare_parameter<int>("feedback_timeout_ms", 300);
        require_motor_ready_ = this->declare_parameter<bool>("require_motor_ready", true);
        auto_recovery_enabled_ =
            this->declare_parameter<bool>("steer_auto_recovery", true);

        // 오도메트리
        odom_frame_      = this->declare_parameter<std::string>("odom_frame_id", "odom");
        base_frame_      = this->declare_parameter<std::string>("base_frame_id", "base_link");
        publish_tf_      = this->declare_parameter<bool>("publish_odom_tf", true);
        odom_v_deadband_ = this->declare_parameter<double>("odom_wheel_deadband_mps", 0.005);

        // 구 파라미터. 휠 속도 기반 정기구학에서는 sin 형태가 정확해 무시된다.
        if (this->declare_parameter<bool>("use_tan_yaw_rate_model", false)) {
            RCLCPP_WARN(this->get_logger(),
                "use_tan_yaw_rate_model 은 더 이상 사용되지 않습니다. "
                "오도메트리는 측정 휠속도 기반 4WS 정기구학으로 계산됩니다.");
        }

        if (wheelbase_ <= 0.0)    wheelbase_ = DEF_WHEELBASE;
        if (wheel_circum_ <= 0.0) wheel_circum_ = DEF_WHEEL_CIRCUM;
    }

    // ───────────────────────────── 런타임 파라미터 반영
    rcl_interfaces::msg::SetParametersResult
    on_param_change(const std::vector<rclcpp::Parameter> &params) {
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        for (const auto &p : params) {
            const std::string &n = p.get_name();
            if (n == "steer_ratio") {
                double v = p.as_double();
                if (v <= 0.0) { res.successful = false; res.reason = "steer_ratio > 0"; continue; }
                steer_ratio_ = v;
                RCLCPP_INFO(this->get_logger(), "steer_ratio -> %.3f", v);
            } else if (n == "steer_angle_tracking_gain") {
                steer_gain_ = p.as_double();
            } else if (n == "max_steer_rate_dps") {
                max_steer_rate_dps_ = p.as_double();
            } else if (n == "steer_angle_deadband_deg") {
                steer_deadband_deg_ = p.as_double();
            } else if (n == "steer_settle_deg") {
                steer_settle_deg_ = p.as_double();
            } else if (n == "diff_steer_deg") {
                diff_steer_deg_ = p.as_double();
                RCLCPP_INFO(this->get_logger(), "diff_steer_deg -> %.2f", diff_steer_deg_);
            } else if (n == "max_steer_abs_deg") {
                max_steer_abs_deg_ = p.as_double();
            } else if (n == "max_steer_left_deg") {
                max_steer_left_deg_ = p.as_double();
            } else if (n == "max_steer_right_deg") {
                max_steer_right_deg_ = p.as_double();
            } else if (n == "steer_min_rate_dps") {
                steer_min_rate_dps_ = p.as_double();
            } else if (n == "steer_deadband_exit_ratio") {
                deadband_exit_ratio_ = std::max(1.0, p.as_double());
            } else if (n == "steer_accel") {
                steer_accel_ = p.as_int();
                for (int id : steer_axes())
                    send_sdo_write(id, OD_PROFILE_ACC, 0, steer_accel_, 4);
                RCLCPP_INFO(this->get_logger(), "steer_accel -> %d", steer_accel_);
            } else if (n == "steer_decel") {
                steer_decel_ = p.as_int();
                for (int id : steer_axes())
                    send_sdo_write(id, OD_PROFILE_DEC, 0, steer_decel_, 4);
                RCLCPP_INFO(this->get_logger(), "steer_decel -> %d", steer_decel_);
            } else if (n == "drive_ratio") {
                double v = p.as_double();
                if (v <= 0.0) { res.successful = false; res.reason = "drive_ratio > 0"; continue; }
                drive_ratio_ = v;
                RCLCPP_INFO(this->get_logger(), "drive_ratio -> %.3f", v);
            } else if (n == "drive_accel") {
                drive_accel_ = p.as_int();
                for (int id : drive_axes())
                    send_sdo_write(id, OD_PROFILE_ACC, 0, drive_accel_, 4);
                RCLCPP_INFO(this->get_logger(), "drive_accel -> %d", drive_accel_);
            } else if (n == "drive_decel") {
                drive_decel_ = p.as_int();
                for (int id : drive_axes())
                    send_sdo_write(id, OD_PROFILE_DEC, 0, drive_decel_, 4);
                RCLCPP_INFO(this->get_logger(), "drive_decel -> %d", drive_decel_);
            } else if (n == "max_linear_vel") {
                double v = p.as_double();
                if (v <= 0.0) { res.successful = false; res.reason = "max_linear_vel > 0"; continue; }
                max_linear_vel_ = v;
                RCLCPP_INFO(this->get_logger(), "max_linear_vel -> %.3f", v);
            } else if (n == "max_angular_vel") {
                double v = p.as_double();
                if (v <= 0.0) { res.successful = false; res.reason = "max_angular_vel > 0"; continue; }
                max_angular_vel_ = v;
                RCLCPP_INFO(this->get_logger(), "max_angular_vel -> %.3f", v);
            } else if (n == "wheelbase") {
                double v = p.as_double();
                if (v <= 0.0) { res.successful = false; res.reason = "wheelbase > 0"; continue; }
                wheelbase_ = v;
                RCLCPP_INFO(this->get_logger(), "wheelbase -> %.4f", v);
            } else if (n == "wheel_circumference") {
                double v = p.as_double();
                if (v <= 0.0) {
                    res.successful = false; res.reason = "wheel_circumference > 0"; continue;
                }
                wheel_circum_ = v;
                RCLCPP_INFO(this->get_logger(), "wheel_circumference -> %.5f", v);
            } else if (n == "odom_wheel_deadband_mps") {
                odom_v_deadband_ = std::max(0.0, p.as_double());
            } else if (n == "hold_steer_on_cmd_timeout") {
                hold_steer_on_timeout_ = p.as_bool();
            } else if (n == "diff_linear_x_is_lateral") {
                diff_lateral_ = p.as_bool();
            } else if (n == "ackermann_stationary_steer_gain") {
                ack_stationary_gain_ = p.as_double();
            } else if (n == "steer_hold_until_mode_cmd") {
                steer_hold_until_mode_cmd_ = p.as_bool();
            } else if (n == "cmd_vel_timeout_s") {
                cmd_vel_timeout_s_ = std::max(0.05, p.as_double());
            } else if (n == "diff_steer_front_deg") {
                diff_steer_front_deg_ = p.as_double();
                RCLCPP_INFO(this->get_logger(),
                            "diff_steer_front_deg -> %.2f", diff_steer_front_deg_);
            } else if (n == "diff_steer_rear_deg") {
                diff_steer_rear_deg_ = p.as_double();
                RCLCPP_INFO(this->get_logger(),
                            "diff_steer_rear_deg -> %.2f", diff_steer_rear_deg_);
            } else if (n == "diff_max_linear_vel") {
                diff_max_linear_vel_ = std::max(0.0, p.as_double());
            } else if (n == "diff_max_angular_vel") {
                diff_max_angular_vel_ = std::max(0.0, p.as_double());
            } else if (n == "feedback_timeout_ms") {
                feedback_timeout_ms_ = std::max(50, static_cast<int>(p.as_int()));
            } else if (n == "require_motor_ready") {
                require_motor_ready_ = p.as_bool();
            } else if (n == "steer_auto_recovery") {
                auto_recovery_enabled_ = p.as_bool();
            }
        }
        return res;
    }

    // ───────────────────────────── CAN
    int init_can_socket() {
        can_sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_sock_ < 0) return -1;

        struct ifreq ifr{};
        std::strncpy(ifr.ifr_name, CAN_INTERFACE, IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';
        if (ioctl(can_sock_, SIOCGIFINDEX, &ifr) < 0) {
            close(can_sock_); can_sock_ = -1; return -1;
        }
        struct sockaddr_can addr{};
        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(can_sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            close(can_sock_); can_sock_ = -1; return -1;
        }
        fcntl(can_sock_, F_SETFL, O_NONBLOCK);
        return 0;
    }

    void send_sdo_write(int id, uint16_t index, uint8_t sub, int32_t data, uint8_t size) {
        if (can_sock_ < 0) return;
        struct can_frame f{};
        f.can_id = 0x600 + id; f.can_dlc = 8;
        f.data[0] = (size == 1) ? 0x2F : (size == 2 ? 0x2B : 0x23);
        f.data[1] = index & 0xFF;
        f.data[2] = (index >> 8) & 0xFF;
        f.data[3] = sub;
        std::memcpy(&f.data[4], &data, 4);
        if (write(can_sock_, &f, sizeof(f)) < 0) { }
        usleep(2000);
    }

    // 논블로킹 SDO 업로드 요청 (응답은 handle_frame 에서 처리)
    bool request_sdo_read(int id, uint16_t index, uint8_t sub) {
        if (can_sock_ < 0) return false;
        struct can_frame f{};
        f.can_id = 0x600 + id; f.can_dlc = 8;
        f.data[0] = 0x40;
        f.data[1] = index & 0xFF;
        f.data[2] = (index >> 8) & 0xFF;
        f.data[3] = sub;
        return write(can_sock_, &f, sizeof(f)) > 0;
    }

    // 블로킹 SDO 업로드. 기동/복구처럼 구동이 정지된 구간에서만 쓴다.
    bool sdo_read(int id, uint16_t index, uint8_t sub, int32_t &out,
                  double timeout_s = 0.2) {
        if (can_sock_ < 0) return false;
        if (!request_sdo_read(id, index, sub)) return false;

        auto t0 = std::chrono::steady_clock::now();
        while (std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - t0).count() < timeout_s) {
            struct can_frame r{};
            if (read(can_sock_, &r, sizeof(r)) > 0) {
                if (r.can_id == (canid_t)(0x580 + id) &&
                    (uint16_t)(r.data[1] | (r.data[2] << 8)) == index) {
                    if (r.data[0] == 0x80) return false;
                    std::memcpy(&out, &r.data[4], 4);
                    return true;
                }
                handle_frame(r);
            }
            usleep(500);
        }
        return false;
    }

    void send_nmt_command(uint8_t cmd, uint8_t id) {
        if (can_sock_ < 0) return;
        struct can_frame f{};
        f.can_id = 0x000; f.can_dlc = 2;
        f.data[0] = cmd; f.data[1] = id;
        if (write(can_sock_, &f, sizeof(f)) < 0) { }
        usleep(2000);
    }

    void setup_rpdo_mapping(int id) {
        send_sdo_write(id, 0x1400, 0x01, 0x80000200 + id, 4);
        send_sdo_write(id, 0x1600, 0x00, 0, 1);
        send_sdo_write(id, 0x1600, 0x01, 0x60400010, 4);
        send_sdo_write(id, 0x1600, 0x02, 0x60FF0020, 4);
        send_sdo_write(id, 0x1600, 0x00, 2, 1);
        send_sdo_write(id, 0x1400, 0x02, 255, 1);
        send_sdo_write(id, 0x1400, 0x01, 0x00000200 + id, 4);
    }

    void setup_tpdo_mapping(int id, uint32_t mapped_obj) {
        send_sdo_write(id, 0x1800, 0x01, 0x80000180 + id, 4);
        send_sdo_write(id, 0x1A00, 0x00, 0, 1);
        send_sdo_write(id, 0x1A00, 0x01, 0x60410010, 4);
        send_sdo_write(id, 0x1A00, 0x02, static_cast<int32_t>(mapped_obj), 4);
        send_sdo_write(id, 0x1A00, 0x00, 2, 1);
        send_sdo_write(id, 0x1800, 0x02, 255, 1);
        send_sdo_write(id, 0x1800, 0x05, 20, 2);
        send_sdo_write(id, 0x1800, 0x01, 0x00000180 + id, 4);
    }

    void send_pdo_command(int id, int32_t val, uint16_t cw) {
        if (can_sock_ < 0) return;
        struct can_frame f{};
        f.can_id = 0x200 + id; f.can_dlc = 6;
        std::memcpy(&f.data[0], &cw, 2);
        std::memcpy(&f.data[2], &val, 4);
        if (write(can_sock_, &f, sizeof(f)) < 0) { }
    }

    void init_motor_sdo(int id, int mode) {
        send_sdo_write(id, OD_MODES_OF_OP, 0, mode, 1);
        if (is_steer_axis(id)) {
            send_sdo_write(id, OD_PROFILE_ACC, 0, steer_accel_, 4);
            send_sdo_write(id, OD_PROFILE_DEC, 0, steer_decel_, 4);
        } else {
            send_sdo_write(id, OD_PROFILE_ACC, 0, drive_accel_, 4);
            send_sdo_write(id, OD_PROFILE_DEC, 0, drive_decel_, 4);
        }
        send_sdo_write(id, OD_TARGET_VEL, 0, 0, 4);
        send_sdo_write(id, OD_CONTROL_WORD, 0, 0x0006, 2); usleep(50000);
        send_sdo_write(id, OD_CONTROL_WORD, 0, 0x0007, 2); usleep(50000);
        send_sdo_write(id, OD_CONTROL_WORD, 0, 0x000F, 2); usleep(50000);
    }

    // ───────────────────────────── 영점 계산 (모터 미동작)
    bool compute_steer_user_zero() {
        if (!have_zero_) {
            RCLCPP_WARN(this->get_logger(),
                "have_steer_zero=false. steer_zero.py 로 티칭한 값을 "
                "front_steer_zero / rear_steer_zero 로 넘기세요. 조향 지령을 차단합니다.");
            zero_ready_ = false;
            return false;
        }
        bool ok = true;
        std::map<int, double> zero;
        for (int id : steer_axes()) {
            int32_t pi = 0, pu = 0;
            if (!sdo_read(id, OD_POS_INTERNAL, 0, pi) ||
                !sdo_read(id, OD_POS_ACTUAL, 0, pu)) {
                RCLCPP_ERROR(this->get_logger(), "Node %d 위치 읽기 실패", id);
                ok = false; continue;
            }
            int64_t z = (id == ID_FRONT_STEER) ? front_steer_zero_ : rear_steer_zero_;
            // 6063 과 6064 는 같은 축. 영점에서의 6064 를 역산.
            // 이 환산은 드라이브 내부 단위 관계이므로 override 가 아닌 실측 feed 를 쓴다.
            double uz = (double)pu +
                        (double)(z - pi) * cfg_[id].feed_meas / cfg_[id].enc;
            zero[id] = uz;

            double cur = (double)(pi - z) * cfg_[id].feed_meas / cfg_[id].enc
                         / cfg_[id].feed * 360.0 / steer_ratio_;
            RCLCPP_INFO(this->get_logger(),
                "Node %d 영점 확보: 6063=%d, zero=%ld, 현재 조향각 %+.3f도",
                id, pi, (long)z, cur);
            if (std::fabs(cur) > max_steer_abs_deg_) {
                RCLCPP_ERROR(this->get_logger(),
                    "Node %d 현재 각도가 절대 한계 ±%.0f도를 넘습니다. "
                    "영점 또는 steer_ratio 를 확인하세요.", id, max_steer_abs_deg_);
                ok = false;
            }
        }
        ok = ok && (zero.size() == 2);
        if (ok) {
            steer_user_zero_ = zero;
            zero_ready_ = true;
            RCLCPP_INFO(this->get_logger(), "조향 영점 준비 완료");
        } else {
            zero_ready_ = false;
        }
        return ok;
    }

    // ───────────────────────────── 상태 판정
    bool has_fault(int id) const {
        return (motor_[id].status_word & SW_FAULT_BIT) != 0;
    }
    bool operation_enabled(int id) const {
        return (motor_[id].status_word & SW_STATE_MASK) == SW_OPERATION_ENABLED;
    }
    bool has_internal_limit(int id) const {
        return (motor_[id].status_word & SW_INTERNAL_LIMIT) != 0;
    }
    bool status_is_fresh(int id) const {
        if (!motor_[id].tpdo_received) return false;
        const auto age = std::chrono::steady_clock::now() - motor_[id].last_tpdo;
        return age <= std::chrono::milliseconds(feedback_timeout_ms_);
    }
    bool steer_fault_active() const {
        return has_fault(ID_FRONT_STEER) || has_fault(ID_REAR_STEER);
    }
    bool drive_fault_active() const {
        return has_fault(ID_FRONT_DRIVE) || has_fault(ID_REAR_DRIVE);
    }
    bool steer_feedback_fresh() const {
        return status_is_fresh(ID_FRONT_STEER) && status_is_fresh(ID_REAR_STEER);
    }
    bool drive_feedback_fresh() const {
        return status_is_fresh(ID_FRONT_DRIVE) && status_is_fresh(ID_REAR_DRIVE);
    }
    bool all_motors_operational() const {
        for (int id : all_axes())
            if (!status_is_fresh(id) || has_fault(id) || !operation_enabled(id)) return false;
        return true;
    }
    // 오도메트리 입력이 유효한가. 조향각과 양 축 휠속도가 모두 신선해야 한다.
    bool odom_inputs_valid() const {
        return zero_ready_ && steer_feedback_fresh() && drive_feedback_fresh();
    }

    uint16_t get_control_word(int id) {
        const uint16_t sw = motor_[id].status_word;
        // Fault 는 복구 상태기계가 담당한다. 여기서는 토크를 끊고 대기한다.
        if ((sw & SW_FAULT_BIT) != 0) return 0x0000;
        if ((sw & 0x004F) == 0x0000) return 0x0006;
        if ((sw & 0x004F) == 0x0040) return 0x0006;
        if ((sw & SW_STATE_MASK) == 0x0021) return 0x0007;
        if ((sw & SW_STATE_MASK) == 0x0023) return 0x000F;
        if ((sw & SW_STATE_MASK) == 0x0027) return 0x000F;
        if ((sw & SW_STATE_MASK) == 0x0007) return 0x0000;
        return 0x000F;
    }

    // ───────────────────────────── 수신
    void handle_frame(const struct can_frame &f) {
        const uint32_t cob = f.can_id & CAN_SFF_MASK;

        // TPDO1: statusword(2) + 속도/위치 피드백(4)
        if (cob >= 0x181 && cob <= 0x184 && f.can_dlc >= 6) {
            const int id = static_cast<int>(cob - 0x180);
            const bool prev_fault = has_fault(id);
            const bool prev_limit = has_internal_limit(id);

            std::memcpy(&motor_[id].status_word, &f.data[0], 2);
            motor_[id].tpdo_received = true;
            motor_[id].last_tpdo = std::chrono::steady_clock::now();

            int32_t raw = 0;
            std::memcpy(&raw, &f.data[2], 4);

            if (!is_steer_axis(id)) {
                // 0x606C [pulse/s] -> 휠 접지속도 [m/s]
                const double rpm = (double)raw * 60.0 / (double)cfg_[id].feed;
                drive_vel_[id] = (rpm / 60.0) * wheel_circum_ / drive_ratio_;
            } else if (steer_user_zero_.count(id)) {
                const double motor_rev =
                    ((double)raw - steer_user_zero_[id]) / (double)cfg_[id].feed;
                current_steer_deg_[id] = motor_rev * 360.0 / steer_ratio_;
            }

            const bool now_fault = has_fault(id);
            if (now_fault && !prev_fault) {
                RCLCPP_ERROR(this->get_logger(),
                    "Node %d FAULT (SW=0x%04X). 조향 F/R=%+.1f/%+.1f도",
                    id, (unsigned)motor_[id].status_word,
                    current_steer_deg_[ID_FRONT_STEER], current_steer_deg_[ID_REAR_STEER]);
                motion_inhibited_ = true;
                request_error_code(id);
                if (!is_steer_axis(id))
                    set_init_status("FAILED", "Drive motor fault, ID " + std::to_string(id));
            } else if (!now_fault && prev_fault) {
                RCLCPP_INFO(this->get_logger(), "Node %d fault cleared", id);
            }
            if (has_internal_limit(id) && !prev_limit)
                RCLCPP_WARN(this->get_logger(), "Node %d internal limit active", id);
            return;
        }

        // SDO 응답
        if (cob >= 0x581 && cob <= 0x584 && f.can_dlc >= 8) {
            const int id = static_cast<int>(cob - 0x580);
            uint16_t index = 0;
            std::memcpy(&index, &f.data[1], 2);
            const uint8_t sub = f.data[3];

            if (f.data[0] == 0x80) {
                uint32_t abort_code = 0;
                std::memcpy(&abort_code, &f.data[4], 4);
                motor_[id].error_read_pending = false;
                RCLCPP_ERROR(this->get_logger(),
                    "SDO abort: ID=%d index=0x%04X sub=%u code=0x%08X",
                    id, (unsigned)index, (unsigned)sub, (unsigned)abort_code);
                return;
            }
            // 0x603F 는 UINT16 이라 expedited 응답이 0x4B 이다.
            if (f.data[0] == 0x4B && index == OD_ERROR_CODE && sub == 0) {
                uint16_t ec = 0;
                std::memcpy(&ec, &f.data[4], 2);
                motor_[id].error_code = ec;
                motor_[id].error_code_valid = true;
                motor_[id].error_read_pending = false;
                if (ec == 0)
                    RCLCPP_INFO(this->get_logger(), "Node %d 오류코드 없음 (0x0000)", id);
                else
                    RCLCPP_ERROR(this->get_logger(), "Node %d 오류코드: 0x%04X", id, (unsigned)ec);
            }
        }
    }

    void read_can_messages() {
        if (can_sock_ < 0) return;
        struct can_frame f{};
        int guard = 0;
        while (read(can_sock_, &f, sizeof(f)) > 0 && ++guard < 300) handle_frame(f);
    }

    void request_error_code(int id) {
        if (motor_[id].error_read_pending) return;
        motor_[id].error_code_valid = false;
        motor_[id].error_read_pending = request_sdo_read(id, OD_ERROR_CODE, 0);
    }

    // ───────────────────────────── 상태 발행
    void set_init_status(const std::string &state, const std::string &detail) {
        init_state_ = state;
        init_detail_ = detail;
        if (pub_init_status_) {
            std_msgs::msg::String m;
            m.data = state + "|" + detail;
            pub_init_status_->publish(m);
        }
    }

    diagnostic_msgs::msg::DiagnosticStatus
    make_diag(int id, const std::string &name) const {
        diagnostic_msgs::msg::DiagnosticStatus d;
        d.name = name;
        d.hardware_id = std::string(CAN_INTERFACE) + ":id_" + std::to_string(id);

        if (!status_is_fresh(id)) {
            d.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
            d.message = "feedback_stale";
        } else if (has_fault(id)) {
            d.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            d.message = "fault";
        } else if (operation_enabled(id)) {
            d.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            d.message = "operational";
        } else {
            d.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            d.message = "not_operation_enabled";
        }

        char buf[24]{};
        std::snprintf(buf, sizeof(buf), "0x%04X", (unsigned)motor_[id].status_word);
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = "status_word"; kv.value = buf;
        d.values.push_back(kv);

        if (motor_[id].error_code_valid) {
            std::snprintf(buf, sizeof(buf), "0x%04X", (unsigned)motor_[id].error_code);
            kv.key = "error_code"; kv.value = buf;
            d.values.push_back(kv);
        }
        if (is_steer_axis(id)) {
            std::snprintf(buf, sizeof(buf), "%+.2f", current_steer_deg_[id]);
            kv.key = "steer_deg"; kv.value = buf;
            d.values.push_back(kv);
            std::snprintf(buf, sizeof(buf), "%+.2f", steer_cmd_deg_[id]);
            kv.key = "steer_cmd_deg"; kv.value = buf;
            d.values.push_back(kv);
        } else {
            std::snprintf(buf, sizeof(buf), "%+.3f", drive_vel_[id]);
            kv.key = "wheel_mps"; kv.value = buf;
            d.values.push_back(kv);
        }
        return d;
    }

    void publish_diagnostics() {
        if (pub_init_status_) {
            std_msgs::msg::String m;
            m.data = init_state_ + "|" + init_detail_;
            pub_init_status_->publish(m);
        }
        if (!pub_diag_) return;

        diagnostic_msgs::msg::DiagnosticArray arr;
        arr.header.stamp = this->now();
        arr.status.reserve(5);
        arr.status.push_back(make_diag(ID_FRONT_DRIVE, "lowcon/front_drive"));
        arr.status.push_back(make_diag(ID_FRONT_STEER, "lowcon/front_steer"));
        arr.status.push_back(make_diag(ID_REAR_DRIVE,  "lowcon/rear_drive"));
        arr.status.push_back(make_diag(ID_REAR_STEER,  "lowcon/rear_steer"));

        diagnostic_msgs::msg::DiagnosticStatus node;
        node.name = "lowcon/node";
        node.hardware_id = CAN_INTERFACE;
        if (recovery_ == Recovery::FAILED || init_state_ == "FAILED") {
            node.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        } else if (!zero_ready_ || motion_inhibited_ || !odom_valid_) {
            node.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        } else {
            node.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        }
        node.message = init_state_ + ": " + init_detail_;
        char buf[48]{};
        diagnostic_msgs::msg::KeyValue kv;
        std::snprintf(buf, sizeof(buf), "%d", static_cast<int>(mode_));
        kv.key = "mode"; kv.value = buf; node.values.push_back(kv);
        kv.key = "odom_valid"; kv.value = odom_valid_ ? "true" : "false";
        node.values.push_back(kv);
        kv.key = "motion_inhibited"; kv.value = motion_inhibited_ ? "true" : "false";
        node.values.push_back(kv);
        kv.key = "awaiting_neutral_cmd"; kv.value = awaiting_neutral_cmd_ ? "true" : "false";
        node.values.push_back(kv);
        std::snprintf(buf, sizeof(buf), "%+.3f %+.3f %+.3f", meas_vx_, meas_vy_, meas_wz_);
        kv.key = "twist_vx_vy_wz"; kv.value = buf; node.values.push_back(kv);
        arr.status.push_back(node);

        pub_diag_->publish(arr);
    }

    // ───────────────────────────── Fault 복구 상태기계
    //   motor_node 의 SteeringRecoveryState 를 속도 제어용으로 이식했다.
    //   PP 모드가 아니므로 set-point 토글과 0 위치 복귀 단계가 없고,
    //   대신 마지막에 영점을 다시 읽어 각도 기준을 재확보한다.
    void stop_all_axes() {
        for (int id : all_axes()) send_pdo_command(id, 0, get_control_word(id));
    }

    void fail_recovery(const std::string &reason) {
        recovery_ = Recovery::FAILED;
        recovery_requested_ = false;
        motion_inhibited_ = true;
        set_init_status("FAILED", "Recovery failed: " + reason);
        RCLCPP_ERROR(this->get_logger(),
            "복구 실패: %s. 구동을 계속 차단합니다.", reason.c_str());
    }

    const std::vector<int> &recovery_axes() const {
        return recovery_all_axes_ ? all_axes() : steer_axes();
    }

    void start_recovery(bool requested, bool all_axes_scope) {
        recovery_all_axes_ = all_axes_scope;
        if (!requested) auto_recovery_armed_ = false;   // 자동 복구는 1회성
        motion_inhibited_ = true;
        cmd_vx_ = cmd_vy_ = cmd_wz_ = 0.0;
        ack_delta_deg_ = 0.0;
        stop_all_axes();
        set_init_status("INITIALIZING",
            requested ? "Requested re-initialization" : "Automatic steering fault recovery");

        for (int id : recovery_axes()) {
            if (requested) {
                motor_[id].error_code_valid = false;
                motor_[id].error_read_pending = false;
            }
            if (!motor_[id].error_read_pending) request_error_code(id);
        }
        recovery_ = Recovery::WAIT_ERROR_CODE;
        recovery_deadline_ = std::chrono::steady_clock::now() +
                             std::chrono::milliseconds(RECOVERY_ERROR_READ_MS);
        RCLCPP_WARN(this->get_logger(), requested
            ? "재초기화 시작. 구동을 정지하고 축 상태를 복구합니다."
            : "조향 Fault 감지. 구동 정지 후 1회성 자동 복구를 시작합니다.");
    }

    // 복구가 제어권을 쥐고 있으면 true. control_loop 는 즉시 반환한다.
    bool process_recovery() {
        const auto now = std::chrono::steady_clock::now();

        if (recovery_ == Recovery::IDLE) {
            if (recovery_requested_) {
                recovery_requested_ = false;
                start_recovery(true, true);
                return true;
            }
            if (!steer_fault_active() && !drive_fault_active())
                return false;   // 정상. control_loop 가 계속 진행한다.

            // 구동축 Fault 는 자동 복구하지 않는다. 안전 정지 후 사람이 개입한다.
            if (drive_fault_active()) {
                motion_inhibited_ = true;
                stop_all_axes();
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                    "구동축 Fault. 자동 복구하지 않습니다. "
                    "~/initialize 서비스로 수동 재초기화하세요.");
                return true;
            }
            if (!auto_recovery_enabled_) {
                motion_inhibited_ = true;
                stop_all_axes();
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                    "조향축 Fault. steer_auto_recovery=false 이므로 대기합니다.");
                return true;
            }
            if (!auto_recovery_armed_) {
                fail_recovery("fault recurred after the one-shot automatic recovery");
                return true;
            }
            start_recovery(false, false);
            return true;
        }

        if (recovery_ == Recovery::FAILED) {
            stop_all_axes();
            return true;
        }

        stop_all_axes();

        switch (recovery_) {
        case Recovery::WAIT_ERROR_CODE: {
            bool all_read = true;
            for (int id : recovery_axes())
                if (!motor_[id].error_code_valid) all_read = false;
            if (!all_read && now < recovery_deadline_) return true;

            // Fault Reset(bit7) 상승 에지를 만들기 위해 먼저 LOW 로 내린다.
            for (int id : recovery_axes())
                send_sdo_write(id, OD_CONTROL_WORD, 0, 0x0000, 2);
            recovery_ = Recovery::WAIT_RESET_LOW;
            recovery_deadline_ = now + std::chrono::milliseconds(RECOVERY_STEP_MS);
            return true;
        }

        case Recovery::WAIT_RESET_LOW:
            if (now < recovery_deadline_) return true;
            for (int id : recovery_axes())
                send_sdo_write(id, OD_CONTROL_WORD, 0, 0x0080, 2);
            recovery_ = Recovery::WAIT_FAULT_CLEAR;
            recovery_deadline_ = now + std::chrono::milliseconds(RECOVERY_FAULT_CLEAR_TIMEOUT);
            RCLCPP_WARN(this->get_logger(), "Fault Reset 전송");
            return true;

        case Recovery::WAIT_FAULT_CLEAR: {
            bool cleared = true, fresh = true;
            for (int id : recovery_axes()) {
                if (has_fault(id)) cleared = false;
                if (!status_is_fresh(id)) fresh = false;
            }
            if (fresh && cleared) {
                // 재기동 전에 속도 지령을 0 으로 확정하고 모드/가감속을 재설정한다.
                for (int id : recovery_axes()) {
                    send_sdo_write(id, OD_TARGET_VEL, 0, 0, 4);
                    send_sdo_write(id, OD_MODES_OF_OP, 0, 3, 1);
                    send_sdo_write(id, OD_PROFILE_ACC, 0,
                                   is_steer_axis(id) ? steer_accel_ : drive_accel_, 4);
                    send_sdo_write(id, OD_PROFILE_DEC, 0,
                                   is_steer_axis(id) ? steer_decel_ : drive_decel_, 4);
                    send_sdo_write(id, OD_CONTROL_WORD, 0, 0x0006, 2);
                }
                recovery_ = Recovery::WAIT_SHUTDOWN;
                recovery_deadline_ = now + std::chrono::milliseconds(RECOVERY_STEP_MS);
                return true;
            }
            if (now >= recovery_deadline_) fail_recovery("fault bit did not clear");
            return true;
        }

        case Recovery::WAIT_SHUTDOWN:
            if (now < recovery_deadline_) return true;
            for (int id : recovery_axes())
                send_sdo_write(id, OD_CONTROL_WORD, 0, 0x0007, 2);
            recovery_ = Recovery::WAIT_SWITCH_ON;
            recovery_deadline_ = now + std::chrono::milliseconds(RECOVERY_STEP_MS);
            return true;

        case Recovery::WAIT_SWITCH_ON:
            if (now < recovery_deadline_) return true;
            for (int id : recovery_axes())
                send_sdo_write(id, OD_CONTROL_WORD, 0, 0x000F, 2);
            recovery_ = Recovery::WAIT_ENABLE;
            recovery_deadline_ = now + std::chrono::milliseconds(RECOVERY_ENABLE_TIMEOUT);
            return true;

        case Recovery::WAIT_ENABLE: {
            for (int id : recovery_axes()) {
                if (has_fault(id)) { fail_recovery("fault occurred again while enabling"); return true; }
            }
            bool ready = true;
            for (int id : recovery_axes())
                if (!status_is_fresh(id) || !operation_enabled(id)) ready = false;
            if (ready) {
                recovery_ = Recovery::REFRESH_ZERO;
                return true;
            }
            if (now >= recovery_deadline_) fail_recovery("axis did not reach Operation Enabled");
            return true;
        }

        case Recovery::REFRESH_ZERO: {
            // 절대 엔코더 기준을 다시 읽어 각도 기준을 재확보한다. 모터는 움직이지 않는다.
            if (!compute_steer_user_zero()) {
                fail_recovery("failed to re-acquire the steering zero reference");
                return true;
            }
            for (int i = 0; i < 5; ++i) { in_deadband_[i] = false; stall_cnt_[i] = 0; }
            ack_delta_deg_ = 0.0;
            await_settle_ = true;
            cmd_vx_ = cmd_vy_ = cmd_wz_ = 0.0;
            last_cmd_time_ = this->now();
            // 상위(Nav2 등)가 같은 지령을 계속 보내 곧바로 재-Fault 나는 것을 막는다.
            awaiting_neutral_cmd_ = true;
            motion_inhibited_ = true;
            if (recovery_all_axes_) auto_recovery_armed_ = true;
            recovery_all_axes_ = false;
            recovery_ = Recovery::IDLE;
            set_init_status("READY",
                "All axes operational; steering zero re-acquired. Waiting for neutral cmd_vel");
            RCLCPP_INFO(this->get_logger(),
                "복구 완료. 중립 cmd_vel 을 한 번 받은 뒤 구동을 재개합니다.");
            return true;
        }

        case Recovery::IDLE:
        case Recovery::FAILED:
        default:
            return true;
        }
    }

    // ───────────────────────────── 콜백
    void mode_cb(const std_msgs::msg::Int32::SharedPtr msg) {
        int m = msg->data;
        if (m < 0 || m > 2) {
            RCLCPP_WARN(this->get_logger(), "알 수 없는 mode %d 무시", m);
            return;
        }
        Mode nm = static_cast<Mode>(m);
        mode_cmd_received_ = true;
        cmd_vx_ = cmd_vy_ = cmd_wz_ = 0.0;
        if (nm != mode_) await_settle_ = true;    // 전환 시에만 조향 정착 대기
        if (nm == Mode::ALIGN) ack_delta_deg_ = 0.0;
        mode_ = nm;

        if (nm == Mode::ALIGN)          RCLCPP_INFO(this->get_logger(), "mode 0: 재정렬");
        else if (nm == Mode::ACKERMANN) RCLCPP_INFO(this->get_logger(), "mode 1: 애커만 (후륜 역방향)");
        else                            RCLCPP_INFO(this->get_logger(),
                                            "mode 2: 디프 (전후륜 동방향 %+.0f도)", diff_steer_deg_);
    }

    void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_time_ = this->now();

        if (awaiting_neutral_cmd_) {
            const bool neutral = std::fabs(msg->linear.x) < 0.001 &&
                                 std::fabs(msg->linear.y) < 0.001 &&
                                 std::fabs(msg->angular.z) < 0.001;
            cmd_vx_ = cmd_vy_ = cmd_wz_ = 0.0;
            if (neutral) {
                awaiting_neutral_cmd_ = false;
                motion_inhibited_ = false;
                RCLCPP_INFO(this->get_logger(), "중립 cmd_vel 수신. 구동을 다시 허용합니다.");
            }
            return;
        }

        cmd_vx_ = msg->linear.x;
        cmd_vy_ = msg->linear.y;
        cmd_wz_ = msg->angular.z;
    }

    void initialize_cb(const std_srvs::srv::Trigger::Request::SharedPtr,
                       std_srvs::srv::Trigger::Response::SharedPtr res) {
        if (can_sock_ < 0) {
            res->success = false; res->message = "CAN socket is unavailable"; return;
        }
        if (recovery_ != Recovery::IDLE && recovery_ != Recovery::FAILED) {
            res->success = false; res->message = "Initialization is already running"; return;
        }
        recovery_ = Recovery::IDLE;
        recovery_requested_ = true;
        motion_inhibited_ = true;
        mode_ = Mode::ALIGN;
        set_init_status("INITIALIZING", "Re-initialization requested");
        res->success = true;
        res->message = "Re-initialization accepted";
    }

    void reset_odom_cb(const std_srvs::srv::Trigger::Request::SharedPtr,
                       std_srvs::srv::Trigger::Response::SharedPtr res) {
        x_ = y_ = th_ = 0.0;
        res->success = true;
        res->message = "Odometry reset to origin";
        RCLCPP_INFO(this->get_logger(), "오도메트리 원점 리셋");
    }

    bool steer_settled() const {
        for (int id : steer_axes())
            if (std::fabs(current_steer_deg_[id] - steer_cmd_deg_[id]) > steer_settle_deg_)
                return false;
        return true;
    }

    // 조향각 오차 -> 모터 속도 지령(pulse/s)
    int32_t steer_velocity_cmd(int id) {
        if (!zero_ready_ || !status_is_fresh(id) || has_fault(id)) return 0;
        double err = steer_cmd_deg_[id] - current_steer_deg_[id];
        double ae  = std::fabs(err);

        // 데드밴드 히스테리시스: 한 번 들어가면 더 큰 오차가 생겨야 다시 나온다.
        // 경계에서 들락날락하며 생기는 리밋사이클(움찔거림)을 막는다.
        if (in_deadband_[id]) {
            if (ae < steer_deadband_deg_ * deadband_exit_ratio_) return 0;
            in_deadband_[id] = false;
        } else if (ae < steer_deadband_deg_) {
            in_deadband_[id] = true;
            return 0;
        }

        double cur = current_steer_deg_[id];
        if ((err > 0 && cur >= max_steer_abs_deg_) ||
            (err < 0 && cur <= -max_steer_abs_deg_)) return 0;

        double rate_dps = std::clamp(steer_gain_ * err,
                                     -max_steer_rate_dps_, max_steer_rate_dps_);

        // 최소 속도 하한: 감속기 마찰 구간에서 기어가는 것을 막는다
        if (std::fabs(rate_dps) < steer_min_rate_dps_)
            rate_dps = std::copysign(steer_min_rate_dps_, rate_dps);

        double pulse_s = rate_dps * steer_ratio_ / 360.0 * cfg_[id].feed;
        return (int32_t)std::lround(pulse_s);
    }

    void monitor_steer_stall(int32_t vel_fs, int32_t vel_rs) {
        for (int id : steer_axes()) {
            int32_t v = (id == ID_FRONT_STEER) ? vel_fs : vel_rs;
            if (has_fault(id)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Node %d FAULT (SW=0x%04X). 하드스톱 충돌 또는 과부하 의심",
                    id, motor_[id].status_word);
                stall_cnt_[id] = 0;
            } else if (v != 0) {
                if (std::fabs(current_steer_deg_[id] - last_steer_deg_[id]) < 0.02)
                    stall_cnt_[id]++;
                else
                    stall_cnt_[id] = 0;
                if (stall_cnt_[id] > 50) {   // 약 1초간 정체
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Node %d 조향 정체: 지령 %+.2f도, 실제 %+.2f도, 속도지령 %d pulse/s. "
                        "하드스톱/토크 부족 의심 (steer_ratio 문제가 아님)",
                        id, steer_cmd_deg_[id], current_steer_deg_[id], v);
                    stall_cnt_[id] = 0;
                }
            } else {
                stall_cnt_[id] = 0;
            }
            last_steer_deg_[id] = current_steer_deg_[id];
        }
    }

    // ───────────────────────────── 메인 루프
    void control_loop() {
        read_can_messages();
        update_odometry();          // 지령과 무관하게 항상 측정 기반으로 갱신
        publish_steer_angle();

        if (can_sock_ < 0) return;

        // Fault 복구가 제어권을 가지고 있으면 여기서 끝낸다.
        if (process_recovery()) return;

        const bool cmd_timeout =
            (this->now() - last_cmd_time_).seconds() > cmd_vel_timeout_s_;
        if (cmd_timeout) { cmd_vx_ = cmd_vy_ = cmd_wz_ = 0.0; }

        const double vx = std::clamp(cmd_vx_, -max_linear_vel_, max_linear_vel_);
        const double vy = std::clamp(cmd_vy_, -max_linear_vel_, max_linear_vel_);
        const double wz = std::clamp(cmd_wz_, -max_angular_vel_, max_angular_vel_);

        // 각 축이 굴러야 할 접지속도 [m/s]
        double wheel_front = 0.0, wheel_rear = 0.0;

        if (mode_ == Mode::ALIGN || !zero_ready_) {
            ack_delta_deg_ = 0.0;
            steer_cmd_deg_[ID_FRONT_STEER] = 0.0;
            steer_cmd_deg_[ID_REAR_STEER]  = 0.0;
        }
        else if (mode_ == Mode::ACKERMANN) {
            // 전후륜 역방향 조향. 대칭 4WS 의 정확한 역기구학은
            //   tan(delta) = wz * L / (2 * vx),  v_wheel = vx / cos(delta)
            // 이다. (asin 근사가 아니라 atan 이 정확하다)
            if (!(cmd_timeout && hold_steer_on_timeout_)) {
                if (std::fabs(vx) > 0.01) {
                    ack_delta_deg_ = std::atan(wz * wheelbase_ / (2.0 * vx)) * RAD2DEG;
                } else if (std::fabs(wz) > 0.01) {
                    ack_delta_deg_ = wz * ack_stationary_gain_;   // 정지 중에는 조향만
                }
                // vx, wz 모두 0이면 마지막 조향각 유지
            }
            ack_delta_deg_ = std::clamp(ack_delta_deg_,
                                        -max_steer_right_deg_, max_steer_left_deg_);

            steer_cmd_deg_[ID_FRONT_STEER] =  ack_delta_deg_;
            steer_cmd_deg_[ID_REAR_STEER]  = -ack_delta_deg_;   // 후륜 역방향

            if (std::fabs(vx) > 0.01) {
                // 몸체 전진속도 vx 를 내려면 휠은 vx/cos(delta) 로 굴러야 한다.
                const double cd = std::max(0.2, std::cos(ack_delta_deg_ * DEG2RAD));
                const double v_w = std::clamp(vx / cd, -max_linear_vel_, max_linear_vel_);
                wheel_front = v_w;
                wheel_rear  = v_w;
            }
        }
        else {  // DIFF : 전후륜이 같은 방향으로 정렬 (motor_node 의 도킹 자세와 동일)
            // 전후륜 각도를 따로 줄 수 있다. motor_node 의 docking_front_pulse /
            // docking_rear_pulse 가 서로 다른 캘리브레이션 값이기 때문이다.
            const double df_deg = (diff_steer_front_deg_ != 0.0)
                                      ? diff_steer_front_deg_ : diff_steer_deg_;
            const double dr_deg = (diff_steer_rear_deg_ != 0.0)
                                      ? diff_steer_rear_deg_ : diff_steer_deg_;
            steer_cmd_deg_[ID_FRONT_STEER] = df_deg;
            steer_cmd_deg_[ID_REAR_STEER]  = dr_deg;

            // df != dr 이어도 지령을 정확히 만족시키는 역기구학
            //   v_f sin(df) = v_lat + wz*L/2
            //   v_r sin(dr) = v_lat - wz*L/2
            // 이렇게 주면 odom 이 되돌리는 vy, wz 가 지령과 정확히 일치한다.
            const double sdf = std::sin(df_deg * DEG2RAD);
            const double sdr = std::sin(dr_deg * DEG2RAD);

            // 디프(도킹) 전용 속도 제한. 0 이면 전역 제한을 쓴다.
            const double lin_lim = (diff_max_linear_vel_ > 0.0)
                                       ? std::min(diff_max_linear_vel_, max_linear_vel_)
                                       : max_linear_vel_;
            const double ang_lim = (diff_max_angular_vel_ > 0.0)
                                       ? std::min(diff_max_angular_vel_, max_angular_vel_)
                                       : max_angular_vel_;
            const double v_lat = std::clamp((diff_lateral_ ? vx : 0.0) + vy,
                                            -lin_lim, lin_lim);
            const double half  = std::clamp(wz, -ang_lim, ang_lim) * wheelbase_ * 0.5;

            if (std::fabs(sdf) > 0.1 && std::fabs(sdr) > 0.1) {
                wheel_front = std::clamp((v_lat + half) / sdf,
                                         -max_linear_vel_, max_linear_vel_);
                wheel_rear  = std::clamp((v_lat - half) / sdr,
                                         -max_linear_vel_, max_linear_vel_);
            } else {
                // 조향각이 0도 부근이면 횡이동이 불가능하다. 안전하게 정지한다.
                wheel_front = wheel_rear = 0.0;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "디프 모드 조향각(%.1f/%.1f도)이 0 부근이라 횡이동을 지령할 수 없습니다.",
                    df_deg, dr_deg);
            }
        }

        // ── 조향 (속도 제어)
        //    steer_hold_until_mode_cmd=true 이면 첫 /mode 를 받기 전까지 조향을 움직이지 않는다.
        //    (기동 직후 자동으로 0도로 재정렬되는 것을 원치 않을 때 쓴다)
        const bool steer_hold = steer_hold_until_mode_cmd_ && !mode_cmd_received_;
        if (steer_hold) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "조향 대기: /mode 를 한 번 받을 때까지 조향을 움직이지 않습니다.");
        }
        const int32_t vel_fs = steer_hold ? 0 : steer_velocity_cmd(ID_FRONT_STEER);
        const int32_t vel_rs = steer_hold ? 0 : steer_velocity_cmd(ID_REAR_STEER);
        monitor_steer_stall(vel_fs, vel_rs);

        // ── 구동 허용 조건
        //    정렬 전이거나, 모드 전환 직후 조향이 아직 안 잡혔거나,
        //    축 상태가 stale/Fault/Not-Enabled 이면 구동하지 않는다.
        if (await_settle_ && steer_settled()) await_settle_ = false;

        const bool motors_ready = !require_motor_ready_ || all_motors_operational();
        const bool drive_ok = zero_ready_ && motors_ready && !motion_inhibited_ &&
                              !awaiting_neutral_cmd_ && mode_ != Mode::ALIGN && !await_settle_;

        if (!drive_ok && (std::fabs(wheel_front) > 1e-6 || std::fabs(wheel_rear) > 1e-6)) {
            const char *why = !zero_ready_        ? "조향 영점 미확보"
                            : !motors_ready       ? "축 피드백 stale 또는 Not-Enabled"
                            : motion_inhibited_   ? "motion inhibited"
                            : awaiting_neutral_cmd_ ? "중립 cmd_vel 대기"
                            : await_settle_       ? "조향 정착 대기"
                                                  : "정렬 모드";
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "구동 차단: %s", why);
        }

        int32_t vel_fd = 0, vel_rd = 0;
        if (drive_ok) {
            vel_fd = wheel_mps_to_pulse(ID_FRONT_DRIVE, wheel_front);
            vel_rd = wheel_mps_to_pulse(ID_REAR_DRIVE,  wheel_rear);
        }

        send_pdo_command(ID_FRONT_DRIVE, vel_fd, get_control_word(ID_FRONT_DRIVE));
        send_pdo_command(ID_REAR_DRIVE,  vel_rd, get_control_word(ID_REAR_DRIVE));
        send_pdo_command(ID_FRONT_STEER, vel_fs, get_control_word(ID_FRONT_STEER));
        send_pdo_command(ID_REAR_STEER,  vel_rs, get_control_word(ID_REAR_STEER));
    }

    int32_t wheel_mps_to_pulse(int id, double mps) const {
        const double rpm = (mps / wheel_circum_) * 60.0 * drive_ratio_;
        const auto it = cfg_.find(id);
        const int32_t feed = (it != cfg_.end()) ? it->second.feed : DEFAULT_FEED;
        return (int32_t)std::lround(rpm / 60.0 * feed);
    }

    void publish_steer_angle() {
        std_msgs::msg::Float32MultiArray m;
        m.data = { (float)current_steer_deg_[ID_FRONT_STEER],
                   (float)current_steer_deg_[ID_REAR_STEER],
                   (float)steer_cmd_deg_[ID_FRONT_STEER],
                   (float)steer_cmd_deg_[ID_REAR_STEER] };
        pub_steer_->publish(m);
    }

    // ───────────────────────────── 오도메트리 (모드 무관 단일 모델)
    //
    //   전/후 축을 각각 조향 가능한 2축 플랫폼의 정기구학을 최소자승으로 푼다.
    //   모드별 분기가 전혀 없으므로 애커만 <-> 디프 전환 도중에도 연속적이다.
    //
    //     vx = ( v_f cos(df) + v_r cos(dr) ) / 2
    //     vy = ( v_f sin(df) + v_r sin(dr) ) / 2
    //     wz = ( v_f sin(df) - v_r sin(dr) ) / L
    //
    void update_odometry() {
        const rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        odom_valid_ = odom_inputs_valid();

        if (odom_valid_) {
            const double df = current_steer_deg_[ID_FRONT_STEER] * DEG2RAD;
            const double dr = current_steer_deg_[ID_REAR_STEER]  * DEG2RAD;

            double vf = drive_vel_[ID_FRONT_DRIVE];
            double vr = drive_vel_[ID_REAR_DRIVE];
            if (std::fabs(vf) < odom_v_deadband_) vf = 0.0;
            if (std::fabs(vr) < odom_v_deadband_) vr = 0.0;

            meas_vx_ = 0.5 * (vf * std::cos(df) + vr * std::cos(dr));
            meas_vy_ = 0.5 * (vf * std::sin(df) + vr * std::sin(dr));
            meas_wz_ = (vf * std::sin(df) - vr * std::sin(dr)) / wheelbase_;
        } else {
            meas_vx_ = meas_vy_ = meas_wz_ = 0.0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "오도메트리 입력 없음 (영점 %s, 조향 피드백 %s, 주행 피드백 %s). "
                "포즈를 정지시킵니다.",
                zero_ready_ ? "OK" : "NG",
                steer_feedback_fresh() ? "OK" : "STALE",
                drive_feedback_fresh() ? "OK" : "STALE");
        }

        // 중점(midpoint) 적분: 선회 중 누적 오차를 크게 줄인다.
        if (odom_valid_ && dt > 0.0 && dt <= 0.5) {
            const double th_mid = th_ + meas_wz_ * dt * 0.5;
            x_  += (meas_vx_ * std::cos(th_mid) - meas_vy_ * std::sin(th_mid)) * dt;
            y_  += (meas_vx_ * std::sin(th_mid) + meas_vy_ * std::cos(th_mid)) * dt;
            th_  = normalize_angle(th_ + meas_wz_ * dt);
        }

        tf2::Quaternion q; q.setRPY(0, 0, th_);
        geometry_msgs::msg::Quaternion qm;
        qm.x = q.x(); qm.y = q.y(); qm.z = q.z(); qm.w = q.w();

        if (tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = now;
            tf.header.frame_id = odom_frame_;
            tf.child_frame_id  = base_frame_;
            tf.transform.translation.x = x_;
            tf.transform.translation.y = y_;
            tf.transform.translation.z = 0.0;
            tf.transform.rotation = qm;
            tf_broadcaster_->sendTransform(tf);
        }

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id  = base_frame_;
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = qm;

        // 몸체(base_link) 기준 트위스트. 횡속도 vy 도 그대로 싣는다.
        odom.twist.twist.linear.x  = meas_vx_;
        odom.twist.twist.linear.y  = meas_vy_;
        odom.twist.twist.angular.z = meas_wz_;

        // 공분산. 입력이 무효하면 크게 키워 상위 융합기가 무시하도록 한다.
        const double pxy = odom_valid_ ? 1e-3 : 1e6;
        const double pth = odom_valid_ ? 5e-3 : 1e6;
        const double txy = odom_valid_ ? 1e-3 : 1e6;
        const double tth = odom_valid_ ? 5e-3 : 1e6;
        for (auto &c : odom.pose.covariance)  c = 0.0;
        for (auto &c : odom.twist.covariance) c = 0.0;
        odom.pose.covariance[0]   = pxy;   // x
        odom.pose.covariance[7]   = pxy;   // y
        odom.pose.covariance[14]  = 1e6;   // z
        odom.pose.covariance[21]  = 1e6;   // roll
        odom.pose.covariance[28]  = 1e6;   // pitch
        odom.pose.covariance[35]  = pth;   // yaw
        odom.twist.covariance[0]  = txy;
        odom.twist.covariance[7]  = txy;
        odom.twist.covariance[14] = 1e6;
        odom.twist.covariance[21] = 1e6;
        odom.twist.covariance[28] = 1e6;
        odom.twist.covariance[35] = tth;

        pub_odom_->publish(odom);
    }
};

// 기본 파라미터 파일 경로
//   1) 환경변수 LOWCON_PARAMS_FILE 이 있으면 그 경로
//   2) 없으면 <install>/share/motor_node/config/lowcon.yaml
static std::string default_params_file() {
    if (const char *env = std::getenv("LOWCON_PARAMS_FILE"))
        return std::string(env);
    try {
        return ament_index_cpp::get_package_share_directory("motor_node") +
               "/config/lowcon.yaml";
    } catch (const std::exception &) {
        return std::string();
    }
}

int main(int argc, char **argv) {
    // config/lowcon.yaml 을 자동으로 먼저 읽는다.
    // 사용자가 준 인자는 뒤에 붙으므로 언제나 yaml 보다 우선한다.
    //   ros2 run motor_node lowcon
    //   ros2 run motor_node lowcon --ros-args -p steer_ratio:=110.0   (yaml 을 덮어씀)
    std::string pf = default_params_file();
    bool injected = !pf.empty() && access(pf.c_str(), R_OK) == 0;

    std::vector<std::string> args;
    args.emplace_back(argv[0]);
    if (injected) {
        args.emplace_back("--ros-args");
        args.emplace_back("--params-file");
        args.emplace_back(pf);
        args.emplace_back("--");
    }
    for (int i = 1; i < argc; ++i) args.emplace_back(argv[i]);

    std::vector<const char *> cargv;
    cargv.reserve(args.size());
    for (const auto &a : args) cargv.push_back(a.c_str());
    int cargc = static_cast<int>(cargv.size());

    rclcpp::init(cargc, cargv.data());

    if (injected)
        RCLCPP_INFO(rclcpp::get_logger("lowcon"),
                    "기본 파라미터 파일 적용: %s", pf.c_str());
    else
        RCLCPP_WARN(rclcpp::get_logger("lowcon"),
                    "기본 파라미터 파일을 찾지 못했습니다 (%s). "
                    "--params-file 로 직접 지정하세요.",
                    pf.empty() ? "share 경로 확인 실패" : pf.c_str());

    rclcpp::spin(std::make_shared<MotorNode>());
    rclcpp::shutdown();
    return 0;
}
