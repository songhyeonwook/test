// 4WS 하부 모터 노드 (rbio TransferRobot motor_node 구조 + lowcon 모드 체계)
//
//  - 주행 ID 1/3 : Profile Velocity(mode 3), 0x60FF 로 속도 명령
//  - 조향 ID 2/4 : Profile Position(mode 1), 0x607A 로 목표 각도 명령
//  - 조향은 New set-point(bit4) / ACK(bit12) 핸드셰이크로 확실히 갱신한다
//  - 시동 시 Fault reset 후 조향축을 0 도로 센터링하고 mode 0(정렬)에서 대기한다
//
// 토픽
//   /mode                          std_msgs/Int32  0=재정렬, 1=애커만(4WS 역위상), 2=디프
//   /cmd_vel                       geometry_msgs/Twist
//   /odom                          nav_msgs/Odometry           (발행)
//   /steer_angle_deg               std_msgs/Float32MultiArray  (발행)
//                                    [전륜실제, 후륜실제, 전륜지령, 후륜지령]
//   motor_node/command_ack         std_msgs/String
//   motor_node/diagnostics         diagnostic_msgs/DiagnosticArray (latched)
//   motor_node/initialization_status  std_msgs/String  latched, "STATE|detail"
//   motor_node/drive_mode          std_msgs/String  latched, "MODE|detail"
// 서비스
//   motor_node/initialize          std_srvs/Trigger  4축 재초기화 + 조향 센터링
//   motor_node/reset_odom          std_srvs/Trigger  오도메트리 원점 리셋
//
// 모드 전환: 주행이 완전히 멈춘 뒤 조향을 목표 각도(정렬/애커만=0도, 디프=diff_steer_deg)
//   로 돌리고, 도달하면 모드가 활성화된다. 디프 모드는 cmd_vel 의 linear.x(또는 y) 를
//   횡이동, angular.z 를 제자리 회전으로 해석한다.
//
// ── 오도메트리 (모드 무관 단일 모델) ─────────────────────────────
//   전/후 축을 각각 조향 가능한 2축 플랫폼으로 보고, 측정된 조향각과 측정된
//   휠 속도로부터 몸체 속도를 직접 역산한다. 지령/모드는 전혀 쓰지 않는다.
//
//     접지점  앞 p_f = (+L/2, 0),  뒤 p_r = (-L/2, 0)
//     강체    v_wheel = v_body + omega x p
//       vx           = v_f cos(df)      vx           = v_r cos(dr)
//       vy + w*L/2   = v_f sin(df)      vy - w*L/2   = v_r sin(dr)
//     최소자승해
//       vx = ( v_f cos(df) + v_r cos(dr) ) / 2
//       vy = ( v_f sin(df) + v_r sin(dr) ) / 2
//       w  = ( v_f sin(df) - v_r sin(dr) ) / L
//

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <memory>
#include <net/if.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

#define CAN_INTERFACE "can0"

// 모터 ID
#define ID_FRONT_DRIVE 1
#define ID_FRONT_STEER 2
#define ID_REAR_DRIVE 3
#define ID_REAR_STEER 4

// 로봇 물리 정보
#define WHEEL_CIRCUM 0.47124 // 바퀴 둘레 (0.15 * pi)
#define WHEELBASE 1.29       // 축간거리 1290mm
#define MAX_STEER_DEG 55.0   // 후륜 조향 Fault가 시작되는 약 63도보다 여유 확보

// 기어비
#define DRIVE_RATIO 1.0  // 주행 1:1
#define STEER_RATIO 10.0 // 조향 10:1 (중요!)

// 엔코더 분해능 [pulse/rev]
#define ENCODER_PPR 131072.0

// CANopen 정의
#define NMT_START 0x01
#define OD_CONTROL_WORD 0x6040
#define OD_ERROR_CODE 0x603F
#define OD_MODES_OF_OP 0x6060
#define OD_TARGET_POS 0x607A
#define OD_TARGET_VEL 0x60FF

namespace {

constexpr uint16_t STATUS_STATE_MASK = 0x006F;
constexpr uint16_t STATUS_OPERATION_ENABLED = 0x0027;
constexpr uint16_t STATUS_SETPOINT_ACKNOWLEDGED = 1U << 12;
constexpr auto MOTOR_FEEDBACK_TIMEOUT = std::chrono::milliseconds(300);
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

bool operation_enabled(uint16_t status_word)
{
    return (status_word & STATUS_STATE_MASK) == STATUS_OPERATION_ENABLED;
}

double normalize_angle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

} // namespace

class MotorNode : public rclcpp::Node
{
  public:
    MotorNode() : Node("motor_node")
    {
        sub_mode_ = this->create_subscription<std_msgs::msg::Int32>(
            "mode", 10, std::bind(&MotorNode::mode_cb, this, std::placeholders::_1));
        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&MotorNode::cmd_vel_cb, this, std::placeholders::_1));
        pub_command_ack_ =
            this->create_publisher<std_msgs::msg::String>("motor_node/command_ack", 10);
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        pub_steer_ =
            this->create_publisher<std_msgs::msg::Float32MultiArray>("steer_angle_deg", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1));
        latched_qos.reliable();
        latched_qos.transient_local();
        pub_diagnostics_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "motor_node/diagnostics", latched_qos);
        pub_initialization_status_ = this->create_publisher<std_msgs::msg::String>(
            "motor_node/initialization_status", latched_qos);
        pub_drive_mode_status_ =
            this->create_publisher<std_msgs::msg::String>("motor_node/drive_mode", latched_qos);

        srv_initialize_ = this->create_service<std_srvs::srv::Trigger>(
            "motor_node/initialize", std::bind(&MotorNode::initialize_cb, this,
                                               std::placeholders::_1, std::placeholders::_2));
        srv_reset_odom_ = this->create_service<std_srvs::srv::Trigger>(
            "motor_node/reset_odom", std::bind(&MotorNode::reset_odom_cb, this,
                                               std::placeholders::_1, std::placeholders::_2));

        last_cmd_time_ = this->now();
        last_odom_time_ = this->now();

        // --- 조향 프로파일 (rbio 와 동일 기본값) ---
        steering_profile_velocity_ = std::clamp(
            static_cast<int>(this->declare_parameter<int>("steering_profile_velocity", 40000)),
            1000, 200000);
        steering_profile_acceleration_ = std::clamp(
            static_cast<int>(this->declare_parameter<int>("steering_profile_acceleration", 40000)),
            1000, 200000);
        steering_profile_deceleration_ = std::clamp(
            static_cast<int>(this->declare_parameter<int>("steering_profile_deceleration", 40000)),
            1000, 200000);
        max_steering_angle_rad_ =
            std::clamp(this->declare_parameter<double>("max_steering_angle_deg", MAX_STEER_DEG),
                       30.0, 85.0) *
            DEG2RAD;

        // --- 속도 상한 (lowcon 파라미터명) ---
        max_linear_vel_ =
            std::clamp(this->declare_parameter<double>("max_linear_vel", 0.2), 0.01, 1.0);
        max_angular_vel_ =
            std::clamp(this->declare_parameter<double>("max_angular_vel", 0.3), 0.01, 2.0);
        cmd_vel_timeout_s_ =
            std::max(0.05, this->declare_parameter<double>("cmd_vel_timeout_s", 0.5));

        // --- 디프 모드 (lowcon 파라미터명) ---
        //   전후륜 각도를 따로 줄 수 있다 (0 이면 diff_steer_deg 공통).
        //   rbio 의 docking_front_pulse/rear_pulse 가 서로 달랐던 것과 같은 이유다.
        diff_steer_deg_ =
            std::clamp(this->declare_parameter<double>("diff_steer_deg", 90.0), 45.0, 95.0);
        diff_steer_front_deg_ = this->declare_parameter<double>("diff_steer_front_deg", 0.0);
        diff_steer_rear_deg_ = this->declare_parameter<double>("diff_steer_rear_deg", 0.0);
        diff_max_linear_vel_ =
            std::max(0.0, this->declare_parameter<double>("diff_max_linear_vel", 0.0));
        diff_max_angular_vel_ =
            std::max(0.0, this->declare_parameter<double>("diff_max_angular_vel", 0.0));
        diff_lateral_ = this->declare_parameter<bool>("diff_linear_x_is_lateral", true);

        // --- 오도메트리 (lowcon 파라미터명) ---
        odom_frame_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
        base_frame_ = this->declare_parameter<std::string>("base_frame_id", "base_link");
        // hw 에서는 fast_lio_localization/tf_2d.py 가 odom->base_link 를 내므로 기본 false
        publish_odom_tf_ = this->declare_parameter<bool>("publish_odom_tf", false);
        odom_v_deadband_ = std::clamp(
            this->declare_parameter<double>("odom_wheel_deadband_mps", 0.005), 0.0, 0.05);

        if (init_can_socket() < 0) {
            RCLCPP_ERROR(this->get_logger(), "CAN Socket Init Failed!");
            set_initialization_status("FAILED", "CAN socket initialization failed", false);
            failed_ = true;
            publish_mode_status("CAN socket initialization failed");
            diagnostics_timer_ =
                this->create_wall_timer(250ms, std::bind(&MotorNode::publish_diagnostics, this));
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Configuring four lower motors...");

        init_motor_sdo(ID_FRONT_DRIVE, 3);
        init_motor_sdo(ID_REAR_DRIVE, 3);
        init_motor_sdo(ID_FRONT_STEER, 1);
        init_motor_sdo(ID_REAR_STEER, 1);

        setup_rpdo_mapping(ID_FRONT_DRIVE, 3);
        setup_rpdo_mapping(ID_REAR_DRIVE, 3);
        setup_rpdo_mapping(ID_FRONT_STEER, 1);
        setup_rpdo_mapping(ID_REAR_STEER, 1);

        setup_tpdo_mapping(ID_FRONT_DRIVE, 3);
        setup_tpdo_mapping(ID_REAR_DRIVE, 3);
        setup_tpdo_mapping(ID_FRONT_STEER, 1);
        setup_tpdo_mapping(ID_REAR_STEER, 1);

        for (const int id : {ID_FRONT_DRIVE, ID_FRONT_STEER, ID_REAR_DRIVE, ID_REAR_STEER}) {
            send_nmt_command(NMT_START, id);
        }

        // 초기화 시 원점
        last_pulse_front_ = -999999;
        last_pulse_rear_ = -999999;

        motion_inhibited_ = true;
        steering_recovery_requested_ = true;
        set_initialization_status("INITIALIZING", "Startup lower motor initialization queued",
                                  false);
        publish_mode_status("Startup centering queued");

        timer_ = this->create_wall_timer(20ms, std::bind(&MotorNode::control_loop, this));
        diagnostics_timer_ =
            this->create_wall_timer(250ms, std::bind(&MotorNode::publish_diagnostics, this));
        RCLCPP_INFO(this->get_logger(),
                    "4WS motor driver configured; startup centering queued "
                    "(max_v=%.2f m/s, max_w=%.2f rad/s, max_steer=%.1f deg, diff=%.1f deg, "
                    "odom_tf=%s)",
                    max_linear_vel_, max_angular_vel_, max_steering_angle_rad_ * RAD2DEG,
                    diff_steer_deg_, publish_odom_tf_ ? "on" : "off");
    }

    ~MotorNode()
    {
        if (can_sock_ < 0)
            return;
        send_drive_pdo(ID_FRONT_DRIVE, 0, 0x000F);
        send_drive_pdo(ID_REAR_DRIVE, 0, 0x000F);
        usleep(20000);
        send_nmt_command(0x81, 0);
        close(can_sock_);
    }

  private:
    struct MotorStatus
    {
        uint16_t statusword = 0;
        uint16_t error_code = 0;
        bool tpdo_received = false;
        bool error_code_valid = false;
        bool error_read_pending = false;
        std::chrono::steady_clock::time_point last_tpdo{};
    };

    enum class SteeringRecoveryState {
        IDLE,
        WAIT_ERROR_CODE,
        WAIT_RESET_LOW,
        WAIT_FAULT_CLEAR,
        WAIT_SHUTDOWN,
        WAIT_SWITCH_ON,
        WAIT_ENABLE,
        WAIT_SETPOINT_LOW,
        WAIT_ZERO,
        FAILED
    };

    // /mode 값과 1:1
    enum class Mode : int { ALIGN = 0, ACKERMANN = 1, DIFF = 2 };

    int can_sock_ = -1;
    double cmd_vx_ = 0.0, cmd_vy_ = 0.0, cmd_wz_ = 0.0;

    // 피드백
    double current_drive_vel_front_ = 0.0;   // [m/s] 바퀴 접선 속도 (부호 포함)
    double current_drive_vel_rear_ = 0.0;
    double current_steer_angle_front_ = 0.0; // [rad] 앞바퀴 조향 (+ = 좌)
    double current_steer_angle_rear_ = 0.0;  // [rad] 뒷바퀴 조향
    int32_t current_steer_pulse_front_ = 0;
    int32_t current_steer_pulse_rear_ = 0;
    double steer_cmd_deg_front_ = 0.0; // 진단/steer_angle_deg 용 지령각
    double steer_cmd_deg_rear_ = 0.0;

    // 오도메트리 적분값
    double x_ = 0.0, y_ = 0.0, th_ = 0.0;
    double meas_vx_ = 0.0, meas_vy_ = 0.0, meas_wz_ = 0.0;
    bool odom_valid_ = false;

    // 조향 셋포인트 핸드셰이크
    int32_t last_pulse_front_ = 0;
    int32_t last_pulse_rear_ = 0;
    int toggle_front_ = 0;
    int toggle_rear_ = 0;
    int setpoint_retry_front_ = 0;
    int setpoint_retry_rear_ = 0;
    int32_t setpoint_feedback_front_ = 0;
    int32_t setpoint_feedback_rear_ = 0;
    bool neutral_steering_hold_latched_ = false;
    int32_t neutral_hold_front_pulse_ = 0;
    int32_t neutral_hold_rear_pulse_ = 0;

    std::array<MotorStatus, 5> motor_status_{}; // CANopen node ID 1~4 사용
    SteeringRecoveryState recovery_state_ = SteeringRecoveryState::IDLE;
    std::chrono::steady_clock::time_point recovery_deadline_{};

    // 모드
    Mode mode_ = Mode::ALIGN;          // 활성 모드
    Mode target_mode_ = Mode::ALIGN;   // 전환 중 목표
    bool transitioning_ = false;
    bool mode_request_pending_ = false;
    Mode requested_mode_ = Mode::ALIGN;
    std::chrono::steady_clock::time_point mode_deadline_{};
    bool failed_ = false;

    bool motion_inhibited_ = false;
    bool awaiting_neutral_command_ = false;
    bool initialization_in_progress_ = false;
    bool initialization_complete_ = false;
    bool steering_auto_recovery_armed_ = true;
    bool steering_recovery_requested_ = false;
    bool recovery_was_requested_ = false;
    std::string initialization_state_ = "PENDING";
    std::string initialization_detail_ = "Waiting for motor initialization";

    int steering_profile_velocity_ = 40000;
    int steering_profile_acceleration_ = 40000;
    int steering_profile_deceleration_ = 40000;
    double max_steering_angle_rad_ = MAX_STEER_DEG * DEG2RAD;
    double max_linear_vel_ = 0.2;
    double max_angular_vel_ = 0.3;
    double cmd_vel_timeout_s_ = 0.5;
    double diff_steer_deg_ = 90.0;
    double diff_steer_front_deg_ = 0.0;
    double diff_steer_rear_deg_ = 0.0;
    double diff_max_linear_vel_ = 0.0;
    double diff_max_angular_vel_ = 0.0;
    bool diff_lateral_ = true;

    std::string odom_frame_ = "odom";
    std::string base_frame_ = "base_link";
    bool publish_odom_tf_ = false;
    double odom_v_deadband_ = 0.005;

    static constexpr int RECOVERY_ERROR_READ_MS = 250;
    static constexpr int RECOVERY_STEP_MS = 50;
    static constexpr int RECOVERY_FAULT_CLEAR_TIMEOUT_MS = 1000;
    static constexpr int RECOVERY_ENABLE_TIMEOUT_MS = 1000;
    static constexpr int RECOVERY_ZERO_TIMEOUT_MS = 30000;
    static constexpr int STATUS_FRESH_TIMEOUT_MS = 300;
    static constexpr int SETPOINT_RETRY_CYCLES = 50;
    static constexpr int32_t SETPOINT_STALL_PULSES = 1000;
    static constexpr int MODE_TRANSITION_TIMEOUT_MS = 30000;
    static constexpr double DRIVE_STOPPED_TOLERANCE_MPS = 0.01;
    static constexpr double RECOVERY_ZERO_TOLERANCE_RAD = 3.0 * M_PI / 180.0;
    static constexpr double DRIVE_STEERING_TOLERANCE_RAD = 3.0 * M_PI / 180.0;
    static constexpr double ODOM_MAX_DT_SEC = 0.5;

    rclcpp::Time last_cmd_time_;
    rclcpp::Time last_odom_time_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_mode_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_command_ack_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_steer_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_initialization_status_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_drive_mode_status_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_initialize_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_odom_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;

    // --- CAN Func ---
    int init_can_socket()
    {
        struct sockaddr_can addr;
        struct ifreq ifr;
        if ((can_sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
            return -1;
        strcpy(ifr.ifr_name, CAN_INTERFACE);
        if (ioctl(can_sock_, SIOCGIFINDEX, &ifr) < 0) {
            close(can_sock_);
            can_sock_ = -1;
            return -1;
        }
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(can_sock_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            close(can_sock_);
            can_sock_ = -1;
            return -1;
        }
        fcntl(can_sock_, F_SETFL, O_NONBLOCK);
        return 0;
    }

    void send_sdo_write(int id, uint16_t index, uint8_t sub, int data, uint8_t size)
    {
        struct can_frame frame
        {
        };
        frame.can_id = 0x600 + id;
        frame.can_dlc = 8;
        frame.data[0] = (size == 1) ? 0x2F : (size == 2 ? 0x2B : 0x23);
        frame.data[1] = index & 0xFF;
        frame.data[2] = (index >> 8) & 0xFF;
        frame.data[3] = sub;
        memcpy(&frame.data[4], &data, 4);
        if (write(can_sock_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
            RCLCPP_ERROR(this->get_logger(), "SDO write failed: ID=%d index=0x%04X", id,
                         static_cast<unsigned>(index));
        }
        usleep(2000);
    }

    bool send_sdo_read(int id, uint16_t index, uint8_t sub)
    {
        struct can_frame frame
        {
        };
        frame.can_id = 0x600 + id;
        frame.can_dlc = 8;
        frame.data[0] = 0x40; // expedited SDO upload request
        frame.data[1] = index & 0xFF;
        frame.data[2] = (index >> 8) & 0xFF;
        frame.data[3] = sub;

        if (write(can_sock_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
            RCLCPP_ERROR(this->get_logger(), "SDO read request failed: ID=%d index=0x%04X", id,
                         static_cast<unsigned>(index));
            return false;
        }
        return true;
    }

    void send_nmt_command(uint8_t cmd, uint8_t id)
    {
        struct can_frame frame
        {
        };
        frame.can_id = 0x000;
        frame.can_dlc = 2;
        frame.data[0] = cmd;
        frame.data[1] = id;
        if (write(can_sock_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
            RCLCPP_ERROR(this->get_logger(), "NMT write failed: cmd=0x%02X id=%d",
                         static_cast<unsigned>(cmd), id);
        }
        usleep(2000);
    }

    void setup_rpdo_mapping(int id, int mode)
    {
        send_sdo_write(id, 0x1400, 0x01, 0x80000200 + id, 4);
        send_sdo_write(id, 0x1600, 0x00, 0, 1);
        send_sdo_write(id, 0x1600, 0x01, 0x60400010, 4);
        if (mode == 3)
            send_sdo_write(id, 0x1600, 0x02, 0x60FF0020, 4);
        else
            send_sdo_write(id, 0x1600, 0x02, 0x607A0020, 4);
        send_sdo_write(id, 0x1600, 0x00, 2, 1);
        send_sdo_write(id, 0x1400, 0x01, 0x00000200 + id, 4);
    }

    void setup_tpdo_mapping(int id, int mode)
    {
        send_sdo_write(id, 0x1800, 0x01, 0x80000180 + id, 4);
        send_sdo_write(id, 0x1A00, 0x00, 0, 1);
        send_sdo_write(id, 0x1A00, 0x01, 0x60410010, 4);
        if (mode == 3)
            send_sdo_write(id, 0x1A00, 0x02, 0x606C0020, 4); // 주행: 실제 속도
        else
            send_sdo_write(id, 0x1A00, 0x02, 0x60640020, 4); // 조향: 실제 위치
        send_sdo_write(id, 0x1A00, 0x00, 2, 1);
        send_sdo_write(id, 0x1800, 0x02, 255, 1);
        send_sdo_write(id, 0x1800, 0x05, 50, 2); // 이벤트 타이머 50ms
        send_sdo_write(id, 0x1800, 0x01, 0x00000180 + id, 4);
    }

    void send_pdo(int id, int32_t val, uint16_t cw)
    {
        struct can_frame frame
        {
        };
        frame.can_id = 0x200 + id;
        frame.can_dlc = 6;
        memcpy(&frame.data[0], &cw, 2);
        memcpy(&frame.data[2], &val, 4);
        if (write(can_sock_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "PDO write failed: ID=%d", id);
        }
    }
    void send_drive_pdo(int id, int32_t val, uint16_t cw) { send_pdo(id, val, cw); }
    void send_steer_pdo(int id, int32_t val, uint16_t cw) { send_pdo(id, val, cw); }

    void init_motor_sdo(int id, int mode)
    {
        // Fault Reset bit 7 must see a low-to-high transition.
        send_sdo_write(id, OD_CONTROL_WORD, 0, 0x0000, 2);
        usleep(20000);
        send_sdo_write(id, OD_CONTROL_WORD, 0, 0x0080, 2);
        usleep(20000);
        send_sdo_write(id, OD_MODES_OF_OP, 0, mode, 1);
        if (mode == 1) { // 조향 모터 설정
            send_sdo_write(id, 0x6081, 0, steering_profile_velocity_, 4);
            send_sdo_write(id, 0x6083, 0, steering_profile_acceleration_, 4);
            send_sdo_write(id, 0x6084, 0, steering_profile_deceleration_, 4);
        } else {
            // 주행 모터 설정
            send_sdo_write(id, 0x6083, 0, 20000, 4);
            send_sdo_write(id, 0x6084, 0, 20000, 4);
            send_sdo_write(id, OD_TARGET_VEL, 0, 0, 4);
        }
        send_sdo_write(id, OD_CONTROL_WORD, 0, 0x0006, 2);
        usleep(50000);
        send_sdo_write(id, OD_CONTROL_WORD, 0, 0x0007, 2);
        usleep(50000);
        send_sdo_write(id, OD_CONTROL_WORD, 0, 0x000F, 2);
        usleep(50000);
    }

    // --- 상태 ---
    void set_initialization_status(const std::string& state, const std::string& detail,
                                   bool complete)
    {
        initialization_state_ = state;
        initialization_detail_ = detail;
        initialization_complete_ = complete;

        if (pub_initialization_status_) {
            std_msgs::msg::String status;
            status.data = state + "|" + detail;
            pub_initialization_status_->publish(status);
        }
    }

    static const char* mode_name(Mode m)
    {
        switch (m) {
        case Mode::ALIGN:
            return "ALIGN";
        case Mode::ACKERMANN:
            return "ACKERMANN";
        case Mode::DIFF:
            return "DIFF";
        }
        return "ALIGN";
    }

    void publish_mode_status(const std::string& detail)
    {
        if (!pub_drive_mode_status_)
            return;
        std_msgs::msg::String status;
        std::string name = failed_ ? "FAILED"
                           : transitioning_
                               ? std::string("TO_") + mode_name(target_mode_)
                               : mode_name(mode_);
        status.data = name + "|" + detail;
        pub_drive_mode_status_->publish(status);
    }

    bool drive_is_stopped() const
    {
        return std::abs(current_drive_vel_front_) <= DRIVE_STOPPED_TOLERANCE_MPS &&
               std::abs(current_drive_vel_rear_) <= DRIVE_STOPPED_TOLERANCE_MPS;
    }

    static int32_t deg_to_steer_pulse(double deg)
    {
        return static_cast<int32_t>(std::llround(deg * STEER_RATIO * ENCODER_PPR / 360.0));
    }

    double diff_front_deg() const
    {
        return diff_steer_front_deg_ != 0.0 ? diff_steer_front_deg_ : diff_steer_deg_;
    }
    double diff_rear_deg() const
    {
        return diff_steer_rear_deg_ != 0.0 ? diff_steer_rear_deg_ : diff_steer_deg_;
    }

    // 모드별 조향 목표각 [deg]. ALIGN/ACKERMANN 은 0 (애커만 주행 중 각도는 제어 루프가 결정).
    void mode_steer_targets_deg(Mode m, double& front, double& rear) const
    {
        if (m == Mode::DIFF) {
            front = diff_front_deg();
            rear = diff_rear_deg();
        } else {
            front = 0.0;
            rear = 0.0;
        }
    }

    void fail_mode(const std::string& reason)
    {
        stop_drive_motors();
        cmd_vx_ = cmd_vy_ = cmd_wz_ = 0.0;
        transitioning_ = false;
        mode_request_pending_ = false;
        failed_ = true;
        motion_inhibited_ = true;
        publish_mode_status(reason);
        RCLCPP_ERROR(this->get_logger(), "Mode transition failed: %s", reason.c_str());
    }

    bool all_motors_operational() const
    {
        for (const int id : {ID_FRONT_DRIVE, ID_FRONT_STEER, ID_REAR_DRIVE, ID_REAR_STEER}) {
            if (!status_is_fresh(id) || has_fault(id) ||
                !operation_enabled(motor_status_[id].statusword)) {
                return false;
            }
        }
        return true;
    }

    bool has_fault(int id) const { return (motor_status_[id].statusword & (1U << 3)) != 0; }
    bool is_servo_running(int id) const
    {
        return (motor_status_[id].statusword & (1U << 2)) != 0;
    }
    bool setpoint_acknowledged(int id) const
    {
        return (motor_status_[id].statusword & STATUS_SETPOINT_ACKNOWLEDGED) != 0;
    }
    bool has_internal_limit(int id) const
    {
        return (motor_status_[id].statusword & (1U << 11)) != 0;
    }

    bool status_is_fresh(int id) const
    {
        if (!motor_status_[id].tpdo_received)
            return false;
        const auto age = std::chrono::steady_clock::now() - motor_status_[id].last_tpdo;
        return age <= std::chrono::milliseconds(STATUS_FRESH_TIMEOUT_MS);
    }

    bool both_steering_status_fresh() const
    {
        return status_is_fresh(ID_FRONT_STEER) && status_is_fresh(ID_REAR_STEER);
    }
    bool both_drive_status_fresh() const
    {
        return status_is_fresh(ID_FRONT_DRIVE) && status_is_fresh(ID_REAR_DRIVE);
    }
    bool steering_fault_active() const
    {
        return has_fault(ID_FRONT_STEER) || has_fault(ID_REAR_STEER);
    }
    bool drive_fault_active() const
    {
        return has_fault(ID_FRONT_DRIVE) || has_fault(ID_REAR_DRIVE);
    }

    void request_error_code(int id)
    {
        auto& status = motor_status_[id];
        if (status.error_read_pending)
            return;

        status.error_code_valid = false;
        status.error_read_pending = send_sdo_read(id, OD_ERROR_CODE, 0);
    }

    void handle_sdo_response(int id, const struct can_frame& frame)
    {
        if (id < 1 || id > 4 || frame.can_dlc < 8)
            return;

        uint16_t index = 0;
        memcpy(&index, &frame.data[1], sizeof(index));
        const uint8_t sub = frame.data[3];

        if (frame.data[0] == 0x80) {
            uint32_t abort_code = 0;
            memcpy(&abort_code, &frame.data[4], sizeof(abort_code));
            motor_status_[id].error_read_pending = false;
            RCLCPP_ERROR(this->get_logger(), "SDO abort: ID=%d index=0x%04X sub=%u code=0x%08X", id,
                         static_cast<unsigned>(index), static_cast<unsigned>(sub),
                         static_cast<unsigned>(abort_code));
            return;
        }

        // 603Fh는 Uint16이므로 expedited upload 응답은 0x4B이다.
        if (frame.data[0] == 0x4B && index == OD_ERROR_CODE && sub == 0) {
            uint16_t error_code = 0;
            memcpy(&error_code, &frame.data[4], sizeof(error_code));

            auto& status = motor_status_[id];
            status.error_code = error_code;
            status.error_code_valid = true;
            status.error_read_pending = false;

            if (error_code == 0) {
                RCLCPP_INFO(this->get_logger(), "Motor ID %d reports no error (0x0000)", id);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Motor ID %d error code: 0x%04X", id,
                             static_cast<unsigned>(error_code));
            }
        }
    }

    void read_can_messages()
    {
        struct can_frame frame
        {
        };
        while (read(can_sock_, &frame, sizeof(frame)) > 0) {
            const uint32_t cob_id = frame.can_id & CAN_SFF_MASK;

            // TPDO1: statusword(2 byte) + velocity/position feedback(4 byte)
            if (cob_id >= 0x181 && cob_id <= 0x184 && frame.can_dlc >= 6) {
                const int id = static_cast<int>(cob_id - 0x180);
                auto& status = motor_status_[id];
                const bool previous_fault = has_fault(id);
                const bool previous_limit = has_internal_limit(id);

                memcpy(&status.statusword, &frame.data[0], sizeof(status.statusword));
                status.tpdo_received = true;
                status.last_tpdo = std::chrono::steady_clock::now();

                int32_t feedback = 0;
                memcpy(&feedback, &frame.data[2], sizeof(feedback));

                if (id == ID_FRONT_DRIVE) {
                    const double rpm = static_cast<double>(feedback) * 60.0 / ENCODER_PPR;
                    current_drive_vel_front_ = (rpm / 60.0) * WHEEL_CIRCUM / DRIVE_RATIO;
                } else if (id == ID_REAR_DRIVE) {
                    const double rpm = static_cast<double>(feedback) * 60.0 / ENCODER_PPR;
                    current_drive_vel_rear_ = (rpm / 60.0) * WHEEL_CIRCUM / DRIVE_RATIO;
                } else if (id == ID_FRONT_STEER) {
                    current_steer_pulse_front_ = feedback;
                    const double deg =
                        static_cast<double>(feedback) * 360.0 / (STEER_RATIO * ENCODER_PPR);
                    current_steer_angle_front_ = deg * DEG2RAD;
                } else if (id == ID_REAR_STEER) {
                    current_steer_pulse_rear_ = feedback;
                    const double deg =
                        static_cast<double>(feedback) * 360.0 / (STEER_RATIO * ENCODER_PPR);
                    current_steer_angle_rear_ = deg * DEG2RAD;
                }

                const bool current_fault = has_fault(id);
                if (current_fault && !previous_fault) {
                    motion_inhibited_ = true;
                    RCLCPP_ERROR(this->get_logger(),
                                 "Motor ID %d FAULT, statusword=0x%04X, "
                                 "steering(front/rear)=%.1f/%.1f deg",
                                 id, static_cast<unsigned>(status.statusword),
                                 current_steer_angle_front_ * RAD2DEG,
                                 current_steer_angle_rear_ * RAD2DEG);
                    request_error_code(id);
                    if (id == ID_FRONT_DRIVE || id == ID_REAR_DRIVE) {
                        set_initialization_status(
                            "FAILED", "Drive motor fault detected, ID " + std::to_string(id),
                            false);
                    }
                } else if (!current_fault && previous_fault) {
                    RCLCPP_INFO(this->get_logger(), "Motor ID %d fault cleared", id);
                }

                const bool current_limit = has_internal_limit(id);
                if (current_limit && !previous_limit) {
                    RCLCPP_WARN(this->get_logger(), "Motor ID %d internal limit active", id);
                }
            }
            // SDO response
            else if (cob_id >= 0x581 && cob_id <= 0x584) {
                const int id = static_cast<int>(cob_id - 0x580);
                handle_sdo_response(id, frame);
            }
        }
    }

    void stop_drive_motors()
    {
        send_drive_pdo(ID_FRONT_DRIVE, 0, 0x000F);
        send_drive_pdo(ID_REAR_DRIVE, 0, 0x000F);
    }

    // --- 4WS 오도메트리 (모드 무관, 측정값만 사용) ---
    bool odom_inputs_valid() const
    {
        return both_steering_status_fresh() && both_drive_status_fresh();
    }

    void compute_body_velocity(double& vx, double& vy, double& wz) const
    {
        double v_f = current_drive_vel_front_;
        double v_r = current_drive_vel_rear_;
        if (std::abs(v_f) < odom_v_deadband_)
            v_f = 0.0;
        if (std::abs(v_r) < odom_v_deadband_)
            v_r = 0.0;

        const double cf = std::cos(current_steer_angle_front_);
        const double sf = std::sin(current_steer_angle_front_);
        const double cr = std::cos(current_steer_angle_rear_);
        const double sr = std::sin(current_steer_angle_rear_);

        vx = 0.5 * (v_f * cf + v_r * cr);
        vy = 0.5 * (v_f * sf + v_r * sr);
        wz = (v_f * sf - v_r * sr) / WHEELBASE;
    }

    void update_odometry()
    {
        const rclcpp::Time now = this->now();
        double dt = (now - last_odom_time_).seconds();
        last_odom_time_ = now;

        odom_valid_ = odom_inputs_valid();
        if (odom_valid_) {
            compute_body_velocity(meas_vx_, meas_vy_, meas_wz_);
        } else {
            meas_vx_ = meas_vy_ = meas_wz_ = 0.0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "Odometry inputs unavailable (steer %s, drive %s); pose frozen",
                                 both_steering_status_fresh() ? "OK" : "STALE",
                                 both_drive_status_fresh() ? "OK" : "STALE");
        }

        // 2차 정확도(midpoint) 적분. 시계 점프(sim time 리셋 등)는 적분하지 않는다.
        if (odom_valid_ && dt > 0.0 && dt <= ODOM_MAX_DT_SEC) {
            const double th_mid = th_ + 0.5 * meas_wz_ * dt;
            const double c = std::cos(th_mid);
            const double s = std::sin(th_mid);
            x_ += (meas_vx_ * c - meas_vy_ * s) * dt;
            y_ += (meas_vx_ * s + meas_vy_ * c) * dt;
            th_ = normalize_angle(th_ + meas_wz_ * dt);
        }

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, th_);
        geometry_msgs::msg::Quaternion qm;
        qm.x = q.x();
        qm.y = q.y();
        qm.z = q.z();
        qm.w = q.w();

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = qm;
        // 몸체(base_link) 기준 트위스트. 횡속도 vy 도 그대로 싣는다.
        odom.twist.twist.linear.x = meas_vx_;
        odom.twist.twist.linear.y = meas_vy_;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = meas_wz_;

        // 공분산. 입력이 무효하면 크게 키워 상위 융합기가 무시하도록 한다.
        const bool moving = odom_valid_ && (meas_vx_ != 0.0 || meas_vy_ != 0.0 || meas_wz_ != 0.0);
        const double pxy = !odom_valid_ ? 1e6 : (moving ? 1e-2 : 1e-4);
        const double pth = !odom_valid_ ? 1e6 : (moving ? 5e-2 : 1e-3);
        const double txy = !odom_valid_ ? 1e6 : (moving ? 1e-3 : 1e-5);
        const double tth = !odom_valid_ ? 1e6 : (moving ? 1e-2 : 1e-4);
        for (auto& c_ : odom.pose.covariance)
            c_ = 0.0;
        for (auto& c_ : odom.twist.covariance)
            c_ = 0.0;
        odom.pose.covariance[0] = pxy;
        odom.pose.covariance[7] = pxy;
        odom.pose.covariance[14] = 1e6;
        odom.pose.covariance[21] = 1e6;
        odom.pose.covariance[28] = 1e6;
        odom.pose.covariance[35] = pth;
        odom.twist.covariance[0] = txy;
        odom.twist.covariance[7] = txy;
        odom.twist.covariance[14] = 1e6;
        odom.twist.covariance[21] = 1e6;
        odom.twist.covariance[28] = 1e6;
        odom.twist.covariance[35] = tth;
        pub_odom_->publish(odom);

        if (publish_odom_tf_) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = now;
            tf.header.frame_id = odom_frame_;
            tf.child_frame_id = base_frame_;
            tf.transform.translation.x = x_;
            tf.transform.translation.y = y_;
            tf.transform.translation.z = 0.0;
            tf.transform.rotation = qm;
            tf_broadcaster_->sendTransform(tf);
        }
    }

    void publish_steer_angle()
    {
        std_msgs::msg::Float32MultiArray m;
        m.data = {static_cast<float>(current_steer_angle_front_ * RAD2DEG),
                  static_cast<float>(current_steer_angle_rear_ * RAD2DEG),
                  static_cast<float>(steer_cmd_deg_front_),
                  static_cast<float>(steer_cmd_deg_rear_)};
        pub_steer_->publish(m);
    }

    // --- 진단 ---
    diagnostic_msgs::msg::DiagnosticStatus make_motor_diagnostic(int id,
                                                                 const std::string& name) const
    {
        diagnostic_msgs::msg::DiagnosticStatus diagnostic;
        diagnostic.name = name;
        diagnostic.hardware_id = std::string(CAN_INTERFACE) + ":id_" + std::to_string(id);

        const auto& motor = motor_status_[id];
        const bool fresh =
            motor.tpdo_received &&
            std::chrono::steady_clock::now() - motor.last_tpdo <= MOTOR_FEEDBACK_TIMEOUT;

        if (!fresh) {
            diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
            diagnostic.message = "feedback_stale";
        } else if (has_fault(id)) {
            diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            diagnostic.message = "fault";
        } else if (operation_enabled(motor.statusword)) {
            diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            diagnostic.message = "operational";
        } else {
            diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            diagnostic.message = "not_operation_enabled";
        }

        char buf[32]{};
        diagnostic_msgs::msg::KeyValue kv;
        std::snprintf(buf, sizeof(buf), "0x%04X", static_cast<unsigned>(motor.statusword));
        kv.key = "status_word";
        kv.value = buf;
        diagnostic.values.push_back(kv);

        if (motor.error_code_valid) {
            std::snprintf(buf, sizeof(buf), "0x%04X", static_cast<unsigned>(motor.error_code));
            kv.key = "error_code";
            kv.value = buf;
            diagnostic.values.push_back(kv);
        }
        if (id == ID_FRONT_DRIVE || id == ID_REAR_DRIVE) {
            std::snprintf(buf, sizeof(buf), "%+.3f",
                          id == ID_FRONT_DRIVE ? current_drive_vel_front_
                                               : current_drive_vel_rear_);
            kv.key = "wheel_mps";
            kv.value = buf;
            diagnostic.values.push_back(kv);
        } else {
            std::snprintf(buf, sizeof(buf), "%+.2f",
                          (id == ID_FRONT_STEER ? current_steer_angle_front_
                                                : current_steer_angle_rear_) *
                              RAD2DEG);
            kv.key = "steer_deg";
            kv.value = buf;
            diagnostic.values.push_back(kv);
            std::snprintf(buf, sizeof(buf), "%+.2f",
                          id == ID_FRONT_STEER ? steer_cmd_deg_front_ : steer_cmd_deg_rear_);
            kv.key = "steer_cmd_deg";
            kv.value = buf;
            diagnostic.values.push_back(kv);
        }
        return diagnostic;
    }

    void publish_diagnostics()
    {
        if (!pub_diagnostics_)
            return;

        if (pub_initialization_status_) {
            std_msgs::msg::String status;
            status.data = initialization_state_ + "|" + initialization_detail_;
            pub_initialization_status_->publish(status);
        }

        diagnostic_msgs::msg::DiagnosticArray diagnostics;
        diagnostics.header.stamp = this->now();
        diagnostics.status.reserve(5);
        diagnostics.status.push_back(
            make_motor_diagnostic(ID_FRONT_DRIVE, "motor_node/front_drive"));
        diagnostics.status.push_back(
            make_motor_diagnostic(ID_FRONT_STEER, "motor_node/front_steering"));
        diagnostics.status.push_back(make_motor_diagnostic(ID_REAR_DRIVE, "motor_node/rear_drive"));
        diagnostics.status.push_back(
            make_motor_diagnostic(ID_REAR_STEER, "motor_node/rear_steering"));

        diagnostic_msgs::msg::DiagnosticStatus node;
        node.name = "motor_node/node";
        node.hardware_id = CAN_INTERFACE;
        if (failed_ || recovery_state_ == SteeringRecoveryState::FAILED ||
            initialization_state_ == "FAILED") {
            node.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        } else if (motion_inhibited_ || !odom_valid_ || transitioning_) {
            node.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        } else {
            node.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        }
        node.message = initialization_state_ + ": " + initialization_detail_;
        char buf[48]{};
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = "mode";
        kv.value = transitioning_ ? std::string("TO_") + mode_name(target_mode_)
                                  : mode_name(mode_);
        node.values.push_back(kv);
        kv.key = "odom_valid";
        kv.value = odom_valid_ ? "true" : "false";
        node.values.push_back(kv);
        kv.key = "motion_inhibited";
        kv.value = motion_inhibited_ ? "true" : "false";
        node.values.push_back(kv);
        kv.key = "awaiting_neutral_cmd";
        kv.value = awaiting_neutral_command_ ? "true" : "false";
        node.values.push_back(kv);
        std::snprintf(buf, sizeof(buf), "%+.3f %+.3f %+.3f", meas_vx_, meas_vy_, meas_wz_);
        kv.key = "twist_vx_vy_wz";
        kv.value = buf;
        node.values.push_back(kv);
        diagnostics.status.push_back(node);

        pub_diagnostics_->publish(diagnostics);
    }

    // --- 조향 복구 / 초기화 상태머신 ---
    void fail_steering_recovery(const char* reason)
    {
        recovery_state_ = SteeringRecoveryState::FAILED;
        steering_recovery_requested_ = false;
        recovery_was_requested_ = false;
        initialization_in_progress_ = false;
        initialization_complete_ = false;
        motion_inhibited_ = true;
        stop_drive_motors();
        set_initialization_status("FAILED", std::string("Steering recovery failed: ") + reason,
                                  false);
        RCLCPP_ERROR(this->get_logger(),
                     "Steering auto-recovery FAILED: %s. Motion remains inhibited.", reason);
    }

    void start_steering_recovery(bool requested)
    {
        recovery_was_requested_ = requested;
        if (!requested) {
            steering_auto_recovery_armed_ = false;
        }
        initialization_in_progress_ = true;
        initialization_complete_ = false;
        motion_inhibited_ = true;
        cmd_vx_ = cmd_vy_ = cmd_wz_ = 0.0;
        transitioning_ = false;
        stop_drive_motors();
        set_initialization_status("INITIALIZING",
                                  requested ? "Lower motor initialization and steering centering"
                                            : "Automatic steering fault recovery",
                                  false);

        // Fault 시점의 오류코드를 양쪽 조향축 모두에서 읽는다.
        for (const int id : {ID_FRONT_STEER, ID_REAR_STEER}) {
            if (requested) {
                motor_status_[id].error_code_valid = false;
                motor_status_[id].error_read_pending = false;
            }
            if (!motor_status_[id].error_read_pending) {
                request_error_code(id);
            }
        }

        recovery_state_ = SteeringRecoveryState::WAIT_ERROR_CODE;
        recovery_deadline_ =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(RECOVERY_ERROR_READ_MS);

        if (requested) {
            RCLCPP_WARN(this->get_logger(),
                        "Lower motor initialization started. Drive is stopped while "
                        "steering IDs 2 and 4 return to zero.");
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Steering fault detected. Stop drive and start one-shot "
                         "auto-recovery.");
        }
    }

    // true 를 돌려주면 이번 틱에는 일반 주행 제어를 하지 않는다.
    bool process_steering_recovery()
    {
        const auto now = std::chrono::steady_clock::now();

        // 주행축 Fault는 자동 복구하지 않고 안전 정지한다.
        if (drive_fault_active()) {
            motion_inhibited_ = true;
            stop_drive_motors();
            if (steering_recovery_requested_ || initialization_in_progress_) {
                steering_recovery_requested_ = false;
                recovery_was_requested_ = false;
                initialization_in_progress_ = false;
                initialization_complete_ = false;
                recovery_state_ = SteeringRecoveryState::FAILED;
                set_initialization_status(
                    "FAILED", "Drive motor fault detected during initialization", false);
            }
            return true;
        }

        if (recovery_state_ == SteeringRecoveryState::IDLE) {
            if (steering_recovery_requested_) {
                steering_recovery_requested_ = false;
                start_steering_recovery(true);
                return true;
            }
            if (!steering_fault_active())
                return motion_inhibited_;
            if (!steering_auto_recovery_armed_) {
                fail_steering_recovery("fault recurred after the one-shot automatic recovery");
                return true;
            }
            start_steering_recovery(false);
            return true;
        }

        stop_drive_motors();

        if (recovery_state_ == SteeringRecoveryState::FAILED) {
            return true;
        }

        switch (recovery_state_) {
        case SteeringRecoveryState::WAIT_ERROR_CODE: {
            const bool both_error_codes_received = motor_status_[ID_FRONT_STEER].error_code_valid &&
                                                   motor_status_[ID_REAR_STEER].error_code_valid;

            if (!both_error_codes_received && now < recovery_deadline_)
                return true;

            // Fault Reset Bit 7의 상승 에지를 만들기 위해 먼저 LOW로 내린다.
            send_sdo_write(ID_FRONT_STEER, OD_CONTROL_WORD, 0, 0x0000, 2);
            send_sdo_write(ID_REAR_STEER, OD_CONTROL_WORD, 0, 0x0000, 2);
            recovery_state_ = SteeringRecoveryState::WAIT_RESET_LOW;
            recovery_deadline_ = now + std::chrono::milliseconds(RECOVERY_STEP_MS);
            return true;
        }

        case SteeringRecoveryState::WAIT_RESET_LOW:
            if (now < recovery_deadline_)
                return true;

            send_sdo_write(ID_FRONT_STEER, OD_CONTROL_WORD, 0, 0x0080, 2);
            send_sdo_write(ID_REAR_STEER, OD_CONTROL_WORD, 0, 0x0080, 2);
            recovery_state_ = SteeringRecoveryState::WAIT_FAULT_CLEAR;
            recovery_deadline_ = now + std::chrono::milliseconds(RECOVERY_FAULT_CLEAR_TIMEOUT_MS);
            RCLCPP_WARN(this->get_logger(), "Fault Reset sent to steering IDs 2 and 4");
            return true;

        case SteeringRecoveryState::WAIT_FAULT_CLEAR:
            if (both_steering_status_fresh() && !steering_fault_active()) {
                // 이전 제한각 목표가 재사용되지 않도록 Enable 전에 목표 위치를 0으로 쓴다.
                send_sdo_write(ID_FRONT_STEER, OD_TARGET_POS, 0, 0, 4);
                send_sdo_write(ID_REAR_STEER, OD_TARGET_POS, 0, 0, 4);
                send_sdo_write(ID_FRONT_STEER, OD_CONTROL_WORD, 0, 0x0006, 2);
                send_sdo_write(ID_REAR_STEER, OD_CONTROL_WORD, 0, 0x0006, 2);
                recovery_state_ = SteeringRecoveryState::WAIT_SHUTDOWN;
                recovery_deadline_ = now + std::chrono::milliseconds(RECOVERY_STEP_MS);
                return true;
            }
            if (now >= recovery_deadline_) {
                fail_steering_recovery("fault bit did not clear");
            }
            return true;

        case SteeringRecoveryState::WAIT_SHUTDOWN:
            if (now < recovery_deadline_)
                return true;

            send_sdo_write(ID_FRONT_STEER, OD_CONTROL_WORD, 0, 0x0007, 2);
            send_sdo_write(ID_REAR_STEER, OD_CONTROL_WORD, 0, 0x0007, 2);
            recovery_state_ = SteeringRecoveryState::WAIT_SWITCH_ON;
            recovery_deadline_ = now + std::chrono::milliseconds(RECOVERY_STEP_MS);
            return true;

        case SteeringRecoveryState::WAIT_SWITCH_ON:
            if (now < recovery_deadline_)
                return true;

            send_sdo_write(ID_FRONT_STEER, OD_CONTROL_WORD, 0, 0x000F, 2);
            send_sdo_write(ID_REAR_STEER, OD_CONTROL_WORD, 0, 0x000F, 2);
            recovery_state_ = SteeringRecoveryState::WAIT_ENABLE;
            recovery_deadline_ = now + std::chrono::milliseconds(RECOVERY_ENABLE_TIMEOUT_MS);
            return true;

        case SteeringRecoveryState::WAIT_ENABLE:
            if (steering_fault_active()) {
                fail_steering_recovery("fault occurred again while enabling");
                return true;
            }

            if (both_steering_status_fresh() && is_servo_running(ID_FRONT_STEER) &&
                is_servo_running(ID_REAR_STEER)) {
                // Profile Position의 New set-point Bit 4를 상승시킨다.
                send_steer_pdo(ID_FRONT_STEER, 0, 0x003F);
                send_steer_pdo(ID_REAR_STEER, 0, 0x003F);
                recovery_state_ = SteeringRecoveryState::WAIT_SETPOINT_LOW;
                recovery_deadline_ = now + std::chrono::milliseconds(RECOVERY_STEP_MS);
                return true;
            }

            if (now >= recovery_deadline_) {
                fail_steering_recovery("steering servo did not enter running state");
            }
            return true;

        case SteeringRecoveryState::WAIT_SETPOINT_LOW:
            if (now < recovery_deadline_)
                return true;

            send_steer_pdo(ID_FRONT_STEER, 0, 0x000F);
            send_steer_pdo(ID_REAR_STEER, 0, 0x000F);
            recovery_state_ = SteeringRecoveryState::WAIT_ZERO;
            recovery_deadline_ = now + std::chrono::milliseconds(RECOVERY_ZERO_TIMEOUT_MS);
            RCLCPP_WARN(this->get_logger(), "Steering IDs 2 and 4 commanded to zero position");
            return true;

        case SteeringRecoveryState::WAIT_ZERO: {
            if (steering_fault_active()) {
                fail_steering_recovery("fault occurred again while returning to zero");
                return true;
            }

            // 목표값을 유지하되 New set-point Bit는 다시 올리지 않는다.
            send_steer_pdo(ID_FRONT_STEER, 0, 0x000F);
            send_steer_pdo(ID_REAR_STEER, 0, 0x000F);
            steer_cmd_deg_front_ = steer_cmd_deg_rear_ = 0.0;

            const bool at_zero =
                both_steering_status_fresh() &&
                std::abs(current_steer_angle_front_) <= RECOVERY_ZERO_TOLERANCE_RAD &&
                std::abs(current_steer_angle_rear_) <= RECOVERY_ZERO_TOLERANCE_RAD;

            if (at_zero) {
                if (!all_motors_operational()) {
                    fail_steering_recovery("one or more lower motors are not operational");
                    return true;
                }
                last_pulse_front_ = 0;
                last_pulse_rear_ = 0;
                toggle_front_ = 0;
                toggle_rear_ = 0;
                setpoint_retry_front_ = 0;
                setpoint_retry_rear_ = 0;
                setpoint_feedback_front_ = 0;
                setpoint_feedback_rear_ = 0;
                neutral_steering_hold_latched_ = false;
                cmd_vx_ = cmd_vy_ = cmd_wz_ = 0.0;
                last_cmd_time_ = this->now();
                // 센터링 완료 -> mode 0(정렬)에서 대기. /mode 로 주행 모드를 골라야 움직인다.
                mode_ = Mode::ALIGN;
                target_mode_ = Mode::ALIGN;
                transitioning_ = false;
                failed_ = false;
                awaiting_neutral_command_ = false;
                motion_inhibited_ = false;
                initialization_in_progress_ = false;
                recovery_state_ = SteeringRecoveryState::IDLE;
                if (recovery_was_requested_) {
                    steering_auto_recovery_armed_ = true;
                }
                recovery_was_requested_ = false;
                set_initialization_status(
                    "READY", "All four lower motors operational; steering centered", true);
                publish_mode_status("Steering centered; waiting for /mode (1=ackermann, 2=diff)");
                RCLCPP_INFO(this->get_logger(),
                            "Steering centered: IDs 2 and 4 are at zero. mode 0 (ALIGN); "
                            "send /mode 1 or 2 to drive.");
                return true;
            }

            if (now >= recovery_deadline_) {
                fail_steering_recovery("steering did not reach zero position");
            }
            return true;
        }

        case SteeringRecoveryState::IDLE:
        case SteeringRecoveryState::FAILED:
            return true;
        }

        return true;
    }

    // --- 모드 요청 / 전환 ---
    void mode_cb(const std_msgs::msg::Int32::SharedPtr msg)
    {
        const int m = msg->data;
        if (m < 0 || m > 2) {
            RCLCPP_WARN(this->get_logger(), "Unknown mode %d ignored (0=align, 1=ackermann, 2=diff)",
                        m);
            return;
        }
        requested_mode_ = static_cast<Mode>(m);
        mode_request_pending_ = true;
        RCLCPP_INFO(this->get_logger(), "mode %d (%s) requested", m, mode_name(requested_mode_));
    }

    // 제어 루프에서 호출. 준비가 안 됐으면 요청을 보류한다.
    void process_mode_request()
    {
        if (!mode_request_pending_)
            return;

        if (!initialization_complete_ || initialization_in_progress_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "mode %s pending until initialization completes",
                                 mode_name(requested_mode_));
            return;
        }
        if (failed_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "mode %s ignored: node is FAILED; call motor_node/initialize",
                                 mode_name(requested_mode_));
            mode_request_pending_ = false;
            return;
        }
        mode_request_pending_ = false;

        if (!transitioning_ && requested_mode_ == mode_) {
            publish_mode_status(std::string(mode_name(mode_)) + " already active");
            return;
        }
        if (transitioning_ && requested_mode_ == target_mode_)
            return;

        // 전환 시작: 주행 정지 -> 조향 목표각 이동 -> 도달 시 활성화
        stop_drive_motors();
        cmd_vx_ = cmd_vy_ = cmd_wz_ = 0.0;
        awaiting_neutral_command_ = false;
        neutral_steering_hold_latched_ = false;
        target_mode_ = requested_mode_;
        transitioning_ = true;
        mode_deadline_ = std::chrono::steady_clock::now() +
                         std::chrono::milliseconds(MODE_TRANSITION_TIMEOUT_MS);
        double f = 0.0, r = 0.0;
        mode_steer_targets_deg(target_mode_, f, r);
        publish_mode_status("Drive stopped; steering toward target");
        RCLCPP_WARN(this->get_logger(), "Mode %s -> %s: steering to F/R=%.1f/%.1f deg",
                    mode_name(mode_), mode_name(target_mode_), f, r);
    }

    bool process_mode_transition()
    {
        if (!transitioning_)
            return false;

        stop_drive_motors();
        const auto now = std::chrono::steady_clock::now();
        if (now >= mode_deadline_) {
            fail_mode(std::string("Steering alignment for ") + mode_name(target_mode_) +
                      " timed out");
            return true;
        }
        if (!all_motors_operational()) {
            fail_mode("Motor feedback stale or motor not operational");
            return true;
        }

        // 주행축이 실제로 멈춘 뒤에만 조향축을 큰 각도로 전환한다.
        if (!drive_is_stopped()) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Waiting for drive stop before mode transition: front=%.3f rear=%.3f m/s",
                current_drive_vel_front_, current_drive_vel_rear_);
            return true;
        }

        double f_deg = 0.0, r_deg = 0.0;
        mode_steer_targets_deg(target_mode_, f_deg, r_deg);
        steer_cmd_deg_front_ = f_deg;
        steer_cmd_deg_rear_ = r_deg;
        const int32_t target_front = deg_to_steer_pulse(f_deg);
        const int32_t target_rear = deg_to_steer_pulse(r_deg);
        handle_steer_toggle(ID_FRONT_STEER, target_front, last_pulse_front_, toggle_front_,
                            setpoint_retry_front_, setpoint_feedback_front_,
                            current_steer_pulse_front_);
        handle_steer_toggle(ID_REAR_STEER, target_rear, last_pulse_rear_, toggle_rear_,
                            setpoint_retry_rear_, setpoint_feedback_rear_,
                            current_steer_pulse_rear_);

        const int32_t tolerance = deg_to_steer_pulse(3.0);
        const bool target_reached =
            std::llabs(static_cast<long long>(target_front) - current_steer_pulse_front_) <=
                tolerance &&
            std::llabs(static_cast<long long>(target_rear) - current_steer_pulse_rear_) <=
                tolerance;
        if (!target_reached) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Mode transition to %s: target(F/R)=%.1f/%.1f actual(F/R)=%.1f/%.1f deg",
                                 mode_name(target_mode_), f_deg, r_deg,
                                 current_steer_angle_front_ * RAD2DEG,
                                 current_steer_angle_rear_ * RAD2DEG);
            return true;
        }

        mode_ = target_mode_;
        transitioning_ = false;
        cmd_vx_ = cmd_vy_ = cmd_wz_ = 0.0;
        neutral_steering_hold_latched_ = false;
        last_cmd_time_ = this->now();
        if (mode_ == Mode::ACKERMANN) {
            // Nav2 가 같은 제한각 명령을 계속 보내 재차 걸리지 않도록 중립 명령을 한 번 요구한다.
            awaiting_neutral_command_ = true;
            publish_mode_status("Ackermann 4WS ready; waiting for neutral cmd_vel");
        } else if (mode_ == Mode::DIFF) {
            awaiting_neutral_command_ = false;
            publish_mode_status("Diff steering aligned; cmd_vel linear.x/y=lateral, angular.z=spin");
        } else {
            awaiting_neutral_command_ = false;
            publish_mode_status("Steering centered; drive disabled");
        }
        RCLCPP_INFO(this->get_logger(), "Mode %s active (steer F/R=%.1f/%.1f deg)",
                    mode_name(mode_), current_steer_angle_front_ * RAD2DEG,
                    current_steer_angle_rear_ * RAD2DEG);
        return true;
    }

    // --- 모드별 제어 ---
    void control_align_mode()
    {
        stop_drive_motors();
        steer_cmd_deg_front_ = steer_cmd_deg_rear_ = 0.0;
        if (!both_steering_status_fresh() || steering_fault_active())
            return;
        handle_steer_toggle(ID_FRONT_STEER, 0, last_pulse_front_, toggle_front_,
                            setpoint_retry_front_, setpoint_feedback_front_,
                            current_steer_pulse_front_);
        handle_steer_toggle(ID_REAR_STEER, 0, last_pulse_rear_, toggle_rear_,
                            setpoint_retry_rear_, setpoint_feedback_rear_,
                            current_steer_pulse_rear_);
    }

    // 조향을 diff 각도로 고정한 채 차동구동처럼 움직인다.
    //   바퀴 i (x_i = ±L/2) 의 접선 속도 = (v_lat + wz·x_i) / sin(δ_i)
    // compute_body_velocity() 의 정확한 역이라, odom 이 되돌리는 vy/wz 가 지령과 일치한다.
    void control_diff_mode(bool cmd_timeout)
    {
        const double f_deg = diff_front_deg();
        const double r_deg = diff_rear_deg();
        steer_cmd_deg_front_ = f_deg;
        steer_cmd_deg_rear_ = r_deg;
        const int32_t target_front = deg_to_steer_pulse(f_deg);
        const int32_t target_rear = deg_to_steer_pulse(r_deg);
        handle_steer_toggle(ID_FRONT_STEER, target_front, last_pulse_front_, toggle_front_,
                            setpoint_retry_front_, setpoint_feedback_front_,
                            current_steer_pulse_front_);
        handle_steer_toggle(ID_REAR_STEER, target_rear, last_pulse_rear_, toggle_rear_,
                            setpoint_retry_rear_, setpoint_feedback_rear_,
                            current_steer_pulse_rear_);

        if (!all_motors_operational()) {
            stop_drive_motors();
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "Diff drive inhibited: motor feedback stale or not operational");
            return;
        }

        const int32_t tolerance = deg_to_steer_pulse(3.0);
        const bool steering_aligned =
            std::llabs(static_cast<long long>(target_front) - current_steer_pulse_front_) <=
                tolerance &&
            std::llabs(static_cast<long long>(target_rear) - current_steer_pulse_rear_) <=
                tolerance;
        if (!steering_aligned) {
            stop_drive_motors();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Diff drive inhibited: steering left diff position");
            return;
        }
        if (cmd_timeout) {
            stop_drive_motors();
            return;
        }

        // 디프 전용 속도 제한. 0 이면 전역 제한을 쓴다.
        const double lin_lim = diff_max_linear_vel_ > 0.0
                                   ? std::min(diff_max_linear_vel_, max_linear_vel_)
                                   : max_linear_vel_;
        const double ang_lim = diff_max_angular_vel_ > 0.0
                                   ? std::min(diff_max_angular_vel_, max_angular_vel_)
                                   : max_angular_vel_;
        double v_lat = std::clamp((diff_lateral_ ? cmd_vx_ : 0.0) + cmd_vy_, -lin_lim, lin_lim);
        double wz = std::clamp(cmd_wz_, -ang_lim, ang_lim);
        if (std::abs(v_lat) < 0.001)
            v_lat = 0.0;
        if (std::abs(wz) < 0.001)
            wz = 0.0;
        if (v_lat == 0.0 && wz == 0.0) {
            stop_drive_motors();
            return;
        }

        // 실제 조향각(측정값)으로 나눠서, 90 도가 아니어도 명령이 정확하다.
        const double sf = std::sin(current_steer_angle_front_);
        const double sr = std::sin(current_steer_angle_rear_);
        if (std::abs(sf) < 0.1 || std::abs(sr) < 0.1) {
            stop_drive_motors();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Diff steering angle (%.1f/%.1f deg) too close to 0; cannot move",
                                 current_steer_angle_front_ * RAD2DEG,
                                 current_steer_angle_rear_ * RAD2DEG);
            return;
        }
        const double half = wz * WHEELBASE * 0.5;
        const double front_speed =
            std::clamp((v_lat + half) / sf, -max_linear_vel_, max_linear_vel_);
        const double rear_speed =
            std::clamp((v_lat - half) / sr, -max_linear_vel_, max_linear_vel_);

        send_drive_pdo(ID_FRONT_DRIVE, wheel_mps_to_pulse(front_speed), 0x000F);
        send_drive_pdo(ID_REAR_DRIVE, wheel_mps_to_pulse(rear_speed), 0x000F);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Diff cmd lateral=%.3f yaw=%.3f wheel(F/R)=%.3f/%.3f m/s", v_lat,
                             wz, front_speed, rear_speed);
    }

    static int32_t wheel_mps_to_pulse(double mps)
    {
        const double rpm = (mps / WHEEL_CIRCUM) * 60.0 * DRIVE_RATIO;
        return static_cast<int32_t>(std::lround((rpm / 60.0) * ENCODER_PPR));
    }

    // 4WS 역위상 조향 (rbio 제어 루프)
    void control_ackermann_mode(bool cmd_timeout)
    {
        if (cmd_timeout) {
            stop_drive_motors();
            hold_current_steering_position();
            return;
        }

        const double L = WHEELBASE;

        double safe_vx = std::clamp(cmd_vx_, -max_linear_vel_, max_linear_vel_);
        double safe_wz = std::clamp(cmd_wz_, -max_angular_vel_, max_angular_vel_);

        // 1. 노이즈 필터링 (0.001 이하는 0으로)
        if (std::abs(safe_vx) < 0.001)
            safe_vx = 0.0;
        if (std::abs(safe_wz) < 0.001)
            safe_wz = 0.0;

        // 2. 앞/뒤 바퀴가 그려야 할 가상의 X, Y 속도 벡터 계산
        double front_vx = safe_vx;
        double front_vy = (safe_wz * L) / 2.0; // 회전 중심 기준 앞바퀴의 횡방향 속도
        double rear_vx = safe_vx;
        double rear_vy = -(safe_wz * L) / 2.0; // 회전 중심 기준 뒷바퀴의 횡방향 속도

        // 3. 빗변의 길이(hypot) = 실제 바퀴가 굴러가야 할 총 속도 크기 계산
        double front_speed = std::hypot(front_vx, front_vy);
        double rear_speed = std::hypot(rear_vx, rear_vy);

        // 4. 정지 전환 시 진행 중인 조향도 현재 위치에서 멈춘다.
        if (front_speed < 0.001 && rear_speed < 0.001) {
            stop_drive_motors();
            hold_current_steering_position();
            return;
        }
        neutral_steering_hold_latched_ = false;

        if (!all_motors_operational()) {
            stop_drive_motors();
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Drive inhibited: one or more motor states are stale or not enabled");
            return;
        }

        // 5. 조향 각도 (방향) 계산
        double front_rad = std::atan2(front_vy, front_vx);
        double rear_rad = std::atan2(rear_vy, rear_vx);

        // 후진(Reverse) 처리: 바퀴가 90도(PI/2) 이상 뒤로 꺾이려고 하면,
        // 바퀴 각도를 정면으로 돌리고 모터 속도를 마이너스로 뒤집는다.
        if (front_rad > M_PI / 2.0) {
            front_rad -= M_PI;
            front_speed *= -1.0;
        } else if (front_rad < -M_PI / 2.0) {
            front_rad += M_PI;
            front_speed *= -1.0;
        }

        if (rear_rad > M_PI / 2.0) {
            rear_rad -= M_PI;
            rear_speed *= -1.0;
        } else if (rear_rad < -M_PI / 2.0) {
            rear_rad += M_PI;
            rear_speed *= -1.0;
        }

        // 기구 끝단을 누르지 않도록 조향 한계에 여유를 둔다.
        front_rad = std::clamp(front_rad, -max_steering_angle_rad_, max_steering_angle_rad_);
        rear_rad = std::clamp(rear_rad, -max_steering_angle_rad_, max_steering_angle_rad_);

        double front_steer_deg = front_rad * RAD2DEG;
        double rear_steer_deg = rear_rad * RAD2DEG;
        steer_cmd_deg_front_ = front_steer_deg;
        steer_cmd_deg_rear_ = rear_steer_deg;

        double front_error_rad = std::abs(front_rad - current_steer_angle_front_);
        double rear_error_rad = std::abs(rear_rad - current_steer_angle_rear_);

        int32_t pulse_front = deg_to_steer_pulse(front_steer_deg);
        int32_t pulse_rear = deg_to_steer_pulse(rear_steer_deg);

        const double pulses_per_radian = STEER_RATIO * ENCODER_PPR / (2.0 * M_PI);
        const int32_t actual_front_pulse =
            static_cast<int32_t>(std::llround(current_steer_angle_front_ * pulses_per_radian));
        const int32_t actual_rear_pulse =
            static_cast<int32_t>(std::llround(current_steer_angle_rear_ * pulses_per_radian));
        handle_steer_toggle(ID_FRONT_STEER, pulse_front, last_pulse_front_, toggle_front_,
                            setpoint_retry_front_, setpoint_feedback_front_, actual_front_pulse);
        handle_steer_toggle(ID_REAR_STEER, pulse_rear, last_pulse_rear_, toggle_rear_,
                            setpoint_retry_rear_, setpoint_feedback_rear_, actual_rear_pulse);

        const bool straight_command = safe_wz == 0.0;
        const bool steering_aligned = front_error_rad <= DRIVE_STEERING_TOLERANCE_RAD &&
                                      rear_error_rad <= DRIVE_STEERING_TOLERANCE_RAD;

        // 직진/후진은 두 조향축이 0도에 정렬된 뒤에만 출발한다.
        if (straight_command && !steering_aligned) {
            stop_drive_motors();
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Waiting for straight steering alignment: target(F/R)=%.1f/%.1f deg "
                "actual(F/R)=%.1f/%.1f deg",
                front_steer_deg, rear_steer_deg, current_steer_angle_front_ * RAD2DEG,
                current_steer_angle_rear_ * RAD2DEG);
            return;
        }

        // 회전 주행은 조향과 구동을 동시에 수행하되, 각 축의 조향 오차만큼
        // 구동 속도를 줄인다. 오차가 90도 이상이면 해당 축은 정지한다.
        if (!straight_command) {
            const double front_alignment_scale = std::clamp(std::cos(front_error_rad), 0.0, 1.0);
            const double rear_alignment_scale = std::clamp(std::cos(rear_error_rad), 0.0, 1.0);
            front_speed *= front_alignment_scale;
            rear_speed *= rear_alignment_scale;

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Driving while steering: scale(F/R)=%.2f/%.2f "
                                 "target(F/R)=%.1f/%.1f deg actual(F/R)=%.1f/%.1f deg",
                                 front_alignment_scale, rear_alignment_scale, front_steer_deg,
                                 rear_steer_deg, current_steer_angle_front_ * RAD2DEG,
                                 current_steer_angle_rear_ * RAD2DEG);
        }

        // 앞/뒤 구동축은 항상 같은 주기에 함께 갱신한다.
        send_drive_pdo(ID_FRONT_DRIVE, wheel_mps_to_pulse(front_speed), 0x000F);
        send_drive_pdo(ID_REAR_DRIVE, wheel_mps_to_pulse(rear_speed), 0x000F);
    }

    // --- 제어 루프 ---
    void control_loop()
    {
        read_can_messages();
        update_odometry(); // 모드/복구/정지와 무관하게 실제 바퀴 피드백을 계속 적분한다
        publish_steer_angle();

        if (transitioning_ && (steering_fault_active() || drive_fault_active())) {
            fail_mode("Motor fault detected during mode transition");
        }

        if (process_steering_recovery()) {
            return;
        }

        process_mode_request();
        if (process_mode_transition())
            return;
        if (failed_) {
            stop_drive_motors();
            return;
        }

        const bool cmd_timeout = (this->now() - last_cmd_time_).seconds() > cmd_vel_timeout_s_;

        switch (mode_) {
        case Mode::ALIGN:
            control_align_mode();
            break;
        case Mode::DIFF:
            control_diff_mode(cmd_timeout);
            break;
        case Mode::ACKERMANN:
            control_ackermann_mode(cmd_timeout);
            break;
        }
    }

    void handle_steer_toggle(int id, int32_t target_pulse, int32_t& last_pulse, int& state,
                             int& retry_cycles, int32_t& feedback_reference, int32_t actual_pulse)
    {
        const auto pulse_delta = [](int32_t lhs, int32_t rhs) {
            return std::llabs(static_cast<long long>(lhs) - static_cast<long long>(rhs));
        };
        const bool target_changed = pulse_delta(target_pulse, last_pulse) > 100;
        if (target_changed) {
            last_pulse = target_pulse;
            feedback_reference = actual_pulse;
            retry_cycles = 0;
            state = 1;
        }

        switch (state) {
        case 1:
            // 이전 set-point ACK를 내리기 전까지 LOW를 유지한다.
            send_steer_pdo(id, last_pulse, 0x000F);
            if (!setpoint_acknowledged(id))
                state = 2;
            break;
        case 2:
            // ACK가 올라올 때까지 New set-point bit를 HIGH로 유지한다.
            send_steer_pdo(id, last_pulse, 0x003F);
            if (setpoint_acknowledged(id))
                state = 3;
            break;
        case 3:
            // ACK가 다시 내려가야 다음 목표도 확실한 상승 에지를 만들 수 있다.
            send_steer_pdo(id, last_pulse, 0x000F);
            if (!setpoint_acknowledged(id)) {
                state = 0;
                retry_cycles = 0;
                feedback_reference = actual_pulse;
            }
            break;
        default: {
            send_steer_pdo(id, last_pulse, 0x000F);
            const bool target_reached =
                pulse_delta(last_pulse, actual_pulse) <=
                static_cast<long long>(STEER_RATIO * ENCODER_PPR * 3.0 / 360.0);
            if (target_reached) {
                retry_cycles = 0;
                feedback_reference = actual_pulse;
                break;
            }

            if (++retry_cycles >= SETPOINT_RETRY_CYCLES) {
                const bool stalled =
                    pulse_delta(actual_pulse, feedback_reference) < SETPOINT_STALL_PULSES;
                feedback_reference = actual_pulse;
                retry_cycles = 0;
                if (stalled)
                    state = 1;
            }
            break;
        }
        }
    }

    void hold_current_steering_position()
    {
        if (!both_steering_status_fresh() || steering_fault_active())
            return;

        const double pulses_per_radian = STEER_RATIO * ENCODER_PPR / (2.0 * M_PI);
        const int32_t actual_front =
            static_cast<int32_t>(std::llround(current_steer_angle_front_ * pulses_per_radian));
        const int32_t actual_rear =
            static_cast<int32_t>(std::llround(current_steer_angle_rear_ * pulses_per_radian));
        if (!neutral_steering_hold_latched_) {
            neutral_hold_front_pulse_ = actual_front;
            neutral_hold_rear_pulse_ = actual_rear;
            neutral_steering_hold_latched_ = true;
        }

        handle_steer_toggle(ID_FRONT_STEER, neutral_hold_front_pulse_, last_pulse_front_,
                            toggle_front_, setpoint_retry_front_, setpoint_feedback_front_,
                            actual_front);
        handle_steer_toggle(ID_REAR_STEER, neutral_hold_rear_pulse_, last_pulse_rear_, toggle_rear_,
                            setpoint_retry_rear_, setpoint_feedback_rear_, actual_rear);
    }

    // --- 콜백 ---
    void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (pub_command_ack_) {
            std_msgs::msg::String ack;
            ack.data = "cmd_vel_received";
            pub_command_ack_->publish(ack);
        }
        last_cmd_time_ = this->now();

        if (awaiting_neutral_command_) {
            const bool neutral = std::abs(msg->linear.x) < 0.001 &&
                                 std::abs(msg->linear.y) < 0.001 &&
                                 std::abs(msg->angular.z) < 0.001;
            cmd_vx_ = cmd_vy_ = cmd_wz_ = 0.0;
            if (neutral) {
                awaiting_neutral_command_ = false;
                motion_inhibited_ = false;
                RCLCPP_INFO(this->get_logger(),
                            "Neutral cmd_vel received. Motion is enabled again.");
            }
            return;
        }

        cmd_vx_ = msg->linear.x;
        cmd_vy_ = msg->linear.y;
        cmd_wz_ = msg->angular.z;
    }

    void initialize_cb(const std_srvs::srv::Trigger::Request::SharedPtr,
                       std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        if (initialization_in_progress_ || steering_recovery_requested_) {
            response->success = false;
            response->message = "Motor initialization is already running";
            return;
        }
        if (can_sock_ < 0) {
            response->success = false;
            response->message = "CAN socket is unavailable";
            return;
        }
        if (drive_fault_active()) {
            response->success = false;
            response->message = "Drive motor fault must be serviced before initialization";
            return;
        }

        if (recovery_state_ == SteeringRecoveryState::FAILED) {
            recovery_state_ = SteeringRecoveryState::IDLE;
        }
        stop_drive_motors();
        steering_recovery_requested_ = true;
        initialization_complete_ = false;
        motion_inhibited_ = true;
        awaiting_neutral_command_ = false;
        transitioning_ = false;
        failed_ = false;
        mode_ = Mode::ALIGN;
        target_mode_ = Mode::ALIGN;
        set_initialization_status("INITIALIZING", "Lower motor initialization requested", false);
        response->success = true;
        response->message = "Lower motor initialization accepted";
    }

    void reset_odom_cb(const std_srvs::srv::Trigger::Request::SharedPtr,
                       std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        x_ = y_ = th_ = 0.0;
        response->success = true;
        response->message = "Odometry reset to origin";
        RCLCPP_INFO(this->get_logger(), "Odometry reset to origin");
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorNode>());
    rclcpp::shutdown();
    return 0;
}
