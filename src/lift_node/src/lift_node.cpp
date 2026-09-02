// 리프트 노드 (rbio TransferRobot-Hanyang 앱 MotorControlManager 의 리프트 부분을 ROS2 로 옮김)
//
//  - 리프트 제어보드와 RS232 (기본 /dev/ttyTHS1, 115200 8N1) 로 직결한다.
//    앱은 SSH 브리지도 지원했지만 이 PC 가 곧 보드가 붙은 Jetson 이라 직결만 둔다.
//  - 프레임: [0x3E 디바이스ID][프로토콜][페이로드 길이][페이로드][CRC-8 poly 0x07]
//      0x10 수직 리프트      payload 1B  0=정지 1=상승 2=하강
//      0x11 수평 리프트      payload 1B  0=정지 1=전진 2=후진
//      0x20 호이스트         payload 2B  [모터 0..3][제어]  — 여기서는 정지(0)만 낸다 (stop)
//      0x40 로봇암 오류 조회 payload 1B  [모터]  — 링크 점검용. 보드는 0x40 을 그대로
//           되돌리거나 0x41 [모터][오류] 로 답한다 (앱 startup probe 와 같은 판정)
//  - 보드는 마지막 지령을 래치한다 (앱은 버튼 누름에 1/2, 뗌에 0 을 한 번씩만 보냈다).
//    그래서 이 노드는 cmd_timeout_s 안에 같은 지령이 다시 안 오면 정지를 보낸다.
//    같은 값이 반복해서 오면 보드에는 다시 안 보내고 타임아웃만 연장한다.
//  - 정지 프레임은 75/180 ms 뒤 두 번 더 보낸다 (앱과 동일). 그 사이 새 구동 지령이
//    오면 그 축의 재전송은 취소된다.
//  - 포트 열림 직후·재연결·종료 시 전부 정지를 보낸다. 포트 오류는 reconnect_period_s
//    간격으로 다시 연다.
//
// 토픽
//   lift/vertical              std_msgs/Int32  0=정지 1=상승 2=하강   (구독)
//   lift/horizontal            std_msgs/Int32  0=정지 1=전진 2=후진   (구독)
//   lift_node/command_ack      std_msgs/String  "ACCEPTED|…" / "REJECTED|…" / "TIMEOUT|…"
//   lift_node/status           std_msgs/String  latched "STATE|detail"
//                                STATE: DISCONNECTED / CONNECTED(점검 중) / READY / NO_RESPONSE
//   lift_node/diagnostics      diagnostic_msgs/DiagnosticArray  latched, 4 Hz
//   lift_node/rx               std_msgs/String  수신 프레임 hex (디버그)
// 액션
//   lift_node/move             lift_node/action/LiftMove  한 축을 duration 초 동안 구동
//     보드는 위치·리밋을 안 알려주므로 "끝났다" 는 판정은 구동 시간이다. 서버는 goal 이
//     사는 동안 20 ms 마다 지령을 갱신해 두고 (cmd_timeout_s 로 서지 않게) duration 이
//     지나면 정지 → succeed. 취소/선점/토픽 수동 지령/링크 끊김/stop 서비스는 goal 을
//     끝내고 그 축을 세운다. 같은 시각에 사는 goal 은 하나뿐이다 (새 goal 이 앞을 선점).
// 서비스
//   lift_node/stop             std_srvs/Trigger  전부 정지 (리프트 2축 + 호이스트 4)
//   lift_node/reconnect        std_srvs/Trigger  포트 재오픈 (+ 정지 + 점검)
//   lift_node/probe            std_srvs/Trigger  링크 점검 (0x40 왕복). 결과는 status 로

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "lift_node/action/lift_move.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr uint8_t kDeviceId = 0x3E;
constexpr uint8_t kVerticalLiftProtocol = 0x10;
constexpr uint8_t kHorizontalLiftProtocol = 0x11;
constexpr uint8_t kHoistProtocol = 0x20;
constexpr uint8_t kArmErrorReadProtocol = 0x40;
constexpr uint8_t kArmErrorReadResponseProtocol = 0x41;
constexpr uint8_t kArmErrorClearResponseProtocol = 0x51;
constexpr size_t kMaxPayloadLength = 64;
constexpr int kHoistMotorCount = 4;
constexpr std::array<int, 2> kStopResendDelaysMs = {75, 180};

using Bytes = std::vector<uint8_t>;
using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

uint8_t crc8(const uint8_t* data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07)
                               : static_cast<uint8_t>(crc << 1);
    }
    return crc;
}

Bytes make_frame(uint8_t protocol, const Bytes& payload)
{
    Bytes frame;
    frame.reserve(payload.size() + 4);
    frame.push_back(kDeviceId);
    frame.push_back(protocol);
    frame.push_back(static_cast<uint8_t>(payload.size()));
    frame.insert(frame.end(), payload.begin(), payload.end());
    frame.push_back(crc8(frame.data(), frame.size()));
    return frame;
}

std::string to_hex(const uint8_t* data, size_t len)
{
    std::string out;
    out.reserve(len * 3);
    char buf[4]{};
    for (size_t i = 0; i < len; ++i) {
        std::snprintf(buf, sizeof(buf), "%02X", data[i]);
        if (i)
            out += ' ';
        out += buf;
    }
    return out;
}

speed_t baud_constant(int baud)
{
    switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return 0;
    }
}
} // namespace

class LiftNode : public rclcpp::Node
{
  public:
    LiftNode() : Node("lift_node")
    {
        port_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyTHS1");
        baudrate_ = static_cast<int>(this->declare_parameter<int>("baudrate", 115200));
        cmd_timeout_s_ = std::max(0.0, this->declare_parameter<double>("cmd_timeout_s", 0.5));
        probe_on_connect_ = this->declare_parameter<bool>("probe_on_connect", true);
        probe_timeout_s_ =
            std::clamp(this->declare_parameter<double>("probe_timeout_s", 2.5), 0.2, 30.0);
        reconnect_period_s_ =
            std::clamp(this->declare_parameter<double>("reconnect_period_s", 2.0), 0.2, 60.0);
        // 액션 goal 의 duration 이 0 이하일 때 쓰는 방향별 전 구간 구동 시간과 공통 상한.
        // 벤치에서 잰 값이다 (상승: 지령~리밋 정지까지 약 4.2 s). 기계가 바뀌면 여기만 고친다.
        max_move_duration_s_ =
            std::clamp(this->declare_parameter<double>("max_move_duration_s", 30.0), 0.1, 600.0);
        auto travel_param = [this](const char* name, double fallback) {
            return std::clamp(this->declare_parameter<double>(name, fallback), 0.05,
                              max_move_duration_s_);
        };
        default_move_duration_s_[0][0] = travel_param("vertical_up_duration_s", 4.2);
        default_move_duration_s_[0][1] = travel_param("vertical_down_duration_s", 4.2);
        default_move_duration_s_[1][0] = travel_param("horizontal_extend_duration_s", 3.0);
        default_move_duration_s_[1][1] = travel_param("horizontal_retract_duration_s", 3.0);

        sub_vertical_ = this->create_subscription<std_msgs::msg::Int32>(
            "lift/vertical", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) { set_axis(Axis::VERTICAL, msg->data); });
        sub_horizontal_ = this->create_subscription<std_msgs::msg::Int32>(
            "lift/horizontal", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) { set_axis(Axis::HORIZONTAL, msg->data); });

        pub_command_ack_ = this->create_publisher<std_msgs::msg::String>("lift_node/command_ack", 10);
        pub_rx_ = this->create_publisher<std_msgs::msg::String>("lift_node/rx", 10);

        auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1));
        latched_qos.reliable();
        latched_qos.transient_local();
        pub_status_ = this->create_publisher<std_msgs::msg::String>("lift_node/status", latched_qos);
        pub_diagnostics_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "lift_node/diagnostics", latched_qos);

        srv_stop_ = this->create_service<std_srvs::srv::Trigger>(
            "lift_node/stop",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                finish_goal(GoalOutcome::ABORTED, "lift_node/stop", false);
                res->success = send_stop_all(true);
                res->message = res->success ? "stop frames sent (lift x2, hoist x4)"
                                            : "stop requested but link is down; will be sent on reconnect";
            });
        srv_reconnect_ = this->create_service<std_srvs::srv::Trigger>(
            "lift_node/reconnect",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                finish_goal(GoalOutcome::ABORTED, "lift_node/reconnect", true);
                if (connected_)
                    send_stop_all(false);
                res->success = open_port();
                res->message = status_state_ + "|" + status_detail_;
            });
        srv_probe_ = this->create_service<std_srvs::srv::Trigger>(
            "lift_node/probe",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                res->success = start_probe();
                res->message = res->success ? "probe frame sent; watch lift_node/status"
                                            : "not connected";
            });

        action_move_ = rclcpp_action::create_server<LiftMove>(
            this, "lift_node/move",
            std::bind(&LiftNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&LiftNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&LiftNode::handle_accepted, this, std::placeholders::_1));

        const TimePoint now = Clock::now();
        for (auto& ax : axes_)
            ax.last_cmd = now;
        next_reconnect_ = now;

        set_status("DISCONNECTED", "opening " + port_);
        open_port();

        tick_timer_ = this->create_wall_timer(20ms, std::bind(&LiftNode::tick, this));
        diag_timer_ = this->create_wall_timer(250ms, std::bind(&LiftNode::publish_diagnostics, this));
    }

    ~LiftNode() override
    {
        if (fd_ >= 0) {
            send_stop_all(false);
            tcdrain(fd_);
            ::close(fd_);
            fd_ = -1;
        }
    }

  private:
    enum class Axis : int { VERTICAL = 0, HORIZONTAL = 1 };
    enum class GoalOutcome { SUCCEEDED, ABORTED, CANCELED };

    using LiftMove = lift_node::action::LiftMove;
    using GoalHandle = rclcpp_action::ServerGoalHandle<LiftMove>;

    // 살아 있는 액션 goal (동시에 하나). 실제 구동은 tick() 이 돌린다 — 시리얼을 만지는
    // 곳을 타이머 콜백 하나로 묶어 두려는 것이다 (별도 실행 스레드를 두지 않는다).
    struct ActiveGoal {
        std::shared_ptr<GoalHandle> handle;
        Axis axis = Axis::VERTICAL;
        int value = 0;
        double duration = 0.0;
        TimePoint start;
        TimePoint next_feedback;
    };

    struct AxisState {
        int value = 0;           // 보드에 마지막으로 보낸 값 (0/1/2)
        TimePoint last_cmd;      // 토픽으로 마지막 지령을 받은 시각 (타임아웃 기준)
        uint64_t generation = 0; // 지령마다 증가. 정지 재전송이 낡았는지 판정
    };

    // 정지 프레임 재전송 예약. axis_mask 비트가 선 축만, 그 축의 generation 이 그대로일 때만
    struct PendingStop {
        TimePoint due;
        unsigned axis_mask = 0;
        bool hoist = false;
        std::array<uint64_t, 2> generation{};
    };

    static const char* axis_name(Axis axis)
    {
        return axis == Axis::VERTICAL ? "vertical" : "horizontal";
    }
    static uint8_t axis_protocol(Axis axis)
    {
        return axis == Axis::VERTICAL ? kVerticalLiftProtocol : kHorizontalLiftProtocol;
    }
    static const char* value_name(Axis axis, int value)
    {
        if (value == 0)
            return "STOP";
        if (axis == Axis::VERTICAL)
            return value == 1 ? "UP" : "DOWN";
        return value == 1 ? "EXTEND" : "RETRACT";
    }

    // ── 시리얼 ───────────────────────────────────────────────
    bool open_port()
    {
        close_port();
        fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0) {
            link_failed(std::string("open failed: ") + std::strerror(errno));
            return false;
        }

        termios tio{};
        if (tcgetattr(fd_, &tio) != 0) {
            link_failed(std::string("tcgetattr failed: ") + std::strerror(errno));
            return false;
        }
        cfmakeraw(&tio);
        tio.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
        tio.c_cflag |= CS8 | CLOCAL | CREAD;
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;
        const speed_t speed = baud_constant(baudrate_);
        if (speed == 0) {
            link_failed("unsupported baudrate " + std::to_string(baudrate_));
            return false;
        }
        cfsetispeed(&tio, speed);
        cfsetospeed(&tio, speed);
        if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
            link_failed(std::string("tcsetattr failed: ") + std::strerror(errno));
            return false;
        }
        tcflush(fd_, TCIOFLUSH);
        rx_buffer_.clear();
        connected_ = true;
        probe_passed_ = false;
        RCLCPP_INFO(this->get_logger(), "serial open %s @ %d 8N1", port_.c_str(), baudrate_);
        set_status("CONNECTED", port_ + " open, stop sent");

        // 재연결 직후에는 보드가 무엇을 래치하고 있을지 모르니 먼저 전부 정지
        send_stop_all(true);
        if (probe_on_connect_)
            start_probe();
        else
            set_status("READY", "probe skipped");
        return connected_;
    }

    void close_port()
    {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
        connected_ = false;
        probe_running_ = false;
    }

    void link_failed(const std::string& detail)
    {
        const bool was_connected = connected_;
        close_port();
        pending_stops_.clear();
        for (auto& ax : axes_)
            ax.value = 0; // 보드 쪽 상태는 모른다. 재연결 시 stop 을 보낸다
        next_reconnect_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                             std::chrono::duration<double>(reconnect_period_s_));
        finish_goal(GoalOutcome::ABORTED, "serial link lost: " + detail, false);
        set_status("DISCONNECTED", detail);
        if (was_connected)
            RCLCPP_ERROR(this->get_logger(), "serial link lost: %s", detail.c_str());
        else
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                 "serial %s: %s (retrying every %.1fs)", port_.c_str(),
                                 detail.c_str(), reconnect_period_s_);
    }

    bool write_bytes(const Bytes& bytes)
    {
        if (fd_ < 0)
            return false;
        size_t offset = 0;
        int spins = 0;
        while (offset < bytes.size()) {
            const ssize_t n = ::write(fd_, bytes.data() + offset, bytes.size() - offset);
            if (n >= 0) {
                offset += static_cast<size_t>(n);
                continue;
            }
            if (errno == EAGAIN || errno == EINTR) {
                if (++spins > 100) { // 100 ms 동안 못 쓰면 링크 이상
                    link_failed("write stalled (tx buffer full)");
                    return false;
                }
                ::usleep(1000);
                continue;
            }
            link_failed(std::string("write failed: ") + std::strerror(errno));
            return false;
        }
        tx_count_ += 1;
        return true;
    }

    bool send_frames(const std::vector<Bytes>& frames)
    {
        if (!connected_)
            return false;
        Bytes out;
        for (const Bytes& f : frames) {
            out.insert(out.end(), f.begin(), f.end());
            RCLCPP_INFO(this->get_logger(), "TX %s", to_hex(f.data(), f.size()).c_str());
        }
        return write_bytes(out);
    }

    // ── 지령 ─────────────────────────────────────────────────
    void set_axis(Axis axis, int value)
    {
        AxisState& ax = axes_[static_cast<int>(axis)];
        const char* name = axis_name(axis);
        // 액션이 돌고 있는 축에 토픽/서비스로 수동 지령이 들어오면 사람 쪽을 우선한다.
        // 새 지령이 그 축을 곧 덮어쓰므로 여기서 따로 정지는 안 보낸다.
        if (!action_driving_ && active_ && active_->axis == axis)
            finish_goal(GoalOutcome::ABORTED, std::string("preempted by lift/") + name + " command",
                        false);
        if (value < 0 || value > 2) {
            RCLCPP_WARN(this->get_logger(), "%s lift value must be 0..2 (got %d)", name, value);
            publish_ack(std::string("REJECTED|") + name + " " + std::to_string(value) + ": must be 0..2");
            return;
        }
        ax.last_cmd = Clock::now();
        if (!connected_) {
            if (value != 0)
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "%s %s ignored: serial link is down", name,
                                     value_name(axis, value));
            publish_ack(std::string("REJECTED|") + name + " " + value_name(axis, value) +
                        ": not connected");
            return;
        }
        if (value == ax.value)
            return; // 같은 지령 반복: 보드에는 다시 안 보내고 타임아웃만 연장
        if (send_axis(axis, value))
            publish_ack(std::string("ACCEPTED|") + name + " " + value_name(axis, value));
        else
            publish_ack(std::string("REJECTED|") + name + " " + value_name(axis, value) +
                        ": write failed");
    }

    bool send_axis(Axis axis, int value)
    {
        AxisState& ax = axes_[static_cast<int>(axis)];
        ax.generation += 1;
        const bool sent = send_frames({make_frame(axis_protocol(axis), {static_cast<uint8_t>(value)})});
        if (sent)
            ax.value = value;
        if (value == 0)
            schedule_stop_resend(1u << static_cast<int>(axis), false);
        return sent;
    }

    std::vector<Bytes> build_stop_frames(unsigned axis_mask, bool hoist) const
    {
        std::vector<Bytes> frames;
        if (axis_mask & 1u)
            frames.push_back(make_frame(kVerticalLiftProtocol, {0}));
        if (axis_mask & 2u)
            frames.push_back(make_frame(kHorizontalLiftProtocol, {0}));
        if (hoist) {
            for (int motor = 0; motor < kHoistMotorCount; ++motor)
                frames.push_back(make_frame(kHoistProtocol, {static_cast<uint8_t>(motor), 0}));
        }
        return frames;
    }

    // 리프트 2축 + 호이스트 4개 정지 (앱 stopAll 과 같은 프레임 묶음)
    bool send_stop_all(bool schedule_resend)
    {
        for (auto& ax : axes_)
            ax.generation += 1;
        const bool sent = send_frames(build_stop_frames(3u, true));
        if (sent) {
            for (auto& ax : axes_)
                ax.value = 0;
        }
        if (schedule_resend)
            schedule_stop_resend(3u, true);
        return sent;
    }

    void schedule_stop_resend(unsigned axis_mask, bool hoist)
    {
        if (!connected_)
            return;
        const TimePoint now = Clock::now();
        for (const int delay_ms : kStopResendDelaysMs) {
            PendingStop p;
            p.due = now + std::chrono::milliseconds(delay_ms);
            p.axis_mask = axis_mask;
            p.hoist = hoist;
            p.generation = {axes_[0].generation, axes_[1].generation};
            pending_stops_.push_back(p);
        }
    }

    void flush_pending_stops(const TimePoint& now)
    {
        for (size_t i = 0; i < pending_stops_.size();) {
            PendingStop& p = pending_stops_[i];
            if (p.due > now) {
                ++i;
                continue;
            }
            unsigned mask = 0;
            for (int a = 0; a < 2; ++a) {
                if ((p.axis_mask & (1u << a)) && axes_[a].generation == p.generation[a])
                    mask |= 1u << a; // 그 사이 새 지령이 없었던 축만 다시 정지
            }
            const auto frames = build_stop_frames(mask, p.hoist);
            pending_stops_.erase(pending_stops_.begin() + static_cast<long>(i));
            if (!frames.empty() && !send_frames(frames))
                return; // link_failed 가 pending_stops_ 를 비웠다
        }
    }

    // ── 액션 lift_node/move ──────────────────────────────────
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                            std::shared_ptr<const LiftMove::Goal> goal)
    {
        if (goal->axis > 1) {
            RCLCPP_WARN(this->get_logger(), "move rejected: axis must be 0 or 1 (got %u)",
                        static_cast<unsigned>(goal->axis));
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (goal->direction != 1 && goal->direction != 2) {
            RCLCPP_WARN(this->get_logger(), "move rejected: direction must be 1 or 2 (got %u)",
                        static_cast<unsigned>(goal->direction));
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (!connected_) {
            RCLCPP_WARN(this->get_logger(), "move rejected: serial link is down");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
    {
        // 실제 정지와 결과 채우기는 tick() 의 service_active_goal 에서 한다.
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> handle)
    {
        const auto goal = handle->get_goal();
        const Axis axis = static_cast<Axis>(goal->axis);
        const int index = static_cast<int>(axis);
        double duration = static_cast<double>(goal->duration);
        if (duration <= 0.0)
            duration = default_move_duration_s_[index][goal->direction - 1];
        duration = std::clamp(duration, 0.05, max_move_duration_s_);

        // 새 goal 이 앞의 goal 을 선점한다. 다른 축이었다면 그 축은 여기서 세운다.
        if (active_) {
            const bool other_axis = active_->axis != axis;
            finish_goal(GoalOutcome::ABORTED, "preempted by a new goal", other_axis);
        }

        auto next = std::make_shared<ActiveGoal>();
        next->handle = handle;
        next->axis = axis;
        next->value = goal->direction;
        next->duration = duration;
        next->start = Clock::now();
        next->next_feedback = next->start;
        active_ = next;

        RCLCPP_INFO(this->get_logger(), "move %s %s for %.2fs", axis_name(axis),
                    value_name(axis, next->value), duration);
        drive_active_goal(); // 첫 프레임은 tick 을 기다리지 않고 바로 보낸다
    }

    // 액션이 내는 지령. action_driving_ 플래그로 set_axis 의 선점 판정을 비껴간다.
    void drive_active_goal()
    {
        if (!active_)
            return;
        const auto goal = active_;
        action_driving_ = true;
        set_axis(goal->axis, goal->value);
        action_driving_ = false;
    }

    // 살아 있는 goal 을 끝낸다. stop_axis 면 그 축에 정지 프레임도 보낸다.
    // (수동 지령에 선점된 경우처럼 새 값이 곧 덮어쓰는 자리에서는 보내지 않는다.)
    void finish_goal(GoalOutcome outcome, const std::string& detail, bool stop_axis)
    {
        if (!active_)
            return;
        const auto goal = active_;
        active_.reset(); // 정지 프레임이 다시 여기로 들어오는 것을 막는다

        if (stop_axis && connected_) {
            action_driving_ = true;
            set_axis(goal->axis, 0);
            action_driving_ = false;
        }

        auto result = std::make_shared<LiftMove::Result>();
        result->elapsed = static_cast<float>(
            std::chrono::duration<double>(Clock::now() - goal->start).count());
        const std::string what = std::string(axis_name(goal->axis)) + " " +
                                 value_name(goal->axis, goal->value);
        switch (outcome) {
        case GoalOutcome::SUCCEEDED:
            result->success = true;
            result->message = "DONE|" + what + ": " + detail;
            goal->handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "move done: %s (%.2fs)", what.c_str(),
                        static_cast<double>(result->elapsed));
            break;
        case GoalOutcome::CANCELED:
            result->success = false;
            result->message = "CANCELED|" + what + ": " + detail;
            goal->handle->canceled(result);
            RCLCPP_WARN(this->get_logger(), "move canceled: %s (%s)", what.c_str(), detail.c_str());
            break;
        case GoalOutcome::ABORTED:
            result->success = false;
            result->message = "ABORTED|" + what + ": " + detail;
            goal->handle->abort(result);
            RCLCPP_WARN(this->get_logger(), "move aborted: %s (%s)", what.c_str(), detail.c_str());
            break;
        }
        publish_ack(result->message);
    }

    // tick 마다: 취소 확인 → 시간 다 됐는지 확인 → 지령 갱신 → 피드백
    void service_active_goal(const TimePoint& now)
    {
        if (!active_)
            return;
        const auto goal = active_;
        if (goal->handle->is_canceling()) {
            finish_goal(GoalOutcome::CANCELED, "cancel requested", true);
            return;
        }
        const double elapsed = std::chrono::duration<double>(now - goal->start).count();
        if (elapsed >= goal->duration) {
            char buf[64]{};
            std::snprintf(buf, sizeof(buf), "driven %.2fs", elapsed);
            finish_goal(GoalOutcome::SUCCEEDED, buf, true);
            return;
        }

        // 보드는 cmd_timeout_s 안에 지령이 다시 안 오면 서므로 계속 갱신한다.
        drive_active_goal();
        if (active_ != goal)
            return; // 쓰기 실패로 링크가 끊겨 abort 됐다

        if (now < goal->next_feedback)
            return;
        goal->next_feedback = now + 200ms;
        auto feedback = std::make_shared<LiftMove::Feedback>();
        feedback->elapsed = static_cast<float>(elapsed);
        feedback->remaining = static_cast<float>(goal->duration - elapsed);
        feedback->state = "RUNNING";
        goal->handle->publish_feedback(feedback);
    }

    // ── 링크 점검 (앱 runStartupProbe) ───────────────────────
    bool start_probe()
    {
        if (!connected_)
            return false;
        probe_running_ = true;
        probe_passed_ = false;
        probe_deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                             std::chrono::duration<double>(probe_timeout_s_));
        set_status("CONNECTED", "probe 0x40 sent, waiting for reply");
        if (!send_frames({make_frame(kArmErrorReadProtocol, {0})})) {
            probe_running_ = false;
            return false;
        }
        return true;
    }

    void handle_frame(uint8_t protocol, const Bytes& payload)
    {
        rx_count_ += 1;
        last_rx_ = Clock::now();
        if (protocol == kArmErrorReadProtocol && payload.size() == 1 && payload[0] == 0 &&
            probe_running_) {
            probe_running_ = false;
            probe_passed_ = true;
            set_status("READY", "probe echoed (0x40)");
        }
        if ((protocol == kArmErrorReadResponseProtocol ||
             protocol == kArmErrorClearResponseProtocol) &&
            payload.size() == 2) {
            const int motor = payload[0];
            const int error = payload[1];
            RCLCPP_INFO(this->get_logger(), "arm motor %d error state %d", motor, error);
            if (protocol == kArmErrorReadResponseProtocol && motor == 0 && probe_running_) {
                probe_running_ = false;
                probe_passed_ = true;
                set_status("READY", "probe answered (arm 0 error " + std::to_string(error) + ")");
            }
        }
    }

    void parse_rx()
    {
        while (true) {
            auto start = std::find(rx_buffer_.begin(), rx_buffer_.end(), kDeviceId);
            if (start == rx_buffer_.end()) {
                rx_buffer_.clear();
                return;
            }
            if (start != rx_buffer_.begin())
                rx_buffer_.erase(rx_buffer_.begin(), start);
            if (rx_buffer_.size() < 4)
                return;

            const size_t payload_len = rx_buffer_[2];
            if (payload_len > kMaxPayloadLength) {
                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }
            const size_t frame_len = payload_len + 4;
            if (rx_buffer_.size() < frame_len)
                return;

            if (crc8(rx_buffer_.data(), frame_len - 1) != rx_buffer_[frame_len - 1]) {
                crc_errors_ += 1;
                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }

            const uint8_t protocol = rx_buffer_[1];
            const Bytes payload(rx_buffer_.begin() + 3, rx_buffer_.begin() + 3 + static_cast<long>(payload_len));
            const std::string hex = to_hex(rx_buffer_.data(), frame_len);
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<long>(frame_len));

            RCLCPP_INFO(this->get_logger(), "RX %s", hex.c_str());
            std_msgs::msg::String msg;
            msg.data = hex;
            pub_rx_->publish(msg);
            handle_frame(protocol, payload);
        }
    }

    void read_serial()
    {
        uint8_t buf[256];
        while (fd_ >= 0) {
            const ssize_t n = ::read(fd_, buf, sizeof(buf));
            if (n > 0) {
                rx_buffer_.insert(rx_buffer_.end(), buf, buf + n);
                if (rx_buffer_.size() > 4096)
                    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.end() - 1024);
                continue;
            }
            if (n < 0 && errno != EAGAIN && errno != EINTR)
                link_failed(std::string("read failed: ") + std::strerror(errno));
            break;
        }
        if (!rx_buffer_.empty())
            parse_rx();
    }

    // ── 주기 처리 ────────────────────────────────────────────
    void tick()
    {
        const TimePoint now = Clock::now();

        if (!connected_) {
            if (now >= next_reconnect_)
                open_port();
            return;
        }

        read_serial();
        if (!connected_)
            return;

        flush_pending_stops(now);
        if (!connected_)
            return;

        service_active_goal(now);
        if (!connected_)
            return;

        if (probe_running_ && now >= probe_deadline_) {
            probe_running_ = false;
            probe_passed_ = false;
            char buf[96]{};
            std::snprintf(buf, sizeof(buf), "no reply to probe 0x40 within %.1fs", probe_timeout_s_);
            set_status("NO_RESPONSE", buf);
        }

        if (cmd_timeout_s_ > 0.0) {
            for (int a = 0; a < 2; ++a) {
                AxisState& ax = axes_[a];
                if (ax.value == 0)
                    continue;
                const double age = std::chrono::duration<double>(now - ax.last_cmd).count();
                if (age < cmd_timeout_s_)
                    continue;
                const Axis axis = static_cast<Axis>(a);
                RCLCPP_WARN(this->get_logger(), "%s %s: no command for %.2fs, stopping",
                            axis_name(axis), value_name(axis, ax.value), age);
                send_axis(axis, 0);
                publish_ack(std::string("TIMEOUT|") + axis_name(axis) + " stopped");
                if (!connected_)
                    return;
            }
        }
    }

    // ── 상태 발행 ────────────────────────────────────────────
    void publish_ack(const std::string& text)
    {
        std_msgs::msg::String msg;
        msg.data = text;
        pub_command_ack_->publish(msg);
    }

    void set_status(const std::string& state, const std::string& detail)
    {
        const bool changed = state != status_state_ || detail != status_detail_;
        status_state_ = state;
        status_detail_ = detail;
        if (changed)
            RCLCPP_INFO(this->get_logger(), "status %s|%s", state.c_str(), detail.c_str());
        std_msgs::msg::String msg;
        msg.data = state + "|" + detail;
        pub_status_->publish(msg);
    }

    void publish_diagnostics()
    {
        diagnostic_msgs::msg::DiagnosticArray diagnostics;
        diagnostics.header.stamp = this->now();

        diagnostic_msgs::msg::DiagnosticStatus node;
        node.name = "lift_node/node";
        node.hardware_id = port_;
        const bool moving = axes_[0].value != 0 || axes_[1].value != 0;
        if (!connected_)
            node.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        else if (status_state_ != "READY" || moving)
            node.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        else
            node.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        node.message = status_state_ + ": " + status_detail_;

        diagnostic_msgs::msg::KeyValue kv;
        auto add = [&](const char* key, const std::string& value) {
            kv.key = key;
            kv.value = value;
            node.values.push_back(kv);
        };
        add("state", status_state_);
        add("vertical", value_name(Axis::VERTICAL, axes_[0].value));
        add("horizontal", value_name(Axis::HORIZONTAL, axes_[1].value));
        add("probe_passed", probe_passed_ ? "true" : "false");
        add("action", active_ ? std::string(axis_name(active_->axis)) + " " +
                                    value_name(active_->axis, active_->value)
                              : std::string("idle"));
        add("tx_frames", std::to_string(tx_count_));
        add("rx_frames", std::to_string(rx_count_));
        add("crc_errors", std::to_string(crc_errors_));
        char buf[32]{};
        if (rx_count_ > 0) {
            std::snprintf(buf, sizeof(buf), "%.1f",
                          std::chrono::duration<double>(Clock::now() - last_rx_).count());
            add("last_rx_age_s", buf);
        } else {
            add("last_rx_age_s", "never");
        }
        diagnostics.status.push_back(node);
        pub_diagnostics_->publish(diagnostics);
    }

    // ── 멤버 ─────────────────────────────────────────────────
    std::string port_;
    int baudrate_ = 115200;
    double cmd_timeout_s_ = 0.5;
    bool probe_on_connect_ = true;
    double probe_timeout_s_ = 2.5;
    double reconnect_period_s_ = 2.0;
    double max_move_duration_s_ = 30.0;
    // [축][방향-1] 전 구간 구동 시간 [s]
    std::array<std::array<double, 2>, 2> default_move_duration_s_{{{{4.2, 4.2}}, {{3.0, 3.0}}}};

    int fd_ = -1;
    bool connected_ = false;
    TimePoint next_reconnect_;
    Bytes rx_buffer_;

    std::array<AxisState, 2> axes_{};
    std::vector<PendingStop> pending_stops_;

    std::shared_ptr<ActiveGoal> active_;
    bool action_driving_ = false;

    bool probe_running_ = false;
    bool probe_passed_ = false;
    TimePoint probe_deadline_;

    std::string status_state_;
    std::string status_detail_;
    uint64_t tx_count_ = 0;
    uint64_t rx_count_ = 0;
    uint64_t crc_errors_ = 0;
    TimePoint last_rx_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_vertical_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_horizontal_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_command_ack_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_rx_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
    rclcpp_action::Server<LiftMove>::SharedPtr action_move_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reconnect_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_probe_;
    rclcpp::TimerBase::SharedPtr tick_timer_;
    rclcpp::TimerBase::SharedPtr diag_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LiftNode>());
    rclcpp::shutdown();
    return 0;
}
