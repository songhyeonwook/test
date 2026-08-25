#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <cstring>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <fcntl.h>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
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

// 로봇 물리 정보
#define WHEEL_CIRCUM    0.47124   // 바퀴 둘레
#define WHEELBASE       1.29      // 축간거리 [m]

// 조향각 제한 파라미터
#define MAX_STEER_LEFT_DEG   45.0   // 좌측 최대 조향각
#define MAX_STEER_RIGHT_DEG  45.0   // 우측 최대 조향각

#define DRIVE_RATIO     1.0
#define STEER_RATIO     10.0

// 최대 속도 제한
#define MAX_LINEAR_VEL  0.5
#define MAX_ANGULAR_VEL 1.0

// 가감속 파라미터
#define DRIVE_ACCEL     100000
#define DRIVE_DECEL     500000
#define STEER_ACCEL     200000
#define STEER_DECEL     1000000   // 조향만 더 크게

// CANopen Objects
#define NMT_START       0x01
#define OD_CONTROL_WORD 0x6040
#define OD_MODES_OF_OP  0x6060
#define OD_TARGET_VEL   0x60FF

class MotorNode : public rclcpp::Node {
public:
    MotorNode() : Node("motor_node"), can_sock_(-1) {
        if (init_can_socket() < 0) {
            RCLCPP_ERROR(this->get_logger(), "CAN Socket Init Failed!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Initializing Motors...");

        // 1. 초기화 (모든 모터를 속도 제어 모드 3으로 통일)
        init_motor_sdo(ID_FRONT_DRIVE, 3);
        init_motor_sdo(ID_REAR_DRIVE,  3);
        init_motor_sdo(ID_FRONT_STEER, 3);
        init_motor_sdo(ID_REAR_STEER,  3);

        // 2. RPDO 매핑
        setup_rpdo_mapping(ID_FRONT_DRIVE);
        setup_rpdo_mapping(ID_REAR_DRIVE);
        setup_rpdo_mapping(ID_FRONT_STEER);
        setup_rpdo_mapping(ID_REAR_STEER);

        // 3. TPDO 매핑
        // 주행: 실제 속도(0x606C), 조향: 실제 위치(0x6064)
        setup_tpdo_mapping(ID_FRONT_DRIVE, 0x606C0020);
        setup_tpdo_mapping(ID_REAR_DRIVE,  0x606C0020);
        setup_tpdo_mapping(ID_FRONT_STEER, 0x60640020);
        setup_tpdo_mapping(ID_REAR_STEER,  0x60640020);

        // 4. 모터 시작
        send_nmt_command(NMT_START, ID_FRONT_DRIVE);
        send_nmt_command(NMT_START, ID_REAR_DRIVE);
        send_nmt_command(NMT_START, ID_FRONT_STEER);
        send_nmt_command(NMT_START, ID_REAR_STEER);

        // ROS 설정
        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&MotorNode::cmd_vel_cb, this, std::placeholders::_1));

        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        timer_ = this->create_wall_timer(20ms, std::bind(&MotorNode::control_loop, this));

        last_cmd_time_ = this->now();
        last_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Motor Driver Ready! (All Mode 3)");
    }

    ~MotorNode() override {
        if (can_sock_ >= 0) {
            send_pdo_command(ID_FRONT_DRIVE, 0, 0x000F);
            send_pdo_command(ID_REAR_DRIVE,  0, 0x000F);
            send_pdo_command(ID_FRONT_STEER, 0, 0x000F);
            send_pdo_command(ID_REAR_STEER,  0, 0x000F);
            usleep(20000);
            send_nmt_command(0x81, 0);
            close(can_sock_);
        }
    }

private:
    int can_sock_;
    double cmd_vx_ = 0.0;
    double cmd_wz_ = 0.0;

    double x_ = 0.0, y_ = 0.0, th_ = 0.0;
    double current_drive_vel_ = 0.0;
    double current_steer_angle_ = 0.0;

    rclcpp::Time last_cmd_time_;
    rclcpp::Time last_time_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    int init_can_socket() {
        struct sockaddr_can addr{};
        struct ifreq ifr{};

        can_sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_sock_ < 0) {
            return -1;
        }

        std::strncpy(ifr.ifr_name, CAN_INTERFACE, IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';

        if (ioctl(can_sock_, SIOCGIFINDEX, &ifr) < 0) {
            close(can_sock_);
            can_sock_ = -1;
            return -1;
        }

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_sock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
            close(can_sock_);
            can_sock_ = -1;
            return -1;
        }

        fcntl(can_sock_, F_SETFL, O_NONBLOCK);
        return 0;
    }

    void send_sdo_write(int id, uint16_t index, uint8_t sub, int data, uint8_t size) {
        struct can_frame frame{};
        frame.can_id = 0x600 + id;
        frame.can_dlc = 8;
        frame.data[0] = (size == 1) ? 0x2F : (size == 2 ? 0x2B : 0x23);
        frame.data[1] = index & 0xFF;
        frame.data[2] = (index >> 8) & 0xFF;
        frame.data[3] = sub;
        std::memcpy(&frame.data[4], &data, 4);
        write(can_sock_, &frame, sizeof(frame));
        usleep(2000);
    }

    void send_nmt_command(uint8_t cmd, uint8_t id) {
        struct can_frame frame{};
        frame.can_id = 0x000;
        frame.can_dlc = 2;
        frame.data[0] = cmd;
        frame.data[1] = id;
        write(can_sock_, &frame, sizeof(frame));
        usleep(2000);
    }

    void setup_rpdo_mapping(int id) {
        send_sdo_write(id, 0x1400, 0x01, 0x80000200 + id, 4);
        send_sdo_write(id, 0x1600, 0x00, 0, 1);
        send_sdo_write(id, 0x1600, 0x01, 0x60400010, 4);
        send_sdo_write(id, 0x1600, 0x02, 0x60FF0020, 4);
        send_sdo_write(id, 0x1600, 0x00, 2, 1);
        send_sdo_write(id, 0x1400, 0x01, 0x00000200 + id, 4);
    }

    void setup_tpdo_mapping(int id, uint32_t mapped_obj) {
        send_sdo_write(id, 0x1800, 0x01, 0x80000180 + id, 4);
        send_sdo_write(id, 0x1A00, 0x00, 0, 1);
        send_sdo_write(id, 0x1A00, 0x01, 0x60410010, 4);
        send_sdo_write(id, 0x1A00, 0x02, mapped_obj, 4);
        send_sdo_write(id, 0x1A00, 0x00, 2, 1);
        send_sdo_write(id, 0x1800, 0x02, 255, 1);
        send_sdo_write(id, 0x1800, 0x05, 50, 2);
        send_sdo_write(id, 0x1800, 0x01, 0x00000180 + id, 4);
    }

    void send_pdo_command(int id, int32_t val, uint16_t cw) {
        struct can_frame frame{};
        frame.can_id = 0x200 + id;
        frame.can_dlc = 6;
        std::memcpy(&frame.data[0], &cw, 2);
        std::memcpy(&frame.data[2], &val, 4);
        write(can_sock_, &frame, sizeof(frame));
    }

    void init_motor_sdo(int id, int mode) {
        send_sdo_write(id, OD_MODES_OF_OP, 0, mode, 1);

        if (id == ID_FRONT_STEER || id == ID_REAR_STEER) {
            send_sdo_write(id, 0x6083, 0, STEER_ACCEL, 4);
            send_sdo_write(id, 0x6084, 0, STEER_DECEL, 4);
        } else {
            send_sdo_write(id, 0x6083, 0, DRIVE_ACCEL, 4);
            send_sdo_write(id, 0x6084, 0, DRIVE_DECEL, 4);
        }

        send_sdo_write(id, OD_CONTROL_WORD, 0, 0x0006, 2);
        usleep(50000);
        send_sdo_write(id, OD_CONTROL_WORD, 0, 0x0007, 2);
        usleep(50000);
        send_sdo_write(id, OD_CONTROL_WORD, 0, 0x000F, 2);
        usleep(50000);
    }

    void read_can_messages() {
        struct can_frame frame{};
        while (read(can_sock_, &frame, sizeof(frame)) > 0) {
            if (frame.can_id == (0x180 + ID_FRONT_DRIVE)) {
                int32_t raw_vel = 0;
                std::memcpy(&raw_vel, &frame.data[2], 4);
                double rpm = static_cast<double>(raw_vel) * 60.0 / 131072.0;
                current_drive_vel_ = (rpm / 60.0) * WHEEL_CIRCUM / DRIVE_RATIO;
            }
            else if (frame.can_id == (0x180 + ID_FRONT_STEER)) {
                int32_t raw_pos = 0;
                std::memcpy(&raw_pos, &frame.data[2], 4);
                double deg = static_cast<double>(raw_pos) * 360.0 / (STEER_RATIO * 131072.0);
                current_steer_angle_ = deg * M_PI / 180.0;
            }
        }
    }

    void update_odometry() {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        double v = current_drive_vel_;
        double delta = current_steer_angle_;
        double v_x = v * std::cos(delta);
        double omega = (2.0 * v * std::sin(delta)) / WHEELBASE;

        double delta_x = (v_x * std::cos(th_)) * dt;
        double delta_y = (v_x * std::sin(th_)) * dt;
        double delta_th = omega * dt;

        x_ += delta_x;
        y_ += delta_y;
        th_ += delta_th;

        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = current_time;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_link";
        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        odom_tf.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, th_);
        odom_tf.transform.rotation.x = q.x();
        odom_tf.transform.rotation.y = q.y();
        odom_tf.transform.rotation.z = q.z();
        odom_tf.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(odom_tf);

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = odom_tf.transform.rotation;
        odom_msg.twist.twist.linear.x = v_x;
        odom_msg.twist.twist.angular.z = omega;
        pub_odom_->publish(odom_msg);
    }

    void control_loop() {
        read_can_messages();
        update_odometry();

        // 타임아웃 방지
        if ((this->now() - last_cmd_time_).seconds() > 0.5) {
            cmd_vx_ = 0.0;
            cmd_wz_ = 0.0;
        }

        // 입력 속도 제한
        double safe_vx = std::clamp(cmd_vx_, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
        double safe_wz = std::clamp(cmd_wz_, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);

        // 1. 주행 속도 제어
        int32_t vel_pulse_drive = 0;
        if (std::abs(safe_vx) > 0.01) {
            double target_rpm_drive = (safe_vx / WHEEL_CIRCUM) * 60.0 * DRIVE_RATIO;
            vel_pulse_drive = static_cast<int32_t>((target_rpm_drive / 60.0) * 131072.0);
        }

        // 2. 조향 속도 제어
        int32_t vel_pulse_steer = 0;
        if (std::abs(safe_wz) > 0.01) {
            double max_left_rad  = MAX_STEER_LEFT_DEG  * M_PI / 180.0;
            double max_right_rad = MAX_STEER_RIGHT_DEG * M_PI / 180.0;

            if ((safe_wz > 0 && current_steer_angle_ >= max_left_rad) ||
                (safe_wz < 0 && current_steer_angle_ <= -max_right_rad)) {
                vel_pulse_steer = 0;
            } else {
                double target_rpm_steer = (safe_wz / (2.0 * M_PI)) * 60.0 * STEER_RATIO;
                vel_pulse_steer = static_cast<int32_t>((target_rpm_steer / 60.0) * 131072.0);
            }
        }

        // 3. PDO 전송
        send_pdo_command(ID_FRONT_DRIVE, vel_pulse_drive, 0x000F);
        send_pdo_command(ID_REAR_DRIVE,  vel_pulse_drive, 0x000F);

        send_pdo_command(ID_FRONT_STEER, vel_pulse_steer, 0x000F);
        send_pdo_command(ID_REAR_STEER, -vel_pulse_steer, 0x000F);  // 역위상 조향
    }

    void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_vx_ = msg->linear.x;
        cmd_wz_ = msg->angular.z;
        last_cmd_time_ = this->now();
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorNode>());
    rclcpp::shutdown();
    return 0;
}
