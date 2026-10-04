#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <span>

#include <librmcs/board/c_board.hpp>
#include <librmcs/data/datas.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/pid_calculator.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/remote_control.hpp"

namespace rmcs_core::hardware {

class Gantry
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {
public:
    Gantry()
        : Node{get_component_name(),
               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
        , command_component_(
              create_partner_component<Command>(get_component_name() + "_command", *this))
        , pitch_left_motor_(*this, *command_component_, "/pitch_left")
        , pitch_right_motor_(*this, *command_component_, "/pitch_right")
        , yaw_motor_(*this, *command_component_, "/yaw")
        , dr16_{} {

        pitch_left_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 3}
                .enable_multi_turn_angle());
        pitch_right_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 2}
                .enable_multi_turn_angle());
        yaw_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 1}
                .enable_multi_turn_angle());

        board_ = std::make_unique<librmcs::board::CBoard>(
            *this, get_parameter("board_serial").as_string());
        board_->start_transmit();

        remote_control_ = std::make_unique<device::RemoteControl>(*this);
        remote_control_->register_dr16(&dr16_);

        pitch_speed_pid_.kp = get_parameter("pitch_kp").as_double();
        pitch_speed_pid_.ki = get_parameter("pitch_ki").as_double();
        pitch_speed_pid_.kd = get_parameter("pitch_kd").as_double();
        pitch_speed_pid_.output_min = get_parameter("pitch_output_min").as_double();
        pitch_speed_pid_.output_max = get_parameter("pitch_output_max").as_double();

        yaw_speed_pid_.kp = get_parameter("yaw_kp").as_double();
        yaw_speed_pid_.ki = get_parameter("yaw_ki").as_double();
        yaw_speed_pid_.kd = get_parameter("yaw_kd").as_double();
        yaw_speed_pid_.output_min = get_parameter("yaw_output_min").as_double();
        yaw_speed_pid_.output_max = get_parameter("yaw_output_max").as_double();

        deadzone_ = get_parameter("deadzone").as_double();
        max_speed_ = get_parameter("max_speed").as_double();

        RCLCPP_INFO(logger_, "Gantry initialized!");
    }

    ~Gantry() override = default;

    Gantry(const Gantry&) = delete;
    Gantry& operator=(const Gantry&) = delete;
    Gantry(Gantry&&) = delete;
    Gantry& operator=(Gantry&&) = delete;

    void update() override {
        pitch_left_motor_.update_status();
        pitch_right_motor_.update_status();
        yaw_motor_.update_status();

        dr16_.update_status();
        remote_control_->update();

        double pitch_left_vel  = pitch_left_motor_.velocity();
        double pitch_right_vel = pitch_right_motor_.velocity();
        double yaw_vel         = yaw_motor_.velocity();

        double pitch_vel = (pitch_left_vel + pitch_right_vel) / 2.0;

        double pitch_stick = dr16_.joystick_left().y();
        double yaw_stick   = dr16_.joystick_left().x();

        if (std::abs(pitch_stick) < deadzone_) pitch_stick = 0.0;
        if (std::abs(yaw_stick)   < deadzone_) yaw_stick   = 0.0;

        double target_pitch_vel = pitch_stick * max_speed_;
        double target_yaw_vel   = yaw_stick   * max_speed_;

        bool enabled = (dr16_.switch_right() == rmcs_msgs::Switch::UP);
        if (!enabled || !dr16_.valid()) {
            target_pitch_vel = 0.0;
            target_yaw_vel   = 0.0;
            pitch_speed_pid_.reset();
            yaw_speed_pid_.reset();
        }

        double pitch_err = target_pitch_vel - pitch_vel;
        pitch_left_torque_ = pitch_speed_pid_.update(pitch_err);
        pitch_right_torque_ = pitch_left_torque_;

        double yaw_err = target_yaw_vel - yaw_vel;
        yaw_torque_ = yaw_speed_pid_.update(yaw_err);

        static int cnt = 0;
        if (cnt++ % 200 == 0) {
            RCLCPP_INFO(logger_,
                "sw=%d | L: v=%.2f a=%.1f | R: v=%.2f a=%.1f | Y: v=%.2f a=%.1f | tau=%.3f",
                (int)dr16_.switch_right(),
                pitch_left_vel, pitch_left_motor_.angle(),
                pitch_right_vel, pitch_right_motor_.angle(),
                yaw_vel, yaw_motor_.angle(),
                pitch_left_torque_);
        }
    }

    void command_update() {
        auto builder = board_->start_transmit();

        builder.can_transmit(
            Spec::kCans.kCan1,
            {
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                        yaw_motor_.generate_command(yaw_torque_),
                        pitch_right_motor_.generate_command(pitch_right_torque_),
                        pitch_left_motor_.generate_command(pitch_left_torque_),
                        device::CanPacket8::PaddingQuarter{},
                    }
                        .as_bytes(),
            });
    }

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;

        if (can == Spec::kCans.kCan1) {
            auto can_id = data.can_id;
            if (can_id == 0x201) {
                yaw_motor_.store_status(data.can_data);
            } else if (can_id == 0x202) {
                pitch_right_motor_.store_status(data.can_data);
            } else if (can_id == 0x203) {
                pitch_left_motor_.store_status(data.can_data);
            }
        }
    }

    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart == Spec::kUarts.kDbus) {
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
        }
    }

private:
    rclcpp::Logger logger_;
    std::unique_ptr<librmcs::board::CBoard> board_;

    class Command : public rmcs_executor::Component {
    public:
        explicit Command(Gantry& g) : g_(g) {}
        void update() override { g_.command_update(); }
    private:
        Gantry& g_;
    };
    std::shared_ptr<Command> command_component_;

    device::DjiMotor pitch_left_motor_;
    device::DjiMotor pitch_right_motor_;
    device::DjiMotor yaw_motor_;

    device::Dr16 dr16_;
    std::unique_ptr<device::RemoteControl> remote_control_;

    controller::pid::PidCalculator pitch_speed_pid_;
    controller::pid::PidCalculator yaw_speed_pid_;

    double deadzone_  = 0.1;
    double max_speed_ = 8.0;

    double pitch_left_torque_  = 0.0;
    double pitch_right_torque_ = 0.0;
    double yaw_torque_         = 0.0;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Gantry, rmcs_executor::Component)
