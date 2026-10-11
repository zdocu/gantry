#include <cmath>
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
            device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 3}.enable_multi_turn_angle());
        pitch_right_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 2}.enable_multi_turn_angle());
        yaw_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 1}.enable_multi_turn_angle());

        board_ = std::make_unique<librmcs::board::CBoard>(
            *this, get_parameter("board_serial").as_string());
        board_->start_transmit();

        remote_control_ = std::make_unique<device::RemoteControl>(*this);
        remote_control_->register_dr16(&dr16_);

        pitch_angle_pid_.kp = get_parameter("pitch_angle_kp").as_double();
        pitch_angle_pid_.ki = get_parameter("pitch_angle_ki").as_double();
        pitch_angle_pid_.kd = get_parameter("pitch_angle_kd").as_double();
        pitch_angle_pid_.integral_min = get_parameter("pitch_angle_integral_min").as_double();
        pitch_angle_pid_.integral_max = get_parameter("pitch_angle_integral_max").as_double();
        pitch_angle_pid_.output_min = get_parameter("pitch_angle_output_min").as_double();
        pitch_angle_pid_.output_max = get_parameter("pitch_angle_output_max").as_double();

        pitch_speed_pid_.kp = get_parameter("pitch_speed_kp").as_double();
        pitch_speed_pid_.ki = get_parameter("pitch_speed_ki").as_double();
        pitch_speed_pid_.kd = get_parameter("pitch_speed_kd").as_double();
        pitch_speed_pid_.integral_min = get_parameter("pitch_speed_integral_min").as_double();
        pitch_speed_pid_.integral_max = get_parameter("pitch_speed_integral_max").as_double();
        pitch_speed_pid_.output_min = get_parameter("pitch_speed_output_min").as_double();
        pitch_speed_pid_.output_max = get_parameter("pitch_speed_output_max").as_double();

        pitch_sync_pid_.kp = get_parameter("pitch_sync_kp").as_double();
        pitch_sync_pid_.ki = get_parameter("pitch_sync_ki").as_double();
        pitch_sync_pid_.kd = get_parameter("pitch_sync_kd").as_double();
        pitch_sync_pid_.integral_min = get_parameter("pitch_sync_integral_min").as_double();
        pitch_sync_pid_.integral_max = get_parameter("pitch_sync_integral_max").as_double();
        pitch_sync_pid_.output_min = get_parameter("pitch_sync_output_min").as_double();
        pitch_sync_pid_.output_max = get_parameter("pitch_sync_output_max").as_double();

        yaw_speed_pid_.kp = get_parameter("yaw_kp").as_double();
        yaw_speed_pid_.ki = get_parameter("yaw_ki").as_double();
        yaw_speed_pid_.kd = get_parameter("yaw_kd").as_double();
        yaw_speed_pid_.integral_min = get_parameter("yaw_integral_min").as_double();
        yaw_speed_pid_.integral_max = get_parameter("yaw_integral_max").as_double();
        yaw_speed_pid_.output_min = get_parameter("yaw_output_min").as_double();
        yaw_speed_pid_.output_max = get_parameter("yaw_output_max").as_double();

        yaw_angle_pid_.kp = get_parameter("yaw_angle_kp").as_double();
        yaw_angle_pid_.ki = get_parameter("yaw_angle_ki").as_double();
        yaw_angle_pid_.kd = get_parameter("yaw_angle_kd").as_double();
        yaw_angle_pid_.integral_min = get_parameter("yaw_angle_integral_min").as_double();
        yaw_angle_pid_.integral_max = get_parameter("yaw_angle_integral_max").as_double();
        yaw_angle_pid_.output_min = get_parameter("yaw_angle_output_min").as_double();
        yaw_angle_pid_.output_max = get_parameter("yaw_angle_output_max").as_double();

        deadzone_  = get_parameter("deadzone").as_double();
        max_speed_ = get_parameter("max_speed").as_double();

        max_acceleration_ = get_parameter("max_acceleration").as_double();
        max_angle_error_  = get_parameter("max_angle_error").as_double();
        sync_error_limit_ = get_parameter("sync_error_limit").as_double();

        pitch_target_angle_ = pitch_left_motor_.angle();
        yaw_target_angle_   = yaw_motor_.angle();

        RCLCPP_INFO(
            logger_,
            "Gantry initialized!");
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

        const double pitch_left_angle  = pitch_left_motor_.angle();
        const double pitch_right_angle = pitch_right_motor_.angle();
        const double pitch_left_vel    = pitch_left_motor_.velocity();
        const double yaw_angle         = yaw_motor_.angle();
        const double yaw_vel           = yaw_motor_.velocity();

        double pitch_stick = dr16_.joystick_left().y();
        double yaw_stick   = dr16_.joystick_left().x();

        if (std::abs(pitch_stick) < deadzone_)
            pitch_stick = 0.0;
        if (std::abs(yaw_stick) < deadzone_)
            yaw_stick = 0.0;

        const bool enabled = (dr16_.switch_right() == rmcs_msgs::Switch::UP) && dr16_.valid();

        if (!enabled) {
            pitch_target_angle_    = pitch_left_angle;
            pitch_target_velocity_ = 0.0;
            pitch_velocity_ref_    = 0.0;
            pitch_left_torque_     = 0.0;
            pitch_right_torque_    = 0.0;
            yaw_torque_            = 0.0;
            yaw_target_angle_      = yaw_angle;
            yaw_target_velocity_   = 0.0;
            yaw_velocity_ref_      = 0.0;
            pitch_angle_pid_.reset();
            pitch_speed_pid_.reset();
            pitch_sync_pid_.reset();
            yaw_angle_pid_.reset();
            yaw_speed_pid_.reset();
            log_status(
                pitch_stick, yaw_stick, pitch_left_angle, pitch_right_angle, pitch_left_vel,
                yaw_angle);
            return;
        }

        const double desired_target_velocity = pitch_stick * max_speed_;
        const double max_velocity_step       = max_acceleration_ * dt_;
        double target_velocity_step          = desired_target_velocity - pitch_target_velocity_;
        if (target_velocity_step > max_velocity_step)
            target_velocity_step = max_velocity_step;
        else if (target_velocity_step < -max_velocity_step)
            target_velocity_step = -max_velocity_step;

        pitch_target_velocity_ += target_velocity_step;
        pitch_target_angle_ += pitch_target_velocity_ * dt_;

        const double angle_error = std::clamp(
            pitch_target_angle_ - pitch_left_angle, -max_angle_error_, max_angle_error_);
        pitch_velocity_ref_ = pitch_angle_pid_.update(angle_error);

        pitch_left_torque_ = pitch_speed_pid_.update(pitch_velocity_ref_ - pitch_left_vel);

        const double sync_error =
            std::clamp(pitch_left_angle - pitch_right_angle, -sync_error_limit_, sync_error_limit_);
        const double sync_correction = pitch_sync_pid_.update(sync_error);

        pitch_right_torque_ = pitch_speed_pid_.update(
            pitch_velocity_ref_ - pitch_left_vel + sync_correction);

        const double desired_yaw_target_velocity = yaw_stick * max_speed_;
        double yaw_velocity_step = desired_yaw_target_velocity - yaw_target_velocity_;
        if (yaw_velocity_step > max_velocity_step)
            yaw_velocity_step = max_velocity_step;
        else if (yaw_velocity_step < -max_velocity_step)
            yaw_velocity_step = -max_velocity_step;

        yaw_target_velocity_ += yaw_velocity_step;
        yaw_target_angle_ += yaw_target_velocity_ * dt_;

        const double yaw_angle_error = std::clamp(
            yaw_target_angle_ - yaw_angle, -max_angle_error_, max_angle_error_);
        yaw_velocity_ref_ = yaw_angle_pid_.update(yaw_angle_error);

        yaw_torque_ = yaw_speed_pid_.update(yaw_velocity_ref_ - yaw_vel);

        log_status(
            pitch_stick, yaw_stick, pitch_left_angle, pitch_right_angle, pitch_left_vel,
            yaw_angle);
    }

private:
    void log_status(
        double pitch_stick, double yaw_stick, double pitch_left_angle, double pitch_right_angle,
        double pitch_left_vel, double yaw_angle) {
        static int cnt = 0;
        if (cnt++ % 200 != 0)
            return;

        RCLCPP_INFO(
            logger_,
            "DR16 sw=%d | stick y=%.2f x=%.2f | "
            "pitch ang=%.3f/%.3f tgt=%.3f vel=%.2f vref=%.2f tau=%.3f/%.3f | "
            "yaw ang=%.3f tgt=%.3f vref=%.2f tau=%.3f",
            static_cast<int>(dr16_.switch_right()), pitch_stick, yaw_stick, pitch_left_angle,
            pitch_right_angle, pitch_target_angle_, pitch_left_vel, pitch_velocity_ref_,
            pitch_left_torque_, pitch_right_torque_, yaw_angle, yaw_target_angle_,
            yaw_velocity_ref_, yaw_torque_);
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
            const auto can_id = data.can_id;

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

    controller::pid::PidCalculator pitch_angle_pid_;
    controller::pid::PidCalculator pitch_speed_pid_;
    controller::pid::PidCalculator pitch_sync_pid_;
    controller::pid::PidCalculator yaw_angle_pid_;
    controller::pid::PidCalculator yaw_speed_pid_;

    double deadzone_  = 0.1;
    double max_speed_ = 8.0;

    double max_acceleration_ = 60.0;
    double max_angle_error_  = 1.5;
    double sync_error_limit_ = 0.5;
    double update_rate_      = 1000.0;
    double dt_               = 0.001;

    double pitch_target_angle_    = 0.0;
    double pitch_target_velocity_ = 0.0;
    double pitch_velocity_ref_    = 0.0;

    double pitch_left_torque_  = 0.0;
    double pitch_right_torque_ = 0.0;
    double yaw_target_angle_    = 0.0;
    double yaw_target_velocity_ = 0.0;
    double yaw_velocity_ref_    = 0.0;

    double yaw_torque_         = 0.0;
};

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Gantry, rmcs_executor::Component)
