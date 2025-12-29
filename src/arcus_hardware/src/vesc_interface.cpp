#include "arcus_hardware/vesc_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <cmath>
#include <algorithm>

namespace arcus_hardware
{

    hardware_interface::CallbackReturn VESCInterface::on_init(
        const hardware_interface::HardwareComponentInterfaceParams &params)
    {
        if (hardware_interface::SystemInterface::on_init(params) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        const auto &info = params.hardware_info;

        // ---- Parameters ----
        port_ = info.hardware_parameters.at("port");
        baud_rate_ = std::stoi(info.hardware_parameters.at("baud_rate"));
        pole_pairs_ = std::stoi(info.hardware_parameters.at("pole_pairs"));
        gear_ratio_ = std::stod(info.hardware_parameters.at("gear_ratio"));

        steering_center_deg_ = std::stod(info.hardware_parameters.at("steering_center_deg"));
        steering_deg_per_rad_ = std::stod(info.hardware_parameters.at("steering_deg_per_rad"));
        max_steering_angle_rad_ = std::stod(info.hardware_parameters.at("max_steering_angle_rad"));

        cmd_timeout_s_ = std::stod(info.hardware_parameters.at("cmd_timeout_s"));
        alternate_drive_steer_ =
            (info.hardware_parameters.at("alternate_drive_steer") == "true");
        alive_every_n_ = std::stoi(info.hardware_parameters.at("alive_every_n"));

        // ---- Joint names ----
        drive_joint_name_ = info.hardware_parameters.at("drive_wheel");
        steer_left_joint_name_ = info.hardware_parameters.at("steering_left_wheel");
        steer_right_joint_name_ = info.hardware_parameters.at("steering_right_wheel");

        // ---- Resolve joint indices ----
        drive_joint_idx_ = steer_left_idx_ = steer_right_idx_ = -1;

        for (size_t i = 0; i < info.joints.size(); ++i)
        {
            const auto &name = info.joints[i].name;
            if (name == drive_joint_name_)
                drive_joint_idx_ = static_cast<int>(i);
            else if (name == steer_left_joint_name_)
                steer_left_idx_ = static_cast<int>(i);
            else if (name == steer_right_joint_name_)
                steer_right_idx_ = static_cast<int>(i);
        }

        if (drive_joint_idx_ < 0 || steer_left_idx_ < 0 || steer_right_idx_ < 0)
        {
            RCLCPP_ERROR(logger_, "Failed to resolve joint indices");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(
            logger_,
            "Using joints:\n  Drive: %s\n  Steering L: %s\n  Steering R: %s",
            drive_joint_name_.c_str(),
            steer_left_joint_name_.c_str(),
            steer_right_joint_name_.c_str());

        // ---- Allocate storage ----
        const size_t n = info.joints.size();
        hw_positions_.assign(n, 0.0);
        hw_velocities_.assign(n, 0.0);
        hw_efforts_.assign(n, 0.0);

        cmd_positions_.assign(n, 0.0);
        cmd_velocities_.assign(n, 0.0);
        cmd_efforts_.assign(n, 0.0);

        // ---- Init VESC ----
        vesc_ = std::make_unique<VESCDriver>(port_, baud_rate_, logger_);
        vesc_->set_state_callback(
            std::bind(&VESCInterface::state_callback, this, std::placeholders::_1));

        RCLCPP_INFO(logger_, "VESCInterface initialized on %s @ %d",
                    port_.c_str(), baud_rate_);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VESCInterface::on_configure(
        const rclcpp_lifecycle::State &)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    VESCInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> si;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            si.emplace_back(info_.joints[i].name,
                            hardware_interface::HW_IF_POSITION,
                            &hw_positions_[i]);
            si.emplace_back(info_.joints[i].name,
                            hardware_interface::HW_IF_VELOCITY,
                            &hw_velocities_[i]);
            si.emplace_back(info_.joints[i].name,
                            hardware_interface::HW_IF_EFFORT,
                            &hw_efforts_[i]);
        }
        return si;
    }

    std::vector<hardware_interface::CommandInterface>
    VESCInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> ci;

        ci.emplace_back(info_.joints[drive_joint_idx_].name,
                        hardware_interface::HW_IF_VELOCITY,
                        &cmd_velocities_[drive_joint_idx_]);

        ci.emplace_back(info_.joints[steer_left_idx_].name,
                        hardware_interface::HW_IF_POSITION,
                        &cmd_positions_[steer_left_idx_]);

        ci.emplace_back(info_.joints[steer_right_idx_].name,
                        hardware_interface::HW_IF_POSITION,
                        &cmd_positions_[steer_right_idx_]);

        return ci;
    }

    hardware_interface::CallbackReturn VESCInterface::on_activate(
        const rclcpp_lifecycle::State &)
    {
        if (!vesc_->connect())
        {
            RCLCPP_ERROR(logger_, "Failed to connect to VESC");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // ORCE FULL STOP SEQUENCE
        vesc_->set_current(0.0);       // cancel current mode
        vesc_->set_brake_current(5.0); // active brake
        rclcpp::sleep_for(std::chrono::milliseconds(200));

        vesc_->set_brake_current(0.0); // release brake
        vesc_->set_rpm(0);             // neutral RPM
        vesc_->set_servo_position(steering_center_deg_);

        std::fill(cmd_velocities_.begin(), cmd_velocities_.end(), 0.0);
        std::fill(cmd_positions_.begin(), cmd_positions_.end(), 0.0);
        std::fill(cmd_efforts_.begin(), cmd_efforts_.end(), 0.0);

        last_cmd_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
        alive_ctr_ = 0;
        send_drive_next_ = true;

        RCLCPP_INFO(logger_, "VESCInterface activated (FORCED NEUTRAL)");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VESCInterface::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        if (vesc_ && vesc_->is_connected())
        {
            vesc_->set_rpm(0);
            vesc_->set_servo_position(steering_center_deg_);
            vesc_->disconnect();
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type VESCInterface::read(
        const rclcpp::Time &, const rclcpp::Duration &)
    {
        if (!vesc_ || !vesc_->is_connected())
            return hardware_interface::return_type::ERROR;

        vesc_->poll();
        vesc_->request_state();
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type VESCInterface::write(
        const rclcpp::Time &time,
        const rclcpp::Duration &)
    {
        if (!vesc_ || !vesc_->is_connected())
            return hardware_interface::return_type::ERROR;

        // ---- Keepalive ----
        if (++alive_ctr_ >= alive_every_n_)
        {
            vesc_->send_alive();
            alive_ctr_ = 0;
        }

        // ---- Constants ----
        constexpr double CMD_DEADBAND = 1e-6;

        constexpr int32_t MIN_ERPM = 2500;
        constexpr int32_t MAX_ERPM = 8000; // adjust if needed

        constexpr double BRAKE_CURRENT = 3.0; // A (reduce to 2.0 for drivetrain testing)

        // ---- Read commands ----
        const double drive_cmd = cmd_velocities_[drive_joint_idx_];
        const double steer_l = cmd_positions_[steer_left_idx_];
        const double steer_r = cmd_positions_[steer_right_idx_];

        const bool drive_active = std::abs(drive_cmd) > CMD_DEADBAND;
        const bool steer_active =
            std::abs(steer_l) > CMD_DEADBAND ||
            std::abs(steer_r) > CMD_DEADBAND;

        // ---- HARD IDLE: brake, do not coast ----
        if (!drive_active && !steer_active)
        {
            vesc_->set_brake_current(BRAKE_CURRENT);
            return hardware_interface::return_type::OK;
        }

        last_cmd_time_ = time;

        // ---- Timeout safety ----
        if ((time - last_cmd_time_).seconds() > cmd_timeout_s_)
        {
            vesc_->set_brake_current(BRAKE_CURRENT);
            return hardware_interface::return_type::OK;
        }

        // ---- Steering ----
        if (steer_active)
        {
            const double steer_rad =
                0.5 * (steer_l + steer_r);

            const double clamped_steer =
                clamp(
                    steer_rad,
                    -max_steering_angle_rad_,
                    max_steering_angle_rad_);

            const double servo_deg =
                steering_center_deg_ +
                clamped_steer * steering_deg_per_rad_;

            vesc_->set_servo_position(servo_deg);
        }

        // ---- Drive ----
        if (!drive_active)
        {
            // Zero command → HARD BRAKE
            vesc_->set_brake_current(BRAKE_CURRENT);
        }
        else
        {
            double motor_rad_s = drive_cmd * gear_ratio_;
            double motor_rpm = motor_rad_s * 60.0 / (2.0 * M_PI);
            int32_t erpm = static_cast<int32_t>(motor_rpm * pole_pairs_);

            if (std::abs(erpm) < MIN_ERPM)
                erpm = (erpm > 0) ? MIN_ERPM : -MIN_ERPM;

            RCLCPP_INFO(logger_, "drive_cmd=%.3f → ERPM=%d", drive_cmd, erpm);

            vesc_->set_rpm(erpm);
        }

        return hardware_interface::return_type::OK;
    }

    void VESCInterface::state_callback(const VESCState &state)
    {
        std::lock_guard<std::mutex> lk(state_mtx_);

        const double motor_rpm = static_cast<double>(state.rpm);
        const double motor_rad_s = (motor_rpm * 2.0 * M_PI) / 60.0;
        const double wheel_rad_s = motor_rad_s / gear_ratio_;

        hw_velocities_[drive_joint_idx_] = wheel_rad_s;
        hw_efforts_[drive_joint_idx_] = state.current_motor;

        const double steer_rad = state.position * (M_PI / 180.0);
        hw_positions_[steer_left_idx_] = steer_rad;
        hw_positions_[steer_right_idx_] = steer_rad;
    }

} // namespace arcus_hardware

PLUGINLIB_EXPORT_CLASS(
    arcus_hardware::VESCInterface,
    hardware_interface::SystemInterface)
