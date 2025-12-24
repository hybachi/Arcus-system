#include "arcus_controllers/ackermann_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <cmath>
#include <algorithm>

namespace arcus_controllers
{
    AckermannController::AckermannController() : ControllerInterface() {}

    controller_interface::CallbackReturn AckermannController::on_init()
    {
        try
        {
            auto_declare<std::string>("drive_wheel", "rear_left_wheel_joint");
            auto_declare<std::string>("steering_left_wheel", "front_left_steering_joint");
            auto_declare<std::string>("steering_right_wheel", "front_right_steering_joint");
            auto_declare<bool>("use_dual_steering", false);
            auto_declare<double>("track_width", 0.2);
            auto_declare<double>("wheel_base", 0.4);
            auto_declare<double>("wheel_radius", 0.1);
            auto_declare<double>("max_steering_angle", 0.5);
            auto_declare<double>("max_speed", 2.0);
            auto_declare<double>("cmd_timeout", 0.5);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn AckermannController::on_configure(const rclcpp_lifecycle::State &)
    {
        drive_wheel_ = get_node()->get_parameter("drive_wheel").as_string();
        steering_left_wheel_ = get_node()->get_parameter("steering_left_wheel").as_string();
        steering_right_wheel_ = get_node()->get_parameter("steering_right_wheel").as_string();
        use_dual_steering_ = get_node()->get_parameter("use_dual_steering").as_bool();
        track_width_ = get_node()->get_parameter("track_width").as_double();
        wheel_base_ = get_node()->get_parameter("wheel_base").as_double();
        wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
        max_steering_angle_ = get_node()->get_parameter("max_steering_angle").as_double();
        max_speed_ = get_node()->get_parameter("max_speed").as_double();
        cmd_timeout_ = get_node()->get_parameter("cmd_timeout").as_double();

        RCLCPP_INFO(get_node()->get_logger(), "AckermannController configured with:");
        RCLCPP_INFO(get_node()->get_logger(), "  Drive wheel: %s", drive_wheel_.c_str());
        RCLCPP_INFO(get_node()->get_logger(), "  Left Steering joint: %s", steering_left_wheel_.c_str());
        RCLCPP_INFO(get_node()->get_logger(), "  Right Steering joint: %s", steering_right_wheel_.c_str());
        RCLCPP_INFO(get_node()->get_logger(), "  Using dual steering: %s", use_dual_steering_ ? "true" : "false");

        cmd_subscriber_ = get_node()->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "cmd_ackermann", rclcpp::SystemDefaultsQoS(),
            [this](const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
            {
                this->cmd_callback(msg);
            });

        RCLCPP_INFO(get_node()->get_logger(), "Subscribed to topic: cmd_ackermann");

        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration AckermannController::command_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        conf_names.push_back(drive_wheel_ + "/" + hardware_interface::HW_IF_VELOCITY);
        conf_names.push_back(steering_left_wheel_ + "/" + hardware_interface::HW_IF_POSITION);
        conf_names.push_back(steering_right_wheel_ + "/" + hardware_interface::HW_IF_POSITION);

        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::InterfaceConfiguration AckermannController::state_interface_configuration() const
    {
        // TODO: state interfaces for odometry
        return {controller_interface::interface_configuration_type::NONE};
    }

    controller_interface::CallbackReturn AckermannController::on_activate(const rclcpp_lifecycle::State &)
    {
        drive_cmd_ = std::ref(command_interfaces_[0]);
        steering_left_cmd_ = std::ref(command_interfaces_[1]);
        steering_right_cmd_ = std::ref(command_interfaces_[2]);

        if (!drive_cmd_ || !steering_left_cmd_ || !steering_right_cmd_)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to get all command interfaces");
            return CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_node()->get_logger(), "AckermannController activated");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn AckermannController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        set_cmd(drive_cmd_, 0.0);
        set_cmd(steering_left_cmd_, 0.0);
        set_cmd(steering_right_cmd_, 0.0);

        RCLCPP_INFO(get_node()->get_logger(), "AckermannController deactivated");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type AckermannController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // time_since_last_cmd_ += period;

        // if (new_cmd_received_)
        // {
        //     time_since_last_cmd_ = rclcpp::Duration::from_seconds(0.0);
        //     new_cmd_received_ = false;
        // }

        // if (time_since_last_cmd_.seconds() > cmd_timeout_)
        // {
        //     set_cmd(drive_cmd_, 0.0);
        //     set_cmd(steering_left_cmd_, 0.0);
        //     set_cmd(steering_right_cmd_, 0.0);

        //     return controller_interface::return_type::OK;
        // }

        double speed = std::clamp(static_cast<double>(current_cmd_.speed), -max_speed_, max_speed_);
        double steering_angle = std::clamp(static_cast<double>(current_cmd_.steering_angle), -max_steering_angle_, max_steering_angle_);
        double wheel_velocity = speed / wheel_radius_;

        set_cmd(drive_cmd_, wheel_velocity);

        if (std::abs(steering_angle) < 1e-5)
        { // Driving straight
            set_cmd(steering_left_cmd_, 0.0);
            set_cmd(steering_right_cmd_, 0.0);
        }
        else if (use_dual_steering_)
        {
            double R = wheel_base_ / std::tan(steering_angle);
            double half_track = track_width_ / 2.0;

            double left_steering_angle = std::atan(wheel_base_ / (R - half_track));
            double right_steering_angle = std::atan(wheel_base_ / (R + half_track));

            set_cmd(steering_left_cmd_, left_steering_angle);
            set_cmd(steering_right_cmd_, right_steering_angle);
        }
        else
        {
            set_cmd(steering_left_cmd_, steering_angle);
            set_cmd(steering_right_cmd_, steering_angle);
        }

        return controller_interface::return_type::OK;
    }

    void AckermannController::cmd_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        current_cmd_ = msg->drive;
        new_cmd_received_ = true;

        RCLCPP_DEBUG(get_node()->get_logger(),
                     "Received command: speed=%.2f, steering_angle=%.2f",
                     current_cmd_.speed, current_cmd_.steering_angle);
    }

    void AckermannController::set_cmd(std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> &cmd, double value)
    {
        if (cmd && !cmd->get().set_value(value))
        {
            RCLCPP_WARN(get_node()->get_logger(), "Failed to set command interface");
        }
    }

} // namespace ackerman_robot_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    arcus_controllers::AckermannController,
    controller_interface::ControllerInterface)