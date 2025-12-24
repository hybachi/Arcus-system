#ifndef ARCUS_CONTROLLERS__ACKERMANN_CONTROLLER_HPP_
#define ARCUS_CONTROLLERS__ACKERMANN_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <optional>
#include <functional>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

namespace arcus_controllers 
{
    class AckermannController : public controller_interface::ControllerInterface
    {
        public:
            AckermannController();
            ~AckermannController() = default;

            controller_interface::CallbackReturn on_init() override;
            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;
              
            controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            void cmd_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
            void set_cmd(std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> &cmd, double value);

            std::string drive_wheel_;
            std::string steering_left_wheel_;
            std::string steering_right_wheel_;
            bool use_dual_steering_;
            double track_width_;
            double wheel_base_;
            double wheel_radius_;
            double max_steering_angle_;
            double max_speed_;
            double cmd_timeout_;

            std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> drive_cmd_;
            std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> steering_left_cmd_;
            std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> steering_right_cmd_;

            ackermann_msgs::msg::AckermannDrive current_cmd_;
            
            rclcpp::Duration time_since_last_cmd_{0, 0};
            bool new_cmd_received_ = false;

            rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_subscriber_;
    };
}

#endif //ARCUS_CONTROLLERS__ACKERMANN_CONTROLLER_HPP_