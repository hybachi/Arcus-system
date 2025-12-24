#ifndef VESC_INTERFACE_HPP
#define VESC_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Include your VESC driver
#include "vesc_driver.hpp"

namespace arcus_hardware
{

    class VESCInterface : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(VESCInterface)

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        void state_callback(const vesc_driver::VESCState &state);

        // VESC interface
        std::unique_ptr<vesc_driver::VESCInterface> vesc_;

        // Configuration
        std::string port_;
        int baud_rate_;
        int pole_pairs_;

        // Steering configuration
        double steering_center_;    // VESC servo position for center (typically 0.5 or 90 degrees)
        double steering_scale_;     // Scaling factor (degrees of servo per degree of steering)
        double max_steering_angle_; // Max steering angle in radians

        // Joint indices
        int steering_joint_idx_ = -1;
        std::vector<size_t> wheel_joint_indices_;

        // Hardware interface data
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;
        std::vector<double> hw_efforts_;
        std::vector<double> hw_commands_velocity_;
        std::vector<double> hw_commands_position_;
        std::vector<double> hw_commands_effort_;
    };

} // namespace vesc_hardware_interface

#endif // VESC_HARDWARE_INTERFACE_HPP