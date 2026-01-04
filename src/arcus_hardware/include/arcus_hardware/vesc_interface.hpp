#ifndef VESC_INTERFACE_HPP_
#define VESC_INTERFACE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "arcus_hardware/vesc_driver.hpp"

namespace arcus_hardware
{

    class VESCInterface : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(VESCInterface)

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams &params);

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) override;

    private:
        // ---- helpers ----
        static double clamp(double v, double lo, double hi)
        {
            return std::max(lo, std::min(hi, v));
        }

        void state_callback(const VESCState &state);

        // ---- VESC driver ----
        std::unique_ptr<VESCDriver> vesc_;
        rclcpp::Logger logger_{rclcpp::get_logger("arcus_hardware.VESCInterface")};

        // ---- Configuration ----
        std::string port_{"/dev/ttyACM0"};
        int baud_rate_{115200};

        // Drive
        int pole_pairs_{7};
        double gear_ratio_{1.0}; // motor_rot / wheel_rot
        int min_erpm_{2500};
        int max_erpm_{5000};

        // Steering mapping
        double servo_center_deg_{90.0};             // servo center position (deg)
        double steering_deg_per_rad_{57.295779513}; // 180/pi
        double max_servo_angle_deg_{180};
        double min_servo_angle_deg_{0};

        // Safety / comms
        double cmd_timeout_s_{0.5};
        int alive_every_n_{50};

        // ---- Joint indices ----
        int drive_joint_idx_{-1}; // rear_left_wheel_joint
        int steer_left_idx_{-1};  // front_left_steering_joint
        int steer_right_idx_{-1}; // front_right_steering_joint

        std::string drive_joint_name_;
        std::string steer_left_joint_name_;
        std::string steer_right_joint_name_;

        // ---- Interface storage ----
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;
        std::vector<double> hw_efforts_;

        std::vector<double> cmd_positions_;
        std::vector<double> cmd_velocities_;
        std::vector<double> cmd_efforts_;

        // ---- Feedback state ----
        std::mutex state_mtx_;
        VESCState last_state_{};
        bool have_state_{false};

        // tracking
        rclcpp::Time last_cmd_time_{0, 0, RCL_STEADY_TIME};
        int alive_ctr_{0};
    };

} // namespace arcus_hardware

#endif // VESC_INTERFACE_HPP_
