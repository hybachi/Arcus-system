// #include "arcus_hardware/vesc_interface.hpp"
// #include "pluginlib/class_list_macros.hpp"
// #include <cmath>
// #include <algorithm>

// namespace arcus_hardware
// {

//     hardware_interface::CallbackReturn VESCInterface::on_init(
//         const hardware_interface::HardwareInfo &info)
//     {

//         if (hardware_interface::SystemInterface::on_init(info) !=
//             hardware_interface::CallbackReturn::SUCCESS)
//         {
//             return hardware_interface::CallbackReturn::ERROR;
//         }

//         // Get VESC parameters from URDF/config
//         port_ = info_.hardware_parameters["port"];
//         baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
//         pole_pairs_ = std::stoi(info_.hardware_parameters["pole_pairs"]);

//         // Steering configuration
//         steering_center_ = std::stod(info_.hardware_parameters["steering_center_position"]);
//         steering_scale_ = std::stod(info_.hardware_parameters["steering_scale"]);
//         max_steering_angle_ = std::stod(info_.hardware_parameters["max_steering_angle"]);

//         // Validate joint configuration
//         // Expected: 4 wheel joints + 1 steering joint = 5 joints total
//         if (info_.joints.size() != 5)
//         {
//             RCLCPP_ERROR(rclcpp::get_logger("VESCInterface"),
//                          "Expected 5 joints (4 wheels + 1 steering), got %zu",
//                          info_.joints.size());
//             return hardware_interface::CallbackReturn::ERROR;
//         }

//         // Find joint indices by name
//         for (size_t i = 0; i < info_.joints.size(); i++)
//         {
//             const std::string &joint_name = info_.joints[i].name;

//             if (joint_name.find("steering") != std::string::npos)
//             {
//                 steering_joint_idx_ = i;
//                 RCLCPP_INFO(rclcpp::get_logger("VESCInterface"),
//                             "Steering joint: %s (index %zu)", joint_name.c_str(), i);
//             }
//             else if (joint_name.find("wheel") != std::string::npos)
//             {
//                 wheel_joint_indices_.push_back(i);
//                 RCLCPP_INFO(rclcpp::get_logger("VESCInterface"),
//                             "Wheel joint: %s (index %zu)", joint_name.c_str(), i);
//             }
//         }

//         if (steering_joint_idx_ == -1 || wheel_joint_indices_.size() != 4)
//         {
//             RCLCPP_ERROR(rclcpp::get_logger("VESCInterface"),
//                          "Invalid joint configuration. Need 4 wheel joints and 1 steering joint");
//             return hardware_interface::CallbackReturn::ERROR;
//         }

//         // Initialize state and command vectors
//         hw_positions_.resize(info_.joints.size(), 0.0);
//         hw_velocities_.resize(info_.joints.size(), 0.0);
//         hw_efforts_.resize(info_.joints.size(), 0.0);
//         hw_commands_velocity_.resize(info_.joints.size(), 0.0);
//         hw_commands_position_.resize(info_.joints.size(), 0.0);
//         hw_commands_effort_.resize(info_.joints.size(), 0.0);

//         // Initialize single VESC interface (but don't connect yet)
//         vesc_ = std::make_unique<vesc_driver::VESCInterface>(
//             port_, baud_rate_,
//             rclcpp::get_logger("VESCInterface"));

//         // Set up state callback
//         vesc_->set_state_callback(
//             std::bind(&VESCInterface::state_callback, this,
//                       std::placeholders::_1));

//         return hardware_interface::CallbackReturn::SUCCESS;
//     }

//     hardware_interface::CallbackReturn VESCInterface::on_configure(
//         const rclcpp_lifecycle::State & /*previous_state*/)
//     {

//         RCLCPP_INFO(rclcpp::get_logger("VESCInterface"),
//                     "Configuring VESC hardware interface...");

//         return hardware_interface::CallbackReturn::SUCCESS;
//     }

//     std::vector<hardware_interface::StateInterface> VESCInterface::export_state_interfaces()
//     {
//         std::vector<hardware_interface::StateInterface> state_interfaces;

//         for (size_t i = 0; i < info_.joints.size(); i++)
//         {
//             state_interfaces.emplace_back(hardware_interface::StateInterface(
//                 info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
//             state_interfaces.emplace_back(hardware_interface::StateInterface(
//                 info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
//             state_interfaces.emplace_back(hardware_interface::StateInterface(
//                 info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
//         }

//         return state_interfaces;
//     }

//     std::vector<hardware_interface::CommandInterface> VESCInterface::export_command_interfaces()
//     {
//         std::vector<hardware_interface::CommandInterface> command_interfaces;

//         for (size_t i = 0; i < info_.joints.size(); i++)
//         {
//             // All wheel joints get velocity and effort commands
//             if (i != steering_joint_idx_)
//             {
//                 command_interfaces.emplace_back(hardware_interface::CommandInterface(
//                     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
//                     &hw_commands_velocity_[i]));
//                 command_interfaces.emplace_back(hardware_interface::CommandInterface(
//                     info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
//                     &hw_commands_effort_[i]));
//             }
//             // Steering joint gets position command
//             else
//             {
//                 command_interfaces.emplace_back(hardware_interface::CommandInterface(
//                     info_.joints[i].name, hardware_interface::HW_IF_POSITION,
//                     &hw_commands_position_[i]));
//             }
//         }

//         return command_interfaces;
//     }

//     hardware_interface::CallbackReturn VESCInterface::on_activate(
//         const rclcpp_lifecycle::State & /*previous_state*/)
//     {

//         RCLCPP_INFO(rclcpp::get_logger("VESCInterface"),
//                     "Activating VESC hardware interface...");

//         // Connect to VESC
//         if (!vesc_->connect())
//         {
//             RCLCPP_ERROR(rclcpp::get_logger("VESCInterface"),
//                          "Failed to connect to VESC on %s", port_.c_str());
//             return hardware_interface::CallbackReturn::ERROR;
//         }
//         RCLCPP_INFO(rclcpp::get_logger("VESCInterface"),
//                     "Connected to VESC on %s", port_.c_str());

//         // Reset commands
//         for (size_t i = 0; i < hw_commands_velocity_.size(); i++)
//         {
//             hw_commands_velocity_[i] = 0.0;
//             hw_commands_position_[i] = 0.0;
//             hw_commands_effort_[i] = 0.0;
//         }

//         // Center the steering
//         vesc_->set_position(steering_center_);

//         return hardware_interface::CallbackReturn::SUCCESS;
//     }

//     hardware_interface::CallbackReturn VESCInterface::on_deactivate(
//         const rclcpp_lifecycle::State & /*previous_state*/)
//     {

//         RCLCPP_INFO(rclcpp::get_logger("VESCInterface"),
//                     "Deactivating VESC hardware interface...");

//         // Stop drive motor
//         if (vesc_ && vesc_->is_connected())
//         {
//             vesc_->set_duty_cycle(0.0);
//             // Center steering
//             vesc_->set_position(steering_center_);
//         }

//         // Disconnect VESC
//         vesc_->disconnect();

//         return hardware_interface::CallbackReturn::SUCCESS;
//     }

//     hardware_interface::return_type VESCInterface::read(
//         const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
//     {

//         if (!vesc_ || !vesc_->is_connected())
//         {
//             return hardware_interface::return_type::ERROR;
//         }

//         // Poll for incoming data
//         vesc_->poll();

//         // Request state update (callback will update hw_* variables)
//         vesc_->request_state();

//         return hardware_interface::return_type::OK;
//     }

//     hardware_interface::return_type VESCInterface::write(
//         const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
//     {

//         if (!vesc_ || !vesc_->is_connected())
//         {
//             return hardware_interface::return_type::ERROR;
//         }

//         // Send alive packet periodically
//         static int alive_counter = 0;
//         if (++alive_counter >= 50)
//         { // Every 50 cycles (~500ms at 100Hz)
//             vesc_->send_alive();
//             alive_counter = 0;
//         }

//         // Alternate between drive and steering commands
//         // The VESC can only process one command at a time
//         static bool control_drive = true;

//         if (control_drive)
//         {
//             // Process drive commands
//             // Average the commanded velocity from all 4 wheels
//             double avg_wheel_velocity = 0.0;
//             double avg_wheel_effort = 0.0;
//             int velocity_count = 0;
//             int effort_count = 0;

//             for (size_t idx : wheel_joint_indices_)
//             {
//                 if (std::abs(hw_commands_velocity_[idx]) > 0.001)
//                 {
//                     avg_wheel_velocity += hw_commands_velocity_[idx];
//                     velocity_count++;
//                 }
//                 if (std::abs(hw_commands_effort_[idx]) > 0.001)
//                 {
//                     avg_wheel_effort += hw_commands_effort_[idx];
//                     effort_count++;
//                 }
//             }

//             if (velocity_count > 0)
//             {
//                 avg_wheel_velocity /= velocity_count;
//             }
//             if (effort_count > 0)
//             {
//                 avg_wheel_effort /= effort_count;
//             }

//             // Send command to VESC
//             // Priority: effort (current) control over velocity control
//             if (std::abs(avg_wheel_effort) > 0.01)
//             {
//                 vesc_->set_current(avg_wheel_effort);
//             }
//             else if (std::abs(avg_wheel_velocity) > 0.01)
//             {
//                 // Convert velocity (rad/s) to RPM
//                 double rpm = (avg_wheel_velocity * 60.0) / (2.0 * M_PI);

//                 // Convert to ERPM
//                 int32_t erpm = static_cast<int32_t>(rpm * pole_pairs_);

//                 vesc_->set_rpm(erpm);
//             }
//             else
//             {
//                 // No command, stop motor
//                 vesc_->set_current(0.0);
//             }
//         }
//         else
//         {
//             // Process steering command
//             if (steering_joint_idx_ >= 0 && steering_joint_idx_ < hw_commands_position_.size())
//             {
//                 double commanded_angle = hw_commands_position_[steering_joint_idx_];

//                 // Clamp steering angle to limits
//                 commanded_angle = std::clamp(commanded_angle, -max_steering_angle_, max_steering_angle_);

//                 // Convert steering angle (radians) to VESC servo position
//                 // Scale and add center offset
//                 double vesc_position = steering_center_ + (commanded_angle * 180.0 / M_PI * steering_scale_);

//                 vesc_->set_position(vesc_position);
//             }
//         }

//         // Toggle between drive and steering control
//         control_drive = !control_drive;

//         return hardware_interface::return_type::OK;
//     }

//     void VESCInterface::state_callback(const vesc_driver::VESCState &state)
//     {
//         // The VESC returns state for whichever mode it's currently in
//         // We need to update both drive and steering based on the data

//         // Update all 4 wheel states with drive motor data
//         // Convert ERPM to rad/s
//         double rpm = static_cast<double>(state.rpm) / pole_pairs_;
//         double velocity = (rpm * 2.0 * M_PI) / 60.0;

//         for (size_t idx : wheel_joint_indices_)
//         {
//             // Position: integrate velocity (approximate)
//             hw_positions_[idx] = state.position * M_PI / 180.0;

//             // All wheels have same velocity (assuming no slip)
//             hw_velocities_[idx] = velocity;

//             // Current divided among wheels (approximate)
//             hw_efforts_[idx] = state.current_motor / 4.0;
//         }

//         // Update steering joint state
//         // Note: When in servo mode, the VESC position represents servo position
//         // We need to convert this back to steering angle
//         if (steering_joint_idx_ >= 0)
//         {
//             // For now, use the commanded position as actual position
//             // since we're alternating control and can't read both simultaneously
//             // A more sophisticated approach would track the last known steering position
//             hw_positions_[steering_joint_idx_] = hw_commands_position_[steering_joint_idx_];
//             hw_velocities_[steering_joint_idx_] = 0.0;
//             hw_efforts_[steering_joint_idx_] = 0.0;
//         }
//     }

// } // namespace arcus_hardware

// PLUGINLIB_EXPORT_CLASS(
//     arcus_hardware::VESCInterface,
//     hardware_interface::SystemInterface)