#ifndef VESC_DRIVER_HPP
#define VESC_DRIVER_HPP

#include "vesc_packet.hpp"
#include <string>
#include <memory>
#include <functional>
#include <boost/asio.hpp>
#include "rclcpp/rclcpp.hpp"

namespace arcus_hardware
{

    // VESC command IDs
    enum class VESCCommand : uint8_t
    {
        COMM_FW_VERSION = 0,
        COMM_GET_VALUES = 4,
        COMM_SET_DUTY = 5,
        COMM_SET_CURRENT = 6,
        COMM_SET_CURRENT_BRAKE = 7,
        COMM_SET_RPM = 8,
        COMM_SET_SERVO_POS = 12,
        COMM_ALIVE = 30,
        COMM_GET_DECODED_PPM = 11,
        COMM_GET_DECODED_ADC = 12,
        COMM_GET_DECODED_CHUK = 13,
        COMM_ROTOR_POSITION = 21
    };

    // Structure to hold VESC state data
    struct VESCState
    {
        double temp_fet;      // FET temperature (°C)
        double temp_motor;    // Motor temperature (°C)
        double current_motor; // Motor current (A)
        double current_input; // Input current (A)
        double voltage_input; // Input voltage (V)
        double duty_cycle;    // Duty cycle (0-1)
        int32_t rpm;          // RPM (ERPM / pole_pairs)
        double position;      // Position (degrees)
        double tachometer;    // Tachometer (total rotations)
        double distance;      // Distance traveled
        uint8_t fault_code;   // Fault code
    };

    class VESCDriver
    {
    public:
        using StateCallback = std::function<void(const VESCState &)>;

        VESCDriver(const std::string &port, int baud_rate,
                   rclcpp::Logger logger);
        ~VESCDriver();

        bool connect();
        void disconnect();
        bool is_connected() const;

        // Command methods
        void set_duty_cycle(double duty);
        void set_current(double current);
        void set_brake_current(double current);
        void set_rpm(int32_t rpm);
        void set_servo_position(double position_deg);
        void request_state();
        void send_alive();

        void set_state_callback(StateCallback callback);
        void poll();

    private:
        void send_packet(const std::vector<uint8_t> &payload);
        void start_read();
        void handle_read(std::size_t bytes_transferred);
        void process_payload(const std::vector<uint8_t> &payload);

        // Helper functions for byte manipulation
        void append_int32(std::vector<uint8_t> &vec, int32_t value);
        int16_t read_int16(const std::vector<uint8_t> &data, size_t offset);
        int32_t read_int32(const std::vector<uint8_t> &data, size_t offset);

        boost::asio::io_context io_context_;
        boost::asio::serial_port serial_port_;
        std::string port_name_;
        int baud_rate_;
        rclcpp::Logger logger_;
        bool is_connected_;

        VESCPacket::Parser parser_;
        std::array<uint8_t, 512> read_buffer_;
        StateCallback state_callback_;
    };

} // namespace arcus_hardware

#endif // VESC_DRIVER_HPP