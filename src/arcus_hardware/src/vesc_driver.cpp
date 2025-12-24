#include "arcus_hardware/vesc_driver.hpp"

namespace arcus_hardware
{

    VESCDriver::VESCDriver(const std::string &port, int baud_rate,
                           rclcpp::Logger logger)
        : io_context_(),
          serial_port_(io_context_),
          port_name_(port),
          baud_rate_(baud_rate),
          logger_(logger),
          is_connected_(false)
    {
    }

    VESCDriver::~VESCDriver()
    {
        disconnect();
    }

    bool VESCDriver::connect()
    {
        try
        {
            // Open serial port
            serial_port_.open(port_name_);

            // Set baud rate
            serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));

            // 8 data bits (VESC standard)
            serial_port_.set_option(boost::asio::serial_port_base::character_size(8));

            // 1 stop bit
            serial_port_.set_option(boost::asio::serial_port_base::stop_bits(
                boost::asio::serial_port_base::stop_bits::one));

            // No parity bit
            serial_port_.set_option(boost::asio::serial_port_base::parity(
                boost::asio::serial_port_base::parity::none));

            // No flow control
            serial_port_.set_option(boost::asio::serial_port_base::flow_control(
                boost::asio::serial_port_base::flow_control::none));

            is_connected_ = true;

            // start async read
            start_read();

            RCLCPP_INFO(logger_, "Connected to VESC on port %s", port_name_.c_str());
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger_, "Failed to connect to VESC: %s", e.what());
            return false;
        }
    }

    // Close connection
    void VESCDriver::disconnect()
    {
        if (is_connected_)
        {
            serial_port_.close();
            is_connected_ = false;
        }
    }

    // Check connection
    bool VESCDriver::is_connected() const { return is_connected_; }

    // Set motor duty cycle
    void VESCDriver::set_duty_cycle(double duty)
    {
        if (!is_connected_)
            return;

        std::vector<uint8_t> payload;
        payload.push_back(static_cast<uint8_t>(VESCCommand::COMM_SET_DUTY));

        // Convert duty to int32 (duty * 100000)
        int32_t duty_int = static_cast<int32_t>(duty * 100000.0);
        append_int32(payload, duty_int);

        send_packet(payload);
    }

    // Set motor current (I hope)
    void VESCDriver::set_current(double current)
    {
        if (!is_connected_)
            return;

        std::vector<uint8_t> payload;
        payload.push_back(static_cast<uint8_t>(VESCCommand::COMM_SET_CURRENT));

        // Convert current to milliamps
        int32_t current_ma = static_cast<int32_t>(current * 1000.0);
        append_int32(payload, current_ma);

        send_packet(payload);
    }

    // Set brake current ?? TODO: check if redundant
    void VESCDriver::set_brake_current(double current)
    {
        if (!is_connected_)
            return;

        std::vector<uint8_t> payload;
        payload.push_back(static_cast<uint8_t>(VESCCommand::COMM_SET_CURRENT_BRAKE));

        int32_t current_ma = static_cast<int32_t>(current * 1000.0);
        append_int32(payload, current_ma);

        send_packet(payload);
    }

    // Set motor speed in ERPM
    void VESCDriver::set_rpm(int32_t rpm)
    {
        if (!is_connected_)
            return;

        std::vector<uint8_t> payload;
        payload.push_back(static_cast<uint8_t>(VESCCommand::COMM_SET_RPM));
        append_int32(payload, rpm);

        send_packet(payload);
    }

    // Set servo position (I hope)
    void VESCDriver::set_position(double position_deg)
    {
        if (!is_connected_)
            return;

        std::vector<uint8_t> payload;
        payload.push_back(static_cast<uint8_t>(VESCCommand::COMM_SET_POS));

        // Position is sent as int32 (position * 1000000)
        int32_t pos_int = static_cast<int32_t>(position_deg * 1000000.0);
        append_int32(payload, pos_int);

        send_packet(payload);
    }

    // Request current state from VESC
    void VESCDriver::request_state()
    {
        if (!is_connected_)
            return;

        std::vector<uint8_t> payload;
        payload.push_back(static_cast<uint8_t>(VESCCommand::COMM_GET_VALUES));
        send_packet(payload);
    }

    // Send alive packets to prevent timeout
    void VESCDriver::send_alive()
    {
        if (!is_connected_)
            return;

        std::vector<uint8_t> payload;
        payload.push_back(static_cast<uint8_t>(VESCCommand::COMM_ALIVE));
        send_packet(payload);
    }

    // Callback for state updates
    void VESCDriver::set_state_callback(StateCallback callback)
    {
        state_callback_ = callback;
    }

    // Polling pending I/O operations
    void VESCDriver::poll()
    {
        io_context_.poll();
    }

    // Send framed packet
    void VESCDriver::send_packet(const std::vector<uint8_t> &payload)
    {
        auto packet = VESCPacket::frame(payload);

        try
        {
            // send synchronously
            boost::asio::write(serial_port_, boost::asio::buffer(packet));
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger_, "Failed to send packet: %s", e.what());
        }
    }

    // Asynchronous read operation
    void VESCDriver::start_read()
    {
        serial_port_.async_read_some(
            boost::asio::buffer(read_buffer_),
            [this](const boost::system::error_code &ec, std::size_t bytes_transferred)
            {
                if (!ec)
                {
                    handle_read(bytes_transferred);
                    start_read();
                }
                else
                {
                    RCLCPP_ERROR(logger_, "Read error: %s", ec.message().c_str());
                }
            });
    }

    // Handle data received
    void VESCDriver::handle_read(std::size_t bytes_transferred)
    {
        for (std::size_t i = 0; i < bytes_transferred; ++i)
        {
            std::vector<uint8_t> payload;
            if (parser_.process_byte(read_buffer_[i], payload))
            {
                process_payload(payload);
            }
        }
    }

    // Process VESC payload
    void VESCDriver::process_payload(const std::vector<uint8_t> &payload)
    {
        if (payload.empty())
            return;

        VESCCommand cmd = static_cast<VESCCommand>(payload[0]);

        if (cmd == VESCCommand::COMM_GET_VALUES && payload.size() >= 61)
        {
            VESCState state;
            int idx = 1;

            state.temp_fet = static_cast<double>(read_int16(payload, idx)) / 10.0;
            idx += 2;
            state.temp_motor = static_cast<double>(read_int16(payload, idx)) / 10.0;
            idx += 2;
            state.current_motor = static_cast<double>(read_int32(payload, idx)) / 100.0;
            idx += 4;
            state.current_input = static_cast<double>(read_int32(payload, idx)) / 100.0;
            idx += 4;
            idx += 4; // Skip id (not used)
            idx += 4; // Skip iq (not used)
            state.duty_cycle = static_cast<double>(read_int16(payload, idx)) / 1000.0;
            idx += 2;
            state.rpm = read_int32(payload, idx);
            idx += 4;
            state.voltage_input = static_cast<double>(read_int16(payload, idx)) / 10.0;
            idx += 2;
            idx += 4; // Skip amp hours
            idx += 4; // Skip amp hours charged
            idx += 4; // Skip watt hours
            idx += 4; // Skip watt hours charged
            state.tachometer = read_int32(payload, idx);
            idx += 4;
            state.tachometer = read_int32(payload, idx);
            idx += 4;
            state.position = static_cast<double>(read_int32(payload, idx)) / 1000000.0;
            idx += 4;
            state.fault_code = payload[idx];

            if (state_callback_)
            {
                state_callback_(state);
            }
        }
    }

    // Helper functions //

    void VESCDriver::append_int32(std::vector<uint8_t> &vec, int32_t value)
    {
        vec.push_back((value >> 24) & 0xFF);
        vec.push_back((value >> 16) & 0xFF);
        vec.push_back((value >> 8) & 0xFF);
        vec.push_back(value & 0xFF);
    }

    int16_t VESCDriver::read_int16(const std::vector<uint8_t> &data, size_t offset)
    {
        return (static_cast<int16_t>(data[offset]) << 8) | data[offset + 1];
    }

    int32_t VESCDriver::read_int32(const std::vector<uint8_t> &data, size_t offset)
    {
        return (static_cast<int32_t>(data[offset]) << 24) |
               (static_cast<int32_t>(data[offset + 1]) << 16) |
               (static_cast<int32_t>(data[offset + 2]) << 8) |
               static_cast<int32_t>(data[offset + 3]);
    }

} // namespace arcus_driver