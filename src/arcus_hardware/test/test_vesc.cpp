#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>

// Include VESC driver headers
#include "arcus_hardware/vesc_packet.hpp"
#include "arcus_hardware/vesc_driver.hpp"

// Use rclcpp for logging
#include <rclcpp/rclcpp.hpp>

using namespace arcus_hardware;

// Global flag for signal handling
std::atomic<bool> keep_running(true);

void signal_handler(int signal)
{
    if (signal == SIGINT)
    {
        std::cout << "\nReceived Ctrl+C, shutting down..." << std::endl;
        keep_running = false;
    }
}

void print_separator()
{
    std::cout << std::string(60, '=') << std::endl;
}

void print_test_header(const std::string &test_name)
{
    std::cout << "\n";
    print_separator();
    std::cout << test_name << std::endl;
    print_separator();
}

void print_vesc_state(const VESCState &state)
{
    std::cout << "=== VESC State ===" << std::endl;
    std::cout << "  FET Temperature:   " << std::fixed << std::setprecision(1)
              << state.temp_fet << " °C" << std::endl;
    std::cout << "  Motor Temperature: " << state.temp_motor << " °C" << std::endl;
    std::cout << "  Motor Current:     " << std::setprecision(2)
              << state.current_motor << " A" << std::endl;
    std::cout << "  Input Current:     " << state.current_input << " A" << std::endl;
    std::cout << "  Input Voltage:     " << state.voltage_input << " V" << std::endl;
    std::cout << "  Duty Cycle:        " << std::setprecision(3)
              << state.duty_cycle << std::endl;
    std::cout << "  ERPM:              " << state.rpm << std::endl;
    std::cout << "  Position:          " << state.position << " degrees" << std::endl;
    std::cout << "  Fault Code:        " << static_cast<int>(state.fault_code) << std::endl;
}

bool test_packet_framing()
{
    print_test_header("TEST 1: Packet Framing");

    std::vector<uint8_t> payload = {0x04, 0x01, 0x02, 0x03};
    auto packet = VESCPacket::frame(payload);

    std::cout << "Testing short packet..." << std::endl;
    std::cout << "  Payload size: " << payload.size() << " bytes" << std::endl;
    std::cout << "  Packet size:  " << packet.size() << " bytes" << std::endl;

    if (packet[0] != 0x02 || packet[1] != payload.size() || packet.back() != 0x03)
    {
        std::cout << "✗ Packet structure incorrect" << std::endl;
        return false;
    }

    std::cout << "✓ Packet structure correct" << std::endl;

    VESCPacket::Parser parser;
    std::vector<uint8_t> received_payload;
    bool packet_complete = false;

    for (uint8_t byte : packet)
    {
        if (parser.process_byte(byte, received_payload))
        {
            packet_complete = true;
            break;
        }
    }

    if (!packet_complete || received_payload != payload)
    {
        std::cout << "✗ Parser failed" << std::endl;
        return false;
    }

    std::cout << "✓ Parser correctly decoded packet" << std::endl;
    std::cout << "✓ Test passed!" << std::endl;

    return true;
}

bool test_connection(const std::string &port, int baud_rate, rclcpp::Logger logger)
{
    print_test_header("TEST 2: VESC Connection");

    auto vesc = std::make_unique<VESCDriver>(port, baud_rate, logger);

    std::cout << "Attempting to connect to VESC..." << std::endl;
    std::cout << "  Port: " << port << std::endl;
    std::cout << "  Baud: " << baud_rate << std::endl;

    if (!vesc->connect())
    {
        std::cout << "✗ Failed to connect" << std::endl;
        return false;
    }

    std::cout << "✓ Successfully connected!" << std::endl;

    vesc->disconnect();
    return true;
}

bool test_state_reception(const std::string &port, int baud_rate, rclcpp::Logger logger)
{
    print_test_header("TEST 3: State Reception");

    auto vesc = std::make_unique<VESCDriver>(port, baud_rate, logger);

    if (!vesc->connect())
    {
        std::cout << "✗ Failed to connect" << std::endl;
        return false;
    }

    std::cout << "Requesting VESC state..." << std::endl;

    bool state_received = false;
    VESCState latest_state;

    vesc->set_state_callback([&](const VESCState &state)
                             {
        latest_state = state;
        state_received = true; });

    vesc->request_state();

    auto start_time = std::chrono::steady_clock::now();
    while (!state_received)
    {
        vesc->poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed > std::chrono::seconds(5))
        {
            std::cout << "✗ Timeout waiting for state" << std::endl;
            vesc->disconnect();
            return false;
        }
    }

    std::cout << "✓ State received!" << std::endl;
    print_vesc_state(latest_state);

    vesc->disconnect();
    return true;
}

bool test_continuous_state(const std::string &port, int baud_rate, int duration_sec, rclcpp::Logger logger)
{
    print_test_header("TEST 4: Continuous State Reception");

    auto vesc = std::make_unique<VESCDriver>(port, baud_rate, logger);

    if (!vesc->connect())
    {
        std::cout << "✗ Failed to connect" << std::endl;
        return false;
    }

    std::cout << "Monitoring state for " << duration_sec << " seconds..." << std::endl;

    int state_count = 0;
    VESCState latest_state;

    vesc->set_state_callback([&](const VESCState &state)
                             {
        latest_state = state;
        state_count++; });

    auto start_time = std::chrono::steady_clock::now();
    auto last_request = start_time;

    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(duration_sec))
    {
        auto now = std::chrono::steady_clock::now();
        if (now - last_request > std::chrono::milliseconds(20))
        {
            vesc->request_state();
            last_request = now;
        }

        vesc->poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    double frequency = static_cast<double>(state_count) / duration_sec;
    std::cout << "Received " << state_count << " state messages" << std::endl;
    std::cout << "Update frequency: " << std::fixed << std::setprecision(1)
              << frequency << " Hz" << std::endl;

    if (frequency > 40)
    {
        std::cout << "✓ Good update frequency" << std::endl;
    }
    else
    {
        std::cout << "⚠ Low update frequency" << std::endl;
    }

    std::cout << "\nLatest state:" << std::endl;
    print_vesc_state(latest_state);

    vesc->disconnect();
    return true;
}

bool test_motor_control(const std::string &port, int baud_rate, bool skip_motor, rclcpp::Logger logger)
{
    if (skip_motor)
    {
        std::cout << "\nSkipping motor control tests (--skip-motor flag set)" << std::endl;
        return true;
    }

    print_test_header("TEST 5: Motor Control");
    std::cout << "⚠️  WARNING: Motor will spin!" << std::endl;
    std::cout << "Press Enter to continue or Ctrl+C to abort..." << std::endl;
    std::cin.get();

    auto vesc = std::make_unique<VESCDriver>(port, baud_rate, logger);

    if (!vesc->connect())
    {
        std::cout << "✗ Failed to connect" << std::endl;
        return false;
    }

    VESCState latest_state;
    vesc->set_state_callback([&](const VESCState &state)
                             { latest_state = state; });

    std::cout << "\nTesting current control (12A)..." << std::endl;
    vesc->set_servo_position(50.0);

    for (int i = 0; i < 100; i++)
    {
        vesc->poll();
        vesc->request_state();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::cout << "State after 2 seconds:" << std::endl;
    print_vesc_state(latest_state);

    std::cout << "\nStopping motor..." << std::endl;
    vesc->set_servo_position(0.0);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "✓ Motor control test complete" << std::endl;

    vesc->disconnect();
    return true;
}

void print_usage(const char *program_name)
{
    std::cout << "Usage: " << program_name << " [OPTIONS]" << std::endl;
    std::cout << "\nOptions:" << std::endl;
    std::cout << "  --port <port>       Serial port (default: /dev/ttyACM0)" << std::endl;
    std::cout << "  --baud <rate>       Baud rate (default: 115200)" << std::endl;
    std::cout << "  --skip-motor        Skip tests that spin the motor" << std::endl;
    std::cout << "  --test <name>       Run specific test: packet, connection, state, continuous, motor, all" << std::endl;
    std::cout << "  --help              Show this help message" << std::endl;
}

int main(int argc, char **argv)
{
    // Initialize rclcpp
    rclcpp::init(argc, argv);

    // Parse command line arguments
    std::string port = "/dev/ttyACM0";
    int baud_rate = 115200;
    bool skip_motor = false;
    std::string test_name = "all";

    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];

        if (arg == "--port" && i + 1 < argc)
        {
            port = argv[++i];
        }
        else if (arg == "--baud" && i + 1 < argc)
        {
            baud_rate = std::stoi(argv[++i]);
        }
        else if (arg == "--skip-motor")
        {
            skip_motor = true;
        }
        else if (arg == "--test" && i + 1 < argc)
        {
            test_name = argv[++i];
        }
        else if (arg == "--help")
        {
            print_usage(argv[0]);
            rclcpp::shutdown();
            return 0;
        }
    }

    // Setup signal handler
    signal(SIGINT, signal_handler);

    // Create logger
    auto logger = rclcpp::get_logger("vesc_test");

    // Print header
    print_separator();
    std::cout << "VESC Driver Test Suite" << std::endl;
    print_separator();
    std::cout << "Port: " << port << std::endl;
    std::cout << "Baud: " << baud_rate << std::endl;

    bool all_passed = true;

    // Run tests
    if (test_name == "all" || test_name == "packet")
    {
        if (!test_packet_framing())
            all_passed = false;
    }

    if (keep_running && (test_name == "all" || test_name == "connection"))
    {
        if (!test_connection(port, baud_rate, logger))
            all_passed = false;
    }

    if (keep_running && (test_name == "all" || test_name == "state"))
    {
        if (!test_state_reception(port, baud_rate, logger))
            all_passed = false;
    }

    if (keep_running && (test_name == "all" || test_name == "continuous"))
    {
        if (!test_continuous_state(port, baud_rate, 10, logger))
            all_passed = false;
    }

    if (keep_running && (test_name == "all" || test_name == "motor"))
    {
        if (!test_motor_control(port, baud_rate, skip_motor, logger))
            all_passed = false;
    }

    // Print summary
    std::cout << "\n";
    print_separator();
    if (all_passed)
    {
        std::cout << "✓ All tests PASSED!" << std::endl;
    }
    else
    {
        std::cout << "✗ Some tests FAILED" << std::endl;
    }
    print_separator();

    rclcpp::shutdown();
    return all_passed ? 0 : 1;
}