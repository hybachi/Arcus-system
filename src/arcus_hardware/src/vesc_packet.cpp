#include "arcus_hardware/vesc_packet.hpp"

namespace arcus_hardware
{
    // Frame a payload into a complete VESC packet
    // Creates packet with: start byte, length, payload, CRC, stop byte
    std::vector<uint8_t> VESCPacket::frame(const std::vector<uint8_t> &payload)
    {
        std::vector<uint8_t> packet;

        // Choose packet format based on payload size
        if (payload.size() <= 256)
        {
            // Short packet: start(1) + length(1) + payload + crc(2) + stop(1)
            packet.push_back(0x02);                                 // Short packet start
            packet.push_back(static_cast<uint8_t>(payload.size())); // Length
        }
        else
        {
            // Long packet: start(1) + length(2) + payload + crc(2) + stop(1)
            packet.push_back(0x03);                                        // Long packet start
            packet.push_back(static_cast<uint8_t>(payload.size() >> 8));   // Length high
            packet.push_back(static_cast<uint8_t>(payload.size() & 0xFF)); // Length low
        }

        // Add payload data
        packet.insert(packet.end(), payload.begin(), payload.end());

        // Add CRC checksum
        uint16_t crc = calculate_crc(payload);
        packet.push_back(static_cast<uint8_t>(crc >> 8));   // CRC high
        packet.push_back(static_cast<uint8_t>(crc & 0xFF)); // CRC low

        // Add stop byte
        packet.push_back(0x03);

        return packet;
    }

    // Calculate CRC16 checksum using VESC polynomial (0x1021)
    uint16_t VESCPacket::calculate_crc(const std::vector<uint8_t> &data)
    {
        uint16_t crc = 0;

        for (uint8_t byte : data)
        {
            crc ^= static_cast<uint16_t>(byte) << 8;

            for (int i = 0; i < 8; i++)
            {
                if (crc & 0x8000)
                {
                    crc = (crc << 1) ^ 0x1021; // Apply polynomial
                }
                else
                {
                    crc = crc << 1; // Shift only
                }
            }
        }

        return crc;
    }

    // Initialize Parser in idle
    VESCPacket::Parser::Parser() : state_(State::IDLE), expected_length_(0), crc_(0) {}

    // State machine for processing a single byte from serial
    // Returns true for a complete valid byte
    bool VESCPacket::Parser::process_byte(uint8_t byte, std::vector<uint8_t> &payload_out)
    {
        switch (state_)
        {
        case State::IDLE:
            // Look for start of packet
            if (byte == 0x02)
            {
                state_ = State::GET_LENGTH_SHORT;
                payload_.clear();
            }
            else if (byte == 0x03)
            {
                state_ = State::GET_LENGTH_LONG_1;
                payload_.clear();
            }
            break;

        case State::GET_LENGTH_SHORT:
            // Short packet length
            expected_length_ = byte;
            state_ = State::GET_PAYLOAD;
            break;

        case State::GET_LENGTH_LONG_1:
            // Long packet length high byte
            expected_length_ = static_cast<uint16_t>(byte) << 8;
            state_ = State::GET_LENGTH_LONG_2;
            break;

        case State::GET_LENGTH_LONG_2:
            // Long packet length low byte
            expected_length_ |= byte;
            state_ = State::GET_PAYLOAD;
            break;

        case State::GET_PAYLOAD:
            // Collect payload bytes
            payload_.push_back(byte);

            // Move to CRC when payload complete
            if (payload_.size() >= expected_length_)
            {
                state_ = State::GET_CRC_HIGH;
            }
            break;

        case State::GET_CRC_HIGH:
            // CRC high byte
            crc_ = static_cast<uint16_t>(byte) << 8;
            state_ = State::GET_CRC_LOW;
            break;

        case State::GET_CRC_LOW:
            // CRC low byte
            crc_ |= byte;
            state_ = State::GET_STOP;
            break;

        case State::GET_STOP:
            // Check stop byte and validate CRC
            if (byte == 0x03)
            {
                uint16_t calculated_crc = VESCPacket::calculate_crc(payload_);
                if (calculated_crc == crc_)
                {
                    // Valid packet received
                    payload_out = payload_;
                    state_ = State::IDLE;
                    return true;
                }
            }
            // Invalid packet, reset
            state_ = State::IDLE;
            break;
        }

        return false; // Packet not complete
    }

    // Reset parser to initial state
    void VESCPacket::Parser::reset()
    {
        state_ = State::IDLE;
        payload_.clear();
        expected_length_ = 0;
        crc_ = 0;
    }

} // namespace arcus_hardware