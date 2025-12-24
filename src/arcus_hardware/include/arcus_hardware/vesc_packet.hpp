#ifndef VESC_PACKET_HPP
#define VESC_PACKET_HPP

#include <vector>
#include <cstdint>
#include <cstring>

namespace arcus_hardware
{

    class VESCPacket
    {
    public:
        VESCPacket() = default;
        ~VESCPacket() = default;

        // Pack a payload into a VESC packet with framing and CRC
        static std::vector<uint8_t> frame(const std::vector<uint8_t> &payload);

        // CRC16 calculation for VESC (polynomial 0x1021)
        static uint16_t calculate_crc(const std::vector<uint8_t> &data);

        // State machine for parsing incoming bytes
        class Parser
        {
        public:
            enum class State
            {
                IDLE,
                GET_LENGTH_SHORT,
                GET_LENGTH_LONG_1,
                GET_LENGTH_LONG_2,
                GET_PAYLOAD,
                GET_CRC_HIGH,
                GET_CRC_LOW,
                GET_STOP
            };

            Parser();

            // Process a single byte, returns true if packet is complete
            bool process_byte(uint8_t byte, std::vector<uint8_t> &payload_out);

            void reset();

        private:
            State state_;
            std::vector<uint8_t> payload_;
            uint16_t expected_length_;
            uint16_t crc_;
        };
    };

} // namespace arcus_hardware

#endif // VESC_PACKET_HPP