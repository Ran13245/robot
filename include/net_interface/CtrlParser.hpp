#pragma once

#include "DataParser.hpp"
#include "protocol.hpp"
#include "CRC.h"
namespace Protocol
{
    template <>
    struct SocketParser<FullbodyState, FullbodyState>
    {
        using BinBuffer = std::span<std::byte>;
        using SenderType = FullbodyState;
        using ReceiverType = FullbodyState;
        static constexpr size_t SenderMsgSize = 76;
        static constexpr size_t ReceiverMsgSize = 76;

        static inline void Encode(const SenderType &state, BinBuffer &buffer)
        {
            uint32_t tmp_32bits;
            buffer[0] = static_cast<std::byte>(0xFB);
            buffer[1] = static_cast<std::byte>(state.mask);
            memcpy(&buffer[2], &state.cnt, sizeof(uint32_t));
            memcpy(&buffer[6], &state.time, sizeof(uint64_t));
            memcpy(&buffer[14], state.base_pos.data(), sizeof(float) * 3);
            tmp_32bits = Encode3D<float, 10>(state.left_hand_pos.x(), state.left_hand_pos.y(), state.left_hand_pos.z(), 2.0);
            memcpy(&buffer[26], &tmp_32bits, sizeof(uint32_t));
            tmp_32bits = Encode3D<float, 10>(state.right_hand_pos.x(), state.right_hand_pos.y(), state.right_hand_pos.z(), 2.0);
            memcpy(&buffer[30], &tmp_32bits, sizeof(uint32_t));

            auto quats = EncodeQuaternions<float>(state.base_quat, state.left_hand_quat, state.right_hand_quat);
            memcpy(&buffer[34], &quats.first, sizeof(uint64_t));
            memcpy(&buffer[42], &quats.second, sizeof(uint64_t));

            tmp_32bits = Encode3D<float, 10>(state.base_lin_vel.x(), state.base_lin_vel.y(), state.base_lin_vel.z(), 3.0);
            memcpy(&buffer[50], &tmp_32bits, sizeof(uint32_t));
            tmp_32bits = Encode3D<float, 10>(state.base_ang_vel.x(), state.base_ang_vel.y(), state.base_ang_vel.z(), 3.14);
            memcpy(&buffer[54], &tmp_32bits, sizeof(uint32_t));
            tmp_32bits = Encode3D<float, 10>(state.left_hand_lin_vel.x(), state.left_hand_lin_vel.y(), state.left_hand_lin_vel.z(), 5.0);
            memcpy(&buffer[58], &tmp_32bits, sizeof(uint32_t));
            tmp_32bits = Encode3D<float, 10>(state.right_hand_lin_vel.x(), state.right_hand_lin_vel.y(), state.right_hand_lin_vel.z(), 5.0);
            memcpy(&buffer[62], &tmp_32bits, sizeof(uint32_t));
            tmp_32bits = Encode3D<float, 10>(state.left_hand_ang_vel.x(), state.left_hand_ang_vel.y(), state.left_hand_ang_vel.z(), 6.28);
            memcpy(&buffer[66], &tmp_32bits, sizeof(uint32_t));
            tmp_32bits = Encode3D<float, 10>(state.right_hand_ang_vel.x(), state.right_hand_ang_vel.y(), state.right_hand_ang_vel.z(), 6.28);
            memcpy(&buffer[70], &tmp_32bits, sizeof(uint32_t));

            std::uint16_t crc = CRC::CalculateBits(buffer.data(), 74, CRC::CRC_16_KERMIT());
            memcpy(&buffer[74], &crc, sizeof(uint16_t));
            return;
        }

        static inline void Decode(const BinBuffer &buffer, ReceiverType &data)
        {
            return; // TODO
        }
    };
} // namespace Protocol