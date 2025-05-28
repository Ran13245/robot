// Copyright 2025 Chuangye Liu <chuangyeliu0206@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <span>
#include <concepts>
#include <type_traits>
#include <cstring>
namespace Protocol
{
    template <typename Parser>
    concept isValidParser = requires(const typename Parser::SenderType &s, Parser::BinBuffer &buf, const Parser::BinBuffer &cbuf, typename Parser::ReceiverType &r) {
        typename Parser::BinBuffer;

        { Parser::SenderMsgSize } -> std::convertible_to<std::size_t>;
        { Parser::ReceiverMsgSize } -> std::convertible_to<std::size_t>;

        { Parser::Encode(s, buf) } -> std::same_as<void>;
        { Parser::Decode(cbuf, r) } -> std::same_as<void>;
    };

    template <typename SenderType_, typename ReceiverType_>
    struct SocketParser
    {
        using BinBuffer = std::span<std::byte>;
        using SenderType = SenderType_;
        using ReceiverType = ReceiverType_;
        static constexpr size_t SenderMsgSize = 0;
        static constexpr size_t ReceiverMsgSize = 0;
        static constexpr void Encode(const SenderType &data, BinBuffer &buffer) {};
        static constexpr void Decode(const BinBuffer &buffer, ReceiverType &data) {};
    };

    /*------------------------------------------Direct Parser-----------------------------------------------------*/

    template <typename T>
    concept isDirectType = std::is_same_v<T, std::vector<std::byte>> || std::is_same_v<T, std::span<std::byte>>;

    /*--------------------------------------Dummy Specialization--------------------------------------------------*/
    struct DummySender
    {
        int dummy;
    };

    struct DummyReceiver
    {
        int dummy;
    };

    template <>
    struct SocketParser<DummySender, DummyReceiver>
    {
        using BinBuffer = std::span<std::byte>;
        using SenderType = DummySender;
        using ReceiverType = DummyReceiver;
        static constexpr size_t SenderMsgSize = sizeof(DummySender);
        static constexpr size_t ReceiverMsgSize = sizeof(DummyReceiver);

        static constexpr void Encode(const DummySender &data, BinBuffer &buffer)
        {
            std::memcpy(buffer.data(), &data, sizeof(DummySender));
        }

        static constexpr void Decode(const BinBuffer &buffer, DummyReceiver &data)
        {
            std::memcpy(&data, buffer.data(), sizeof(DummyReceiver));
        }
    };

}; // namespace Protocol