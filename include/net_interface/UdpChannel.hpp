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

#include <concepts>
#include <memory>
#include <asio.hpp>
#include <asio/steady_timer.hpp>
#include "DataParser.hpp"
#include "RingBuf.hpp"

namespace IO_Comm
{
    using namespace Protocol;

    template <typename Parser>
        requires isValidParser<Parser>
    class UdpChannel
    {

    public:
        using SenderBuffer = RingBuffer<std::shared_ptr<typename Parser::SenderType>>;
        using ReceiverBuffer = RingBuffer<std::shared_ptr<typename Parser::ReceiverType>>;
        using SenderBufferPtr = std::weak_ptr<SenderBuffer>;
        using ReceiverBufferPtr = std::shared_ptr<ReceiverBuffer>;
        UdpChannel() = delete; // default constructor is deleted

        UdpChannel(asio::io_context &io_context, std::string local_ip, int local_port,
                   std::string remote_ip, int remote_port) : local_endpoint_(asio::ip::make_address(local_ip), local_port),
                                                             remote_endpoint_(asio::ip::make_address(remote_ip), remote_port),
                                                             socket_(io_context, asio::ip::udp::v4()), timer_(io_context)
        {
            socket_.bind(local_endpoint_);

            receiver_buffer_ = std::make_shared<ReceiverBuffer>(100);
        }

        void enable_sender()
        {
            if (sender_buffer_.expired())
            {
                throw std::runtime_error("sender_buffer_ is null, cannot enable sender.");
            }
            timer_.async_wait(std::bind(&UdpChannel::timer_handler, this, std::placeholders::_1));
            return;
        }
        void enable_receiver()
        {
            this->socket_.async_receive_from(asio::buffer(receiver_bin_data), remote_endpoint_,
                                             std::bind(&UdpChannel::receiver_handler, this, std::placeholders::_1, std::placeholders::_2));
        }

        void register_sender_buffer(std::shared_ptr<SenderBuffer> ptr)
        {
            this->sender_buffer_ = ptr;
            return;
        }

        auto get_receiver_buffer()
        {
            return this->receiver_buffer_;
        }

    private:
        void timer_handler(const asio::error_code &ec)
        {
            if (!ec)
            {
                auto tmp_sender_buffer_ = this->sender_buffer_.lock();
                if (!tmp_sender_buffer_->empty())
                {
                    std::span<std::byte> buffer_view(this->sender_bin_data);
                    auto data = tmp_sender_buffer_->pop();
                    if constexpr (isDirectType<typename Parser::SenderType>)
                    {
                        this->socket_.async_send_to(asio::buffer(data->data(), data->size()), this->remote_endpoint_,
                                                    [](const asio::error_code &error, std::size_t bytes_transferred) {});
                    }
                    else
                    {
                        Parser::Encode(*data, buffer_view);
                        this->socket_.async_send_to(asio::buffer(this->sender_bin_data), this->remote_endpoint_,
                                                    [](const asio::error_code &error, std::size_t bytes_transferred) {});
                    }
                }
            }
            timer_.expires_after(asio::chrono::milliseconds(1));
            timer_.async_wait(std::bind(&UdpChannel::timer_handler, this, std::placeholders::_1));
            return;
        }

        void receiver_handler(const asio::error_code &ec, std::size_t bytes_transferred)
        {
            if (!ec && bytes_transferred > 0)
            {
                std::span<std::byte> buffer_view(this->receiver_bin_data);
                std::shared_ptr<typename Parser::ReceiverType> data = std::make_shared<typename Parser::ReceiverType>();
                Parser::Decode(buffer_view, *data);
                this->receiver_buffer_->push(data);
            }
            this->socket_.async_receive_from(asio::buffer(receiver_bin_data), remote_endpoint_,
                                             std::bind(&UdpChannel::receiver_handler, this, std::placeholders::_1, std::placeholders::_2));
        }

        asio::ip::udp::socket socket_;
        asio::steady_timer timer_;
        asio::ip::udp::endpoint local_endpoint_;
        asio::ip::udp::endpoint remote_endpoint_;

        SenderBufferPtr sender_buffer_;
        ReceiverBufferPtr receiver_buffer_;

        std::array<std::byte, Parser::SenderMsgSize> sender_bin_data;
        std::array<std::byte, Parser::ReceiverMsgSize> receiver_bin_data;
    };
};
