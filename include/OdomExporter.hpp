#pragma once

#include "TaskBase.hpp"
#include "CtrlParser.hpp"
#include "RingBuf.hpp"
#include "UdpChannel.hpp"


namespace Schedule
{
    using namespace IO_Comm;
    using namespace Protocol;
    class CommTask : public TaskBase
    {
    public:
        using channelType = UdpChannel<SocketParser<FullbodyState, FullbodyState>>;
		PCDTransmitTask(std::string remote_ip, int remote_port, std::string local_ip, int local_port):
			remote_ip_(remote_ip),
			remote_port_(remote_port),
			local_ip_(local_ip),
			local_port_(local_port)
		{}
    protected:
        void task() override
        {
            channelType channel(io_context,
                                "127.0.0.1", 12345,
                                "127.0.0.1", 12345);
            auto cmd_msg_queue = quiry_msg_queue("whole_body_ctrl_cmd");
            auto typed_ptr = std::static_pointer_cast<RingBuffer<std::shared_ptr<FullbodyState>>>(cmd_msg_queue->getRawBuffer());
            channel.register_sender_buffer(typed_ptr);
            channel.enable_sender();
            channel.enable_receiver();
            io_context.run();
        }

	void stop() override
		{
			std::cout << "io_context Stopping" << std::endl;
			io_context.stop();
			TaskBase::stop();
			std::cout << "PCDTransmitTask Stopped" << std::endl;
		}

    private:
        asio::io_context io_context;
    };

}; // namespace Schedule
