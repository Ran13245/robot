#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <span>


#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "configParser.hpp"
#include "itc/backend/RingBuf.hpp"
#include "comm_channel.hpp"


#include "nav_state_msg/nav_state_msg.h"
#include "nav_state_msg/nav_state_sender.hpp"
#include "nav_state_msg/nav_state_receiver.hpp"

#include "whole_body_msg/whole_body_msg.h"
#include "whole_body_msg/whole_body_sender.hpp"
#include "whole_body_msg/whole_body_receiver.hpp"



namespace WHU_ROBOT {

	class MergeHandler {
		public:
			explicit MergeHandler(const param_t& _param);

			~MergeHandler(){}

			void init(void);
			void stop(void);

			void exec(void);

		private:
			param_t param;
			asio::io_context io_context;
			std::thread t;
			CommChannel<ChannelMode::UDP, NavStateSender, WholeBodyReceiver> remote_channel;
			CommChannel<ChannelMode::Unix, NavStateReceiver> local_channel_odom_rcv;
			CommChannel<ChannelMode::Unix, WholeBodySender> local_channel_carcmd_snd;

			MsgQueue recv_mq_remote;
			MsgQueue send_mq_remote;
			MsgQueue recv_mq_local_odom;
			MsgQueue send_mq_local_carcmd;

			nav_state_msg stateMerge(nav_state_msg& msg_from_odom){return msg_from_odom;}
	};

	inline MergeHandler::MergeHandler(const param_t& _param):
		param{_param},
		io_context{},
		remote_channel(io_context, param.merge_local_ip, param.merge_local_port, 
			param.merge_remote_ip, param.merge_remote_port),
		local_channel_odom_rcv(io_context, param.odom_unix_channel, param.odom_unix_channel),
		local_channel_carcmd_snd(io_context, param.cmd_unix_channel, param.cmd_unix_channel),
		recv_mq_remote(RingBuffer<whole_body_msg>{10}),
		send_mq_remote(RingBuffer<nav_state_msg>{10}),
		recv_mq_local_odom(RingBuffer<nav_state_msg>{10}),
		send_mq_local_carcmd(RingBuffer<whole_body_msg>{10})
	{
		std::cout<<"MergeHandler constructing"<<std::endl;
	}

	inline void MergeHandler::init(void){
		std::cout << "MergeHandler Starting..." << std::endl;

		remote_channel.bind_message_queue("nav_state_sender", ParserType::Sender, send_mq_remote);
		remote_channel.bind_message_queue("whole_body_receiver", ParserType::Receiver, recv_mq_remote);
		local_channel_odom_rcv.bind_message_queue("nav_state_receiver", ParserType::Receiver, 
				recv_mq_local_odom);
		local_channel_carcmd_snd.bind_message_queue("whole_body_sender", ParserType::Sender, 
				send_mq_local_carcmd);

		remote_channel.enable_receiver();
		remote_channel.enable_sender();
		local_channel_odom_rcv.enable_receiver();
		local_channel_carcmd_snd.enable_sender();

		t = std::thread([this]() { io_context.run(); });
		
		std::cout << "MergeHandler Start" << std::endl;
	}

	inline void MergeHandler::stop(void){
		std::cout << "MergeHandler Stopping..." << std::endl;

		io_context.stop();
		t.join();

		std::cout << "MergeHandler Stopped" << std::endl;
	}

	inline void MergeHandler::exec(void){
		// std::cout << "DEBUG:---------------------"<<recv_mq.size()<<std::endl;

		static nav_state_msg recv_odom;
		// static nav_state_msg recv_arm;
		static whole_body_msg recv_remote_data;
		static nav_state_msg merged_msg;

		static bool get_local_odom = 0;
		static bool get_local_arm = 0;

		if(!recv_mq_local_odom.empty()){
			recv_mq_local_odom.dequeue(recv_odom);
			std::cout<<"get odom data"<<std::endl;

			get_local_odom = 1;
		}

		// if(!recv_mq_local_arm.empty()){ //TODO
			get_local_arm = 1;
		// }

		if(get_local_odom && get_local_arm){
			merged_msg = stateMerge(recv_odom);
			send_mq_remote.enqueue(merged_msg);
		}

		if(!recv_mq_remote.empty()){
			
			recv_mq_remote.dequeue(recv_remote_data);
			std::cout<<"get command data"<<std::endl;
			
			send_mq_local_carcmd.enqueue(recv_remote_data);
			// local_channel_arm_snd.enqueue(recv_remote_data);
		}

	}

}//namespace WHU_ROBOT
