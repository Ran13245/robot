#pragma once

#include <vector>
#include <string>
#include <memory>
#include <iostream>
#include <tuple>
#include <algorithm>

#include "base_type.hpp"

#include <Eigen/Eigen>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include "configParser.hpp"

// #include "TaskBase.hpp"
// #include "CtrlParser.hpp"
// #include "RingBuf.hpp"
// #include "UdpChannel.hpp"


// #include "DataParser.hpp"
// #include "protocol.hpp"
// #include "CRC.h"

#include "itc/backend/RingBuf.hpp"
#include "comm_channel.hpp"
#include "whole_body_msg/whole_body_receiver.hpp"
#include "whole_body_msg/whole_body_sender.hpp"
#include "whole_body_msg/whole_body_msg.h"

// namespace Protocol
// {
//     template <>
//     struct SocketParser<FullbodyCmd, FullbodyCmd>
//     {
//         using BinBuffer = std::span<std::byte>;
//         using SenderType = FullbodyCmd;
//         using ReceiverType = FullbodyCmd;
//         static constexpr size_t SenderMsgSize = 112;
//         static constexpr size_t ReceiverMsgSize = 112;

//         static inline void Encode(const SenderType &state, BinBuffer &buffer)
//         {
// 		std::cout<<"illegal call from FullbodyCmd Encode, unused function"<<std::endl;
//             return;
//         }

//         static inline void Decode(const BinBuffer &buffer, ReceiverType &data)
//         {
// 		memset(&data, 0, sizeof(data));
// 		uint16_t crc = CRC::CalculateBits(buffer.data(), ReceiverMsgSize - sizeof(FullbodyCmd::crc_bits),
// 				CRC::CRC_16_KERMIT());
// 		if (crc != *(uint16_t*)&buffer[ReceiverMsgSize - sizeof(FullbodyCmd::crc_bits)])
// 		{
// 			throw std::runtime_error("CRC check failed");
// 		}

// 		size_t _offset = 0;

// 		data.header = *(decltype(data.header)*)(&buffer[_offset]);
// 		_offset += sizeof(data.header);
// 		data.mask = *(decltype(data.mask)*)(&buffer[_offset]);
// 		_offset += sizeof(data.mask);
// 		data.cnt = *(decltype(data.cnt)*)(&buffer[_offset]);
// 		_offset += sizeof(data.cnt);
// 		data.time = *(decltype(data.time)*)(&buffer[_offset]);
// 		_offset += sizeof(data.time);
// 		data.base_x = *(decltype(data.base_x)*)(&buffer[_offset]);
// 		_offset += sizeof(data.base_x);
// 		data.base_y = *(decltype(data.base_y)*)(&buffer[_offset]);
// 		_offset += sizeof(data.base_y);
// 		data.base_z = *(decltype(data.base_z)*)(&buffer[_offset]);
// 		_offset += sizeof(data.base_z);
// 		data.left_hand_pos = *(decltype(data.left_hand_pos)*)(&buffer[_offset]);
// 		_offset += sizeof(data.left_hand_pos);
// 		data.right_hand_pos = *(decltype(data.right_hand_pos)*)(&buffer[_offset]);
// 		_offset += sizeof(data.right_hand_pos);
// 		// data.rotation_upper = *(decltype(data.rotation_upper)*)(&buffer[_offset]);
// 		// _offset += sizeof(data.rotation_upper);
// 		// data.rotation_lower = *(decltype(data.rotation_lower)*)(&buffer[_offset]);
// 		// _offset += sizeof(data.rotation_lower);
// 		memcpy(&data.base_quat, &buffer[_offset], sizeof(data.base_quat));
// 		_offset += sizeof(data.base_quat);
// 		memcpy(&data.left_hand_quat, &buffer[_offset], sizeof(data.left_hand_quat));
// 		_offset += sizeof(data.left_hand_quat);
// 		memcpy(&data.right_hand_quat, &buffer[_offset], sizeof(data.right_hand_quat));
// 		_offset += sizeof(data.right_hand_quat);
// 		data.base_vel = *(decltype(data.base_vel)*)(&buffer[_offset]);
// 		_offset += sizeof(data.base_vel);
// 		data.base_omega = *(decltype(data.base_omega)*)(&buffer[_offset]);
// 		_offset += sizeof(data.base_omega);
// 		data.left_hand_vel = *(decltype(data.left_hand_vel)*)(&buffer[_offset]);
// 		_offset += sizeof(data.left_hand_vel);
// 		data.right_hand_vel = *(decltype(data.right_hand_vel)*)(&buffer[_offset]);
// 		_offset += sizeof(data.right_hand_vel);
// 		data.left_hand_omega = *(decltype(data.left_hand_omega)*)(&buffer[_offset]);
// 		_offset += sizeof(data.left_hand_omega);
// 		data.right_hand_omega = *(decltype(data.right_hand_omega)*)(&buffer[_offset]);
// 		_offset += sizeof(data.right_hand_omega);
// 		data.right_gripper_ctrl = *(decltype(data.right_gripper_ctrl)*)(&buffer[_offset]);
// 		_offset += sizeof(data.right_gripper_ctrl);
// 		data.left_gripper_ctrl = *(decltype(data.left_gripper_ctrl)*)(&buffer[_offset]);
// 		_offset += sizeof(data.left_gripper_ctrl);
// 		data.crc_bits = *(decltype(data.crc_bits)*)(&buffer[_offset]);

// 		return;
//         }
//     };
// } // namespace Protocol


// namespace Schedule{

// 	using namespace IO_Comm;
// 	using namespace Protocol;

// 	class TargetReceiveTask : public TaskBase
// 	{
// 	public:
// 		using channelType = UdpChannel<SocketParser<FullbodyCmd, FullbodyCmd>>;
// 		TargetReceiveTask(std::string remote_ip, int remote_port, std::string local_ip, int local_port):
// 			remote_ip_(remote_ip),
// 			remote_port_(remote_port),
// 			local_ip_(local_ip),
// 			local_port_(local_port)
// 		{}

// 		~TargetReceiveTask(){}


// 		void stop() override
// 		{
// 			io_context.stop();
// 			TaskBase::stop();
// 			std::cout << "TargetReceiveTask Stopped" << std::endl;
// 		}

// 		channelType* channel_ptr;
// 	protected:
// 		void task() override	
// 		{
// 			channelType channel(io_context,
// 					local_ip_, local_port_,
// 					remote_ip_, remote_port_);
// 			channel.enable_receiver();
// 			channel_ptr = &channel;
// 			io_context.run();
		
// 		}

// 	private:
// 		asio::io_context io_context;

// 		std::string remote_ip_;
// 		int remote_port_;
// 		std::string local_ip_;
// 		int local_port_;
// 	};

// };


namespace WHU_ROBOT{

	// using namespace IO_Comm;
	// using namespace Protocol;
	// using namespace Schedule;

	class CarCmd2ROSHandler{
	public:
		explicit CarCmd2ROSHandler(const param_t& _param, const ros::NodeHandle& _nh):
			nh{_nh},
			param{_param},
			io_context{},
			channel(io_context, param.car_cmd_remote_ip, param.car_cmd_remote_port, 
				param.car_cmd_local_ip, param.car_cmd_local_port),
			recv_mq(RingBuffer<whole_body_msg>{10})
		{
			target_odom_pub = nh.advertise<nav_msgs::Odometry>(param.target_odom_topic, 10);
		}

		~CarCmd2ROSHandler(){}

		void init(void);
		void stop(void);

		void exec(void);
	private:
		ros::NodeHandle nh;
		param_t param;
		ros::Publisher target_odom_pub;

		asio::io_context io_context;
		std::thread t;
		CommChannel<ChannelMode::UDP, WholeBodySender, WholeBodyReceiver> channel;
		MsgQueue recv_mq;

		// TargetReceiveTask transmit_task;
		void cmdPublish(const Eigen::Vector3f& position,
                               const Eigen::Quaternionf& orientation);
		// auto decodePacket(const FullbodyCmd& packet) -> std::tuple<Eigen::Vector3f, Eigen::Quaternionf>{
		// 	Eigen::Vector3f _position;
		// 	Eigen::Quaternionf _orientation;

		// 	_position << packet.base_x, packet.base_y, packet.base_z;
		// 	memcpy(_orientation.coeffs().data(), &packet.base_quat, sizeof(packet.base_quat));

		// 	return {_position, _orientation};
		// }
		// auto decodePacket(const FullbodyState& packet) -> std::tuple<Eigen::Vector3f, Eigen::Quaternionf>{
		// 	return {packet.base_pos, packet.base_quat};
		// }
		auto decodePacket(const whole_body_msg& packet) -> std::tuple<Eigen::Vector3f, Eigen::Quaternionf>{
			return {
				Eigen::Map<Eigen::Vector3f>{packet.base_pos.data()}, 
				Eigen::Map<Eigen::Quaternionf>{packet.base_quat.data()}
			};
		}
	};

	void CarCmd2ROSHandler::exec(void){
		// auto rec_buf = transmit_task.channel_ptr->get_receiver_buffer();

		// std::shared_ptr<FullbodyCmd> data_ptr; 					//attention received data type!
		// if(rec_buf->try_pop(data_ptr)){
		// 	ROS_INFO("get command data");
		// 	auto [_position, _orientation] = decodePacket(*data_ptr);

		// 	cmdPublish(_position, _orientation);
		// }

		whole_body_msg recv_data;
		while(recv_mq.size()){
			recv_mq.dequeue(recv_data);
			ROS_INFO("get command data");
			auto [_position, _orientation] = decodePacket(recv_data);
			cmdPublish(_position, _orientation);
		}
	}

	void CarCmd2ROSHandler::init(void){
		// transmit_task.start();

 		channel.bind_message_queue("whole_body_receiver", ParserType::Receiver, recv_mq);
		
  		while(!channel.enable_receiver()){std::cout<<
			"CarCmd2ROSHandler: waiting enable_receiver"<<std::endl;}

  		t = std::thread([this]() { io_context.run(); });


		ROS_INFO("CarCmd2ROS start");
	}


	void CarCmd2ROSHandler::stop(void){
		// transmit_task.stop();
		io_context.stop();
		while(!t.joinable()) {std::cout<<"CarCmd2ROSHandler: waiting thread joinable"<<std::endl;}
		t.join();

		ROS_INFO("CarCmd2ROS stop");
	}

	inline void CarCmd2ROSHandler::cmdPublish(const Eigen::Vector3f& position,
		const Eigen::Quaternionf& orientation)
	{
		nav_msgs::Odometry odom_msg;

		// 设置时间戳和坐标系
		odom_msg.header.stamp = ros::Time::now();
		odom_msg.header.frame_id = "odom";
		odom_msg.child_frame_id = "base_link";

		// 设置位置
		odom_msg.pose.pose.position.x = position.x();
		odom_msg.pose.pose.position.y = position.y();
		odom_msg.pose.pose.position.z = position.z();

		// Eigen::Quaternionf 转 geometry_msgs::Quaternion
		geometry_msgs::Quaternion q_msg;
		q_msg.x = orientation.x();
		q_msg.y = orientation.y();
		q_msg.z = orientation.z();
		q_msg.w = orientation.w();
		odom_msg.pose.pose.orientation = q_msg;

		target_odom_pub.publish(odom_msg);
	}

};//namespace WHU_ROBOT

