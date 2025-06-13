#pragma once

#include <vector>
#include <string>
#include <memory>
#include <iostream>
#include <tuple>
#include <algorithm>


#include <Eigen/Eigen>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include "configParser.hpp"


#include "itc/backend/RingBuf.hpp"
#include "comm_channel.hpp"
#include "whole_body_msg/whole_body_receiver.hpp"
#include "whole_body_msg/whole_body_msg.h"


namespace WHU_ROBOT{

	class CarCmd2ROSHandler{
	public:
		explicit CarCmd2ROSHandler(const param_t& _param, const ros::NodeHandle& _nh):
			nh{_nh},
			param{_param},
			io_context{},
			channel(io_context, param.car_cmd_local_ip, param.car_cmd_local_port, 
				param.car_cmd_remote_ip, param.car_cmd_remote_port),
			recv_mq(RingBuffer<whole_body_msg>{10})
		{
			std::cout<<"CarCmd2ROSHandler constructing"<<std::endl;
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
		CommChannel<ChannelMode::UDP, WholeBodyReceiver> channel;
		MsgQueue recv_mq;

		asio::executor_work_guard<asio::io_context::executor_type> work_guard = 
			asio::make_work_guard(io_context);

		void cmdPublish(const Eigen::Vector3f& position,
                               const Eigen::Quaternionf& orientation);

		auto decodePacket(const whole_body_msg& packet) -> std::tuple<Eigen::Vector3f, Eigen::Quaternionf>{
			    // Copy the first three floats into a Vector3f
			Eigen::Vector3f pos;
			pos << packet.base_pos[0],
				packet.base_pos[1],
				packet.base_pos[2];

			// Copy all four floats into a Quaternionf (w, x, y, z)
			Eigen::Quaternionf quat(
				packet.base_quat[0],  // w
				packet.base_quat[1],  // x
				packet.base_quat[2],  // y
				packet.base_quat[3]   // z
			);

			return { pos, quat };
		}
	};

	void CarCmd2ROSHandler::exec(void){
		whole_body_msg recv_data;
		// std::cout << "DEBUG:---------------------"<<recv_mq.size()<<std::endl;
		if(!recv_mq.empty()){
			recv_mq.dequeue(recv_data);
			ROS_INFO("get command data");
			auto [_position, _orientation] = decodePacket(recv_data);
			cmdPublish(_position, _orientation);
		}
	}

	void CarCmd2ROSHandler::init(void){

 		channel.bind_message_queue("whole_body_receiver", ParserType::Receiver, recv_mq);
		
  		while(!channel.enable_receiver()){std::cout<<
			"CarCmd2ROSHandler: waiting enable_receiver"<<std::endl;}

  		t = std::thread([this]() { io_context.run(); });


		ROS_INFO("CarCmd2ROS start");
	}


	void CarCmd2ROSHandler::stop(void){
		std::cout << "CarCmd2ROSHandler Stopping..." << std::endl;
		io_context.stop();
		// while(!t.joinable()) {std::cout<<"CarCmd2ROSHandler: waiting thread joinable"<<std::endl;}
		t.join();
		std::cout << "CarCmd2ROSHandler Stopped" << std::endl;

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

