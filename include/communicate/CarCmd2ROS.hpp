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

#include "TaskBase.hpp"
#include "CtrlParser.hpp"
#include "RingBuf.hpp"
#include "UdpChannel.hpp"



namespace Schedule{

	using namespace IO_Comm;
	using namespace Protocol;

	class TargetReceiveTask : public TaskBase
	{
	public:
		using channelType = UdpChannel<SocketParser<FullbodyState, FullbodyState>>;
		TargetReceiveTask(std::string remote_ip, int remote_port, std::string local_ip, int local_port):
			remote_ip_(remote_ip),
			remote_port_(remote_port),
			local_ip_(local_ip),
			local_port_(local_port),
			channel(io_context,
				local_ip_, local_port_,
				remote_ip_, remote_port_)
		{}

		~TargetReceiveTask(){}


		void stop() override
		{
			io_context.stop();
			TaskBase::stop();
			std::cout << "TargetReceiveTask Stopped" << std::endl;
		}

		channelType channel;
	protected:
		void task() override	
		{
			// channelType channel(io_context,
			// 		local_ip_, local_port_,
			// 		remote_ip_, remote_port_);
			// auto odom_msg_queue = quiry_msg_queue("OdomMsg");
			// auto typed_ptr = std::static_pointer_cast<RingBuffer<std::shared_ptr<FullbodyState>>>(odom_msg_queue->getRawBuffer());
			// channel.register_sender_buffer(typed_ptr);
			channel.enable_receiver();
			io_context.run();
		
		}

	private:
		asio::io_context io_context;

		std::string remote_ip_;
		int remote_port_;
		std::string local_ip_;
		int local_port_;
	};

};


namespace WHU_ROBOT{

	using namespace IO_Comm;
	using namespace Protocol;
	using namespace Schedule;

	class CarCmd2ROSHandler{
	public:
		explicit CarCmd2ROSHandler(const param_t& _param, const ros::NodeHandle& _nh):
			nh{_nh},
			param{_param},
			transmit_task{param.car_cmd_remote_ip, param.car_cmd_remote_port, 
				param.car_cmd_local_ip, param.car_cmd_local_port}
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
		TargetReceiveTask transmit_task;
		void cmdPublish(const Eigen::Vector3f& position,
                               const Eigen::Quaternionf& orientation);
	};

	void CarCmd2ROSHandler::exec(void){
		auto rec_buf = transmit_task.channel.get_receiver_buffer();

		std::shared_ptr<FullbodyState> data_ptr;
		if(rec_buf->try_pop(data_ptr)){
			ROS_INFO("get command data");
			cmdPublish(data_ptr->base_pos, data_ptr->base_quat);
		}
	}

	void CarCmd2ROSHandler::init(void){
		transmit_task.start();
		ROS_INFO("CarCmd2ROS start");
	}


	void CarCmd2ROSHandler::stop(void){
		transmit_task.stop();
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

