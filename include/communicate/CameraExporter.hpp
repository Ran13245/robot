// PointCloudExporter.hpp
#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <string>
#include <memory>
#include <iostream>
#include <tuple>
#include <algorithm>

#include "base_type.hpp"
#include "configParser.hpp"

#include <Eigen/Eigen>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/random_sample.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/common/concatenate.h>
#include <eigen_conversions/eigen_msg.h>


#include "itc/backend/RingBuf.hpp"
#include "comm_channel.hpp"
#include "camera_msg/camera_sender.hpp"
#include "camera_msg/camera_msg.h"

#include "algo.hpp"



namespace WHU_ROBOT{



class CameraExporter {
public:

	explicit CameraExporter(const param_t& _param, size_t initial_pool_size = 1024);

	void init(void);
	void stop(void);
/**
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::fromROSMsg(*msg, *cloud);
*/
	void addPoints(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& _cloud,Eigen::Vector3f& base_pos, Eigen::Matrix3f& base_rot);

private:

	/*-----------------------------consts-----------------------------------------*/

	static constexpr size_t PACKET_MTU = 1400; // 可调整
	using _PacketType = PointPacket<CompressedPoint, PACKET_MTU>;
	using _SenderType = camera_msg;
	using _PollType = PointPoll<CompressedPoint>;

	static constexpr size_t __pack_size = _PacketType::TotalByte;

	static constexpr size_t transmit_frame_cnt_limit = 3; //每多少帧点云压一次传输包
	/*-----------------------------functions-----------------------------------------*/
	void transmitCloud(_PollType& transmit_poll);
	/*-----------------------------objs-----------------------------------------*/
	param_t param;

	const size_t initial_pool_size_; // 初始池大小

	// _PollType global_point_poll_;

	asio::io_context io_context;
	std::thread t;
	CommChannel<ChannelMode::UDP, CameraSender> channel;
	MsgQueue send_mq;
};

inline  CameraExporter::CameraExporter(const param_t& _param, size_t initial_pool_size):
	param{_param},
	initial_pool_size_{initial_pool_size},
	/* global_point_poll_{initial_pool_size_},*/
	io_context{},
	channel(io_context, param.camera_local_ip, param.camera_local_port, 
		param.camera_remote_ip, param.camera_remote_port),
	send_mq(RingBuffer<_SenderType>{50})
{
	std::cout<<"CameraExporter constructing"<<std::endl;

	if(CloudSender::length != __pack_size){
		std::cout<<"\033[31m点云传输包大小与协议不匹配！\033[0m"<<std::endl;
	}
}


inline void CameraExporter::init(void){
	std::cout << "CameraExporter Starting..." << std::endl;

	if(param.enable_camera_trans){

		std::cout << "camera_trans enabled" << std::endl;

		channel.bind_message_queue("camera_sender", ParserType::Sender, send_mq);

		while(!channel.enable_sender()){std::cout<<
			"CameraExporter: waiting enable_sender"<<std::endl;}

		std::cout << "camera transmit Starting..." << std::endl;

		t = std::thread([this]() { io_context.run(); });

	} else {
		std::cout << "camera_trans not enable" << std::endl;
	}
	std::cout << "CameraExporter Started" << std::endl;
}

inline void CameraExporter::stop(void){
	if(param.enable_camera_trans){
	std::cout << "CameraExporter Stopping..." << std::endl;
		io_context.stop();
		t.join();
	std::cout << "CameraExporter Stopped" << std::endl;
	}
}

inline void CameraExporter::addPoints(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, Eigen::Vector3f& base_pos, Eigen::Matrix3f& base_rot)
{
	constexpr uint32_t MASK_DYNAMIC = 0x90'00'00'00;
	static Eigen::Matrix3f R2 = Eigen::AngleAxisf{M_PI / 6, Eigen::Vector3f::UnitY()}.toRotationMatrix();  // camera -> body， camera in body
	static Eigen::Vector3f p2 = Eigen::Vector3f{0,0,0.1};



// std::cout<<"DEBUG: CALL FROM addPoints"<<std::endl;
	if(param.enable_camera_trans){

		static size_t transmit_frame_cnt = 0;
		transmit_frame_cnt++;

		static _PollType transmit_poll{initial_pool_size_};

		if(transmit_frame_cnt >= transmit_frame_cnt_limit){

		size_t __cnt = 0;


		pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::RandomSample<pcl::PointXYZRGB> sampler;
		sampler.setInputCloud(cloud);
		sampler.setSample(std::max<size_t>(1, cloud->size() / 40));  // 1/4
		sampler.filter(*downsampled);

		for (const auto& pt : downsampled->points) {
			if (!std::isfinite(pt.x)) continue;

			Eigen::Vector3f P_camera(pt.z, -pt.x, -pt.y);

			// Transform to world frame
			Eigen::Vector3f P_world = base_rot * (R2 * P_camera + p2) + base_pos;

	
			CompressedPoint base_pt{Eigen::Vector4f( P_camera.x(), P_camera.y(), P_camera.z(), 0u ), 
				((static_cast<uint32_t>(pt.r) << 24) |
				(static_cast<uint32_t>(pt.g) << 16) |
				(static_cast<uint32_t>(pt.b) <<  8) |
				static_cast<uint32_t>(255u)),	MASK_DYNAMIC};
			transmit_poll.AddPoint(base_pt);
			__cnt++;

		}
		std::cout<<"CameraExporter: get available point num: " << __cnt <<std::endl;

			transmitCloud(transmit_poll);
			transmit_poll.clear();
			transmit_frame_cnt = 0;
		}
	}
	
}


inline void CameraExporter::transmitCloud(_PollType& transmit_poll){
	if(param.enable_camera_trans){
		std::cout << "camera transmiting " << std::endl;
		auto packets_ptr = transmit_poll.EncodePackets<_PacketType>(0, transmit_poll.size(), 4);

		for(auto& pkt : *packets_ptr){
			
			send_mq.enqueue(std::move(pkt));
			
		}	
		
		std::cout << "camera transmit enqueueed " << std::endl;
	}
}



};//namespace WHU_robot