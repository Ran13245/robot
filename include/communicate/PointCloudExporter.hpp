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
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/common/concatenate.h>
#include <eigen_conversions/eigen_msg.h>


#include "itc/backend/RingBuf.hpp"
#include "comm_channel.hpp"
#include "cloud_msg/cloud_sender.hpp"
#include "cloud_msg/cloud_msg.h"

#include "algo.hpp"

static constexpr size_t PACKET_MTU = 1400; // 可调整
using _PacketType = PointPacket<CompressedPoint, PACKET_MTU>;
using _SenderType = cloud_msg;
using _PollType = PointPoll<CompressedPoint>;

static constexpr size_t __pack_size = _PacketType::TotalByte;

namespace WHU_ROBOT{


struct RobotArea {
	static constexpr uint32_t DynamicRange_n = 10;
	uint32_t x, y, z;
	auto operator<=>(const RobotArea&) const = default;
	explicit RobotArea(float pos_x, float pos_y, float pos_z){//实际位置转区块标号
		x = static_cast<uint32_t>(pos_x*100 + (1<<20))/(1u<<DynamicRange_n);
		y = static_cast<uint32_t>(pos_y*100 + (1<<20))/(1u<<DynamicRange_n);
		z = static_cast<uint32_t>(pos_z*100 + (1<<20))/(1u<<DynamicRange_n);
	}

	explicit RobotArea(Eigen::Vector3f& base_pos){//实际位置转区块标号
		x = static_cast<uint32_t>(base_pos.x()*100 + (1<<20))/(1u<<DynamicRange_n);
		y = static_cast<uint32_t>(base_pos.y()*100 + (1<<20))/(1u<<DynamicRange_n);
		z = static_cast<uint32_t>(base_pos.z()*100 + (1<<20))/(1u<<DynamicRange_n);
	}
};

class PointCloudExporter {
public:

	explicit PointCloudExporter(const param_t& _param, size_t initial_pool_size = 1024);

	void init(void);
	void stop(void);

	void addPoints(const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr& _cloud,Eigen::Vector3f& base_pos);
	bool saveToBinaryFile(const std::string &filename, size_t num_threads);

private:

	/*-----------------------------consts-----------------------------------------*/
	static constexpr size_t transmit_frame_cnt_limit = 10; //每多少帧雷达点云压一次传输包
	static constexpr uint32_t neighbor_range = 3;//range_meter = 2^neighbor_range * 0.01
	static constexpr size_t neighbor_limit_cnt = 10;
	/*-----------------------------functions-----------------------------------------*/
	uint32_t intensityToHeatmapRGBA(const float& _intensity);
	bool pointAvailable(_PollType& current_global_poll, const Eigen::Vector3f& point);
	void transmitCloud(_PollType& transmit_poll);
	/*-----------------------------objs-----------------------------------------*/
	param_t param;

	const size_t initial_pool_size_; // 初始池大小

	_PollType global_point_poll_;

	asio::io_context io_context;
	std::thread t;
	CommChannel<ChannelMode::UDP, CloudSender> channel;
	MsgQueue send_mq;
};

inline  PointCloudExporter::PointCloudExporter(const param_t& _param, size_t initial_pool_size):
	param{_param},
	initial_pool_size_{initial_pool_size},
	global_point_poll_{initial_pool_size_},
	io_context{},
	channel(io_context, param.local_ip, param.local_port, 
		param.remote_ip, param.remote_port),
	send_mq(RingBuffer<_SenderType>{50})
{
	std::cout<<"PointCloudExporter constructing"<<std::endl;

	if(CloudSender::length != __pack_size){
		std::cout<<"\033[31m点云传输包大小与协议不匹配！\033[0m"<<std::endl;
	}
}


inline void PointCloudExporter::init(void){
	std::cout << "PointCloudExporter Starting..." << std::endl;

	if(param.enable_pcd_trans){

		std::cout << "pcd_trans enabled" << std::endl;

		channel.bind_message_queue("cloud_sender", ParserType::Sender, send_mq);

		while(!channel.enable_sender()){std::cout<<
			"PointCloudExporter: waiting enable_sender"<<std::endl;}

		std::cout << "cloud transmit Starting..." << std::endl;

		t = std::thread([this]() { io_context.run(); });

	} else {
		std::cout << "pcd_trans not enable" << std::endl;
	}
	std::cout << "PointCloudExporter Started" << std::endl;
}

inline void PointCloudExporter::stop(void){
	if(param.enable_pcd_trans){
	std::cout << "PointCloudExporter Stopping..." << std::endl;
		io_context.stop();
		t.join();
	std::cout << "PointCloudExporter Stopped" << std::endl;
	}
}

inline void PointCloudExporter::addPoints(const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr& cloud, Eigen::Vector3f& base_pos)
{
	constexpr int PatternVoxel = 8;
	constexpr uint32_t MASK_DYNAMIC = 0xA0'00'00'00;
	constexpr uint32_t MASK_STATIC = 0x00'00'00'00;

	static RobotArea last_area{0.0f,0.0f,0.0f};

	RobotArea current_area{base_pos};

// std::cout<<"DEBUG: CALL FROM addPoints"<<std::endl;
	if(param.enable_bin_save || param.enable_pcd_trans){

		static size_t transmit_frame_cnt = 0;
		transmit_frame_cnt++;

		static _PollType transmit_poll{initial_pool_size_};

		size_t __cnt = 0;

		auto raw_points = global_point_poll_.ExtractMember<uint64_t>(&CompressedPoint::morton_code);
		std::sort(raw_points.begin(), raw_points.end());

		for (const auto& pt : cloud->points) {
			if (!std::isfinite(pt.x)) continue;

			if(RobotArea{pt.x, pt.y, pt.z} == current_area){
				CompressedPoint base_pt{Eigen::Vector4f( pt.x, pt.y, pt.z, pt.intensity ), 
					intensityToHeatmapRGBA( pt.intensity),MASK_DYNAMIC};
				transmit_poll.AddPoint(base_pt);
				__cnt++;
			} else if(last_area!=current_area){
				size_t neighborPointCount = RangeSearchCnt<PatternVoxel>(raw_points, 
					Eigen::Vector3f{pt.x, pt.y, pt.z}, 
					neighbor_range);

				if(neighborPointCount <= neighbor_limit_cnt){
					CompressedPoint base_pt{Eigen::Vector4f( pt.x, pt.y, pt.z, pt.intensity ), 
						intensityToHeatmapRGBA( pt.intensity), MASK_STATIC};

					global_point_poll_.AddPoint(base_pt);
					transmit_poll.AddPoint(base_pt);

					__cnt++;
				}
			}
		}
		std::cout<<"PointCloudExporter: get available point num: " << __cnt <<std::endl;

		if(transmit_frame_cnt >= transmit_frame_cnt_limit){
			transmitCloud(transmit_poll);
			transmit_poll.clear();
			transmit_frame_cnt = 0;

			last_area = current_area;
			global_point_poll_.clear();
		}
	}
	
}


inline void PointCloudExporter::transmitCloud(_PollType& transmit_poll){
	if(param.enable_pcd_trans){
		std::cout << "cloud transmiting " << std::endl;
		auto packets_ptr = transmit_poll.EncodePackets<_PacketType>(0, transmit_poll.size(), 4);

		for(auto& pkt : *packets_ptr){
			
			send_mq.enqueue(std::move(pkt));
			
		}	
		
		std::cout << "cloud transmit enqueueed " << std::endl;
	}
}


inline uint32_t PointCloudExporter::intensityToHeatmapRGBA(const float& _intensity) {
	// 将 intensity 限定到 [0, 255]
	float intensity = std::max(0.0f, std::min(255.0f, _intensity));

	int r, g, b;
	if (intensity < 128.0f) {
		r = 0;
		g = static_cast<int>(255.0f * intensity / 128.0f);
		b = 255;
	} else {
		float scaled = (intensity - 128.0f) / 127.0f;
		r = static_cast<int>(255.0f * scaled);
		g = 255;
		b = static_cast<int>(255.0f - 255.0f * scaled);
	}

	const uint8_t alpha = 255;  // 不透明
	// 按 0xRRGGBBAA 打包（大端意义上的 RGBA）
	return
		(static_cast<uint32_t>(r) << 24) |
		(static_cast<uint32_t>(g) << 16) |
		(static_cast<uint32_t>(b) <<  8) |
		static_cast<uint32_t>(alpha);
}

inline bool PointCloudExporter::saveToBinaryFile(const std::string &filename, size_t num_threads)
{
	if(param.enable_bin_save){

	std::vector<std::byte> buffer;
	auto out = zpp::bits::out(buffer,zpp::bits::endian::big{});
	out(global_point_poll_).or_throw();

	// Step 2: 打开二进制文件准备写入
	std::ofstream file(filename, std::ios::out | std::ios::binary);
	if (!file.is_open())
	{
		std::cerr << "Failed to open file: " << filename << std::endl;
		return false;
	}


		file.write(
		reinterpret_cast<const char *>(buffer.data()),
		static_cast<std::streamsize>(buffer.size())
	);

	// 4. 检查写入状态
	if (!file.good()) {
		std::cerr << "Error writing to file: " << filename << std::endl;
		file.close();
		return false;
	}

	file.close();
	}
	return true;
}


};//namespace WHU_robot