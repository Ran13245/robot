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

#include <Eigen/Eigen>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/common/concatenate.h>
#include <eigen_conversions/eigen_msg.h>

#include "TaskBase.hpp"
#include "DataParser.hpp"
#include "RingBuf.hpp"
#include "UdpChannel.hpp"

static constexpr size_t PACKET_MTU = 1400; // 可调整
using _PacketType = PointPacket<CompressedPoint, PACKET_MTU>;
using _SenderType = std::span<std::byte>;

namespace Protocol{


	template <>
	struct SocketParser<_SenderType, DummyReceiver>
	{
		using BinBuffer = std::span<std::byte>;
		using SenderType = _SenderType;
		using ReceiverType = DummyReceiver;
		static constexpr size_t SenderMsgSize = _PacketType::TotalByte;
		static constexpr size_t ReceiverMsgSize = sizeof(DummyReceiver);
		static constexpr void Encode(const SenderType &data, BinBuffer &buffer) {};
		static constexpr void Decode(const BinBuffer &buffer, ReceiverType &data) {};
	};
};



namespace Schedule{

	using namespace IO_Comm;
	using namespace Protocol;

	class PCDTransmitTask : public TaskBase
	{
	public:
		using channelType = UdpChannel<SocketParser<_SenderType, DummyReceiver>>;
		PCDTransmitTask(std::string remote_ip, int remote_port, std::string local_ip, int local_port):
			remote_ip_(remote_ip),
			remote_port_(remote_port),
			local_ip_(local_ip),
			local_port_(local_port)
		{}

		~PCDTransmitTask(){}
	protected:
		void task() override	
		{
			// std::cout << "333" << std::endl;
        //     std::cout.flush();

			channelType channel(io_context,
					local_ip_, local_port_,
					remote_ip_, remote_port_);
			auto cld_msg_queue = quiry_msg_queue("PoindCloudPacket");
			auto typed_ptr = std::static_pointer_cast<RingBuffer<std::shared_ptr<_SenderType>>>(cld_msg_queue->getRawBuffer());
			channel.register_sender_buffer(typed_ptr);
			channel.enable_sender();
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



class PointCloudExporter {
public:

	explicit PointCloudExporter(size_t initial_pool_size = 1024);

	explicit PointCloudExporter(TaskBase& transmit_task,bool enable_pcd_trans = false, bool enable_bin_save = true, size_t initial_pool_size = 1024);

	void init(void);
	void stop(void);

	void addPoints(const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr& _cloud);

	bool saveToBinaryFile(const std::string &filename, size_t num_threads);

private:

	/*-----------------------------consts-----------------------------------------*/
	// static constexpr float filter_resolution = 0.05f; //!!!abandon!!!
	// static constexpr uint8_t filter_type = 0;//type 0 none; type 1 VoxelGrid; type 2 UniformSampling//!!!abandon!!!
	// static constexpr size_t transmit_poll_cnt_limit = 10; // transmit_poll到达此次数后发送//!!!abandon!!!
	// static constexpr size_t pcd_sample_cnt_limit = 1; //!!!abandon!!!
	static constexpr size_t pcd_handle_cnt_limit = 10; //累积若干次后处理一次

	/*-----------------------------functions-----------------------------------------*/
	uint32_t intensityToHeatmapRGBA(const float& _intensity);
	pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr cloudFilter(const 
		pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr& cloud, float resolution = filter_resolution);

	void transmitCloud(void);
	/*-----------------------------objs-----------------------------------------*/
	PointPoll<CompressedPoint> point_poll_;
	std::shared_ptr<PointPoll<CompressedPoint>> transmit_poll_ptr_;
	TaskBase* transmit_task_ptr_ = nullptr; // 传输任务
	IMsgQueue* transmit_poll_msg_queue_ptr_ = nullptr; // 传输任务的消息队列

	bool enable_bin_save_; 
	bool enable_pcd_trans_; 

	size_t transmit_poll_cnt_ = 0;// transmit_poll使用计数

	size_t initial_pool_size_; // 初始池大小
};

//TODO: 新函数通过传参进行发送
inline void PointCloudExporter::transmitCloud(void){//abandon
	// std::cout << "cloud transmiting " << std::endl;
	// if(transmit_task_ptr_){
	// 	auto packets_ptr = transmit_poll_ptr_->EncodePackets<_PacketType>(0, transmit_poll_ptr_->size(), 4);
	// 	for(auto& pkt : *packets_ptr){
	// 		std::shared_ptr<_SenderType> sender_ptr = std::make_shared<_SenderType>(pkt);
	// 		if(transmit_poll_msg_queue_ptr_){
	// 			transmit_poll_msg_queue_ptr_->enqueue(&sender_ptr);
	// 		} else {
	// 			std::cerr << "Error: transmit_poll_msg_queue_ptr_ is null!" << std::endl;
	// 		}
	// 	}	
	// }
	// std::cout << "cloud transmitted " << std::endl;
}

inline PointCloudExporter::PointCloudExporter(size_t initial_pool_size): point_poll_{initial_pool_size}
{
	initial_pool_size_ = initial_pool_size;
	transmit_poll_ptr_ = std::make_shared<PointPoll<CompressedPoint>>(initial_pool_size);
}

inline PointCloudExporter::PointCloudExporter(TaskBase& transmit_task ,bool enable_pcd_trans,bool enable_bin_save, size_t initial_pool_size):
	transmit_task_ptr_{&transmit_task},
	enable_bin_save_{enable_bin_save},
	enable_pcd_trans_{enable_pcd_trans},
	point_poll_{initial_pool_size}
{
	initial_pool_size_ = initial_pool_size;
	transmit_poll_ptr_ = std::make_shared<PointPoll<CompressedPoint>>(initial_pool_size);

}


inline void PointCloudExporter::init(void){
	std::cout << "PointCloudExporter Starting..." << std::endl;

	if(enable_pcd_trans_ && transmit_task_ptr_){

		std::cout << "pcd_trans enabled" << std::endl;
		transmit_poll_msg_queue_ptr_ = new MsgQueueImpl<std::shared_ptr<_SenderType>>(1024);
		if(transmit_poll_msg_queue_ptr_ != nullptr){
			transmit_task_ptr_->bind_msg_queue(std::string("PoindCloudPacket"), transmit_poll_msg_queue_ptr_);
		} else{
			std::cout << "transmit_poll_msg_queue_ptr_ nullptr" << std::endl;
		}

		std::cout << "transmit_task Starting..." << std::endl;
		transmit_task_ptr_->start();
	} else {
		std::cout << "pcd_trans not enable" << std::endl;
		transmit_poll_msg_queue_ptr_ = nullptr;
		transmit_task_ptr_ = nullptr;
	}
	std::cout << "PointCloudExporter Started" << std::endl;
}

inline void PointCloudExporter::stop(void){
	std::cout << "PointCloudExporter Stopping..." << std::endl;
	if(transmit_task_ptr_)transmit_task_ptr_->stop();
	if(transmit_poll_msg_queue_ptr_) delete transmit_poll_msg_queue_ptr_;
}

// inline void PointCloudExporter::addPoints(const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr& _cloud)
// {
// 	static size_t pcd_sample_cnt = 0; // 初始池大小

// // bool flag = 0;

// // /*----------------debug-------------------------------- */
// //     pcl::PointXYZINormal min_pt, max_pt;
// //     pcl::getMinMax3D(*_cloud, min_pt, max_pt);

// //     std::cout << "\033[32m" << "X min: " << min_pt.x
// //               << ", X max: " << max_pt.x << "\033[0m"
// //               << std::endl;
// // 	std::cout << "\033[32m" << "Y min: " << min_pt.y
// // 		<< ", Y max: " << max_pt.y << "\033[0m"
// // 		<< std::endl;
// // 	std::cout << "\033[32m" << "Z min: " << min_pt.z
// // 		<< ", Z max: " << max_pt.z << "\033[0m"
// // 		<< std::endl;
// // /*-------------debug-end-------------------------------*/

// 	pcd_sample_cnt++;
// 	pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr cloud = cloudFilter(_cloud);

// 	std::cout << "\033[32m" << "filter: "<< _cloud->points.size() 
// 		<< " -> "<< cloud->points.size() <<" get rid of "
// 		<<_cloud->points.size() - cloud->points.size() <<"\033[0m" << std::endl;
	
// 	for (const auto& pt : cloud->points) {
// 		CompressedPoint base_pt{Eigen::Vector4f( pt.x, pt.y, pt.z, pt.intensity ), intensityToHeatmapRGBA( pt.intensity)};

// // if(!flag){
// // 	std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << pt.x<<" "<< pt.y<<" "<< pt.z<<"!!!!!!!!!!!!!!!!!!!!"<<std::endl;
// // 	flag = 1;
// // }
// // 
// 		if(enable_bin_save_)point_poll_.AddPoint(base_pt);

// 		if(transmit_poll_ptr_ && (pcd_sample_cnt >= pcd_sample_cnt_limit))transmit_poll_ptr_->AddPoint(base_pt);
// 	}

// 	if(transmit_poll_ptr_ && (pcd_sample_cnt >= pcd_sample_cnt_limit)){
// 		std::cout << "PointCloudTransmit: " << cloud->size() << " points added to transmit_poll." << std::endl;
// 		transmit_poll_cnt_++;
// 		if(transmit_poll_cnt_ >= transmit_poll_cnt_limit){
// 			transmit_poll_cnt_ = 0;
// 			transmitCloud();
// 			// transmit_poll_ptr_ = std::make_shared<PointPoll<CompressedPoint>>(initial_pool_size_);
// 			transmit_poll_ptr_->clear();
// 		}
// 	}

// 	if(pcd_sample_cnt >= pcd_sample_cnt_limit)pcd_sample_cnt = 0;
// }

/**	
*	TODO，需求: 
* 	1, 取消采样模式，将所有帧全部压入，每次发送前进行下采样
	2, 下采样不使用PCL库而是直接对pack处理，主要是去重
	3, 通过algo.hpp，遍历点获取其周围点，周围点多于一定数量视为无效点
	4, 只发送增量
	
	例如编码精度1cm，近邻范围5cm，10帧一次发送，分为全局poll和发送poll
	将10帧所有点全部压入->1cm精度意义下的去重->计算纯增量点送入发送poll
	->遍历发送poll,若增量点在全局poll的5cm范围内有大量点则从发送poll去除
	->发送
	问题：
	5cm有大量点的话，是否还存入全局poll；
	复制与内存分配耗时问题
*/	
inline void PointCloudExporter::addPoints(const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr& _cloud)
{

	static size_t pcd_handle_cnt = 0;
	pcd_handle_cnt++;

	if(pcd_handle_cnt >= pcd_handle_cnt_limit){

		pcd_handle_cnt = 0;
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
	if(enable_bin_save_){
	// Step 1: 序列化为数据包
	// auto packets = this->serializeToPackets(num_threads);
	std::vector<std::byte> buffer;
	auto out = zpp::bits::out(buffer,zpp::bits::endian::big{});
	out(point_poll_).or_throw();

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


inline pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr
PointCloudExporter::cloudFilter(const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr& cloud,	float resolution)
{

	if constexpr(filter_type == 0){//none
		return cloud;
	} else if constexpr(filter_type == 1){//VoxelGrid
		pcl::VoxelGrid<pcl::PointXYZINormal> voxel_grid;
		voxel_grid.setInputCloud(cloud);
		voxel_grid.setLeafSize(resolution, resolution, resolution);

		pcl::PointCloud<pcl::PointXYZINormal>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
		voxel_grid.filter(*filtered_cloud);
		return filtered_cloud;
	} else if constexpr(filter_type == 2){//UniformSampling
		pcl::UniformSampling<pcl::PointXYZINormal> uniform_sampling;
		uniform_sampling.setInputCloud(cloud);
		uniform_sampling.setRadiusSearch(resolution);

		pcl::PointCloud<pcl::PointXYZINormal>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
		uniform_sampling.filter(*filtered_cloud);
		return filtered_cloud;
	} else {
		std::cerr << "Invalid filter type. Using VoxelGrid by default." << std::endl;
		return cloud;
	}
}


};//namespace WHU_robot