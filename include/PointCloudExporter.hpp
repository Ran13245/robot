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
#include <eigen_conversions/eigen_msg.h>



namespace WHU_ROBOT{

	// static constexpr size_t PACKET_MTU = 1400; // 可调整

	// using PacketType = PointPacket<CompressedPoint, PACKET_MTU>;

class PointCloudExporter {
public:

	explicit PointCloudExporter(size_t initial_pool_size = 1024);

	void addPoints(const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr& _cloud);

	bool saveToBinaryFile(const std::string &filename, size_t num_threads);

private:
	/**
		* @brief 将点云序列化为多个二进制数据包（适合网络传输）
		* @param num_threads 多线程编码数量
		* @return shared_ptr<vector<span<byte>>> 数据包集合
		*/
	// std::shared_ptr<std::vector<std::span<std::byte>>> serializeToPackets(size_t num_threads = 1);

	/*-----------------------------consts-----------------------------------------*/
	static constexpr float filter_resolution = 0.05f; 
	static constexpr uint8_t filter_type = 2;//type 0 none; type 1 VoxelGrid; type 2 UniformSampling

	/*-----------------------------functions-----------------------------------------*/
	uint32_t intensityToHeatmapRGBA(const float& _intensity);
	pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr cloudFilter(const 
		pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr& cloud, float resolution = filter_resolution);

	/*-----------------------------objs-----------------------------------------*/
	PointPoll<CompressedPoint> point_poll_;
};


inline PointCloudExporter::PointCloudExporter(size_t initial_pool_size): point_poll_{initial_pool_size}
{
		
}

inline void PointCloudExporter::addPoints(const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr& _cloud)
{

	pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr cloud = cloudFilter(_cloud);
	for (const auto& pt : cloud->points) {
		CompressedPoint base_pt{Eigen::Vector4f( pt.x, pt.y, pt.z, pt.intensity ), intensityToHeatmapRGBA( pt.intensity)};
		point_poll_.AddPoint(base_pt);

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


}//namespace WHU_robot