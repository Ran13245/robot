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

// 辅助把宏值转成字符串
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

// 打印 GCC 完整版本信息
#ifdef __VERSION__
#  pragma message("Compiler full version (__VERSION__): " __VERSION__)
#endif

// 打印各个版本宏
#pragma message("GCC major version: "   STR(__GNUC__))
#pragma message("GCC minor version: "   STR(__GNUC_MINOR__))
#pragma message("GCC patchlevel: "      STR(__GNUC_PATCHLEVEL__))

// 打印 C++ 标准模式
#pragma message("__cplusplus macro: "   STR(__cplusplus))

// 打印 bit_cast 特性宏（如果存在）
#ifdef __cpp_lib_bit_cast
#  pragma message("__cpp_lib_bit_cast: " STR(__cpp_lib_bit_cast))
#else
#  pragma message("__cpp_lib_bit_cast is not defined")
#endif
// 检查是否用的是 GCC，并且主版本 >= 10
#if !defined(__GNUC__) || (__GNUC__ < 10)
  #error "This library requires GCC version 10 or newer. Please upgrade your compiler."
#endif

// 检查是否启用了 C++20（包括 GNU 扩展模式 gnu++20）
// __cplusplus 在 -std=gnu++20 下会>=202002L
#if (__cplusplus < 202002L)
  #error "This library requires C++20. Please compile with -std=gnu++20 or higher."
#endif

// 可选：进一步检查 std::bit_cast 支持（C++20 标准库的一部分）
#if !defined(__cpp_lib_bit_cast) || (__cpp_lib_bit_cast < 201806L)
  #error "std::bit_cast not available: ensure you're using a C++20 standard library implementation."
#endif


namespace WHU_ROBOT{

  static constexpr size_t PACKET_MTU = 1400; // 可调整
      // 使用 base_type 定义的类型

  using PacketType = PointPacket<CompressedPoint, PACKET_MTU>;
  using PointPollType = PointPoll<CompressedPoint>;

class PointCloudExporter {
public:

    /**
     * @brief 构造函数
     * @param initial_pool_size 初始点池大小
     */
    explicit PointCloudExporter(size_t initial_pool_size = 1024);

    /**
     * @brief 插入 ROS 点云数据到内部缓冲区（转换为 BasePointf）
     * @param cloud 输入点云（PointXYZI 类型）
     */
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
    pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr cloudFilter(const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr& cloud, float resolution = filter_resolution);

    /*-----------------------------objs-----------------------------------------*/
    PointPollType point_poll_;
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

    // // Step 3: 写入每个 packet 的内容到文件中
    // // for (const auto &packet_span : *packets)
    // // {
    //     file.write(reinterpret_cast<const char *>(packet_span.data()), packet_span.size());
    //     if (!file.good())
    //     {
    //         std::cerr << "Error writing to file: " << filename << std::endl;
    //         file.close();
    //         return false;
    //     }
    // // }

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
PointCloudExporter::cloudFilter(const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr& cloud,
                                float resolution)
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