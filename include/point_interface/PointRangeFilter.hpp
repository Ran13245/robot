// #pragma once


// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <vector>
// #include <string>
// #include <memory>
// #include <iostream>
// #include <tuple>
// #include <algorithm>
// #include <compare>

// #include "base_type.hpp"

// #include <Eigen/Eigen>

// #include <Eigen/Core>
// #include <Eigen/Dense>
// #include <pcl/io/pcd_io.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/keypoints/uniform_sampling.h>
// #include <pcl/common/concatenate.h>

// #include "algo.hpp"


// /*norm32:
// enable Life(1b) | Life (3b) | reserved(12b) | norm(16b)
// */
// namespace WHU_ROBOT{

// 	constexpr uint32_t DynamicRange_n = 10;
// 	constexpr uint32_t neighbor_range = 2;//range_meter = 2^neighbor_range * 0.01
// 	constexpr size_t neighbor_limit_cnt = 10;


// 	struct RobotArea {
// 		uint32_t x, y, z;
// 		auto operator<=>(const RobotArea&) const = default;
// 		explicit RobotArea(float pos_x, float pos_y, float pos_z){//实际位置转区块标号
// 			x = static_cast<uint32_t>(pos_x*100 + (1<<20))/(1u<<DynamicRange_n);
// 			y = static_cast<uint32_t>(pos_y*100 + (1<<20))/(1u<<DynamicRange_n);
// 			z = static_cast<uint32_t>(pos_z*100 + (1<<20))/(1u<<DynamicRange_n);
// 		}

// 		explicit RobotArea(Eigen::Vector3f& base_pos){//实际位置转区块标号
// 			x = static_cast<uint32_t>(base_pos.x()*100 + (1<<20))/(1u<<DynamicRange_n);
// 			y = static_cast<uint32_t>(base_pos.y()*100 + (1<<20))/(1u<<DynamicRange_n);
// 			z = static_cast<uint32_t>(base_pos.z()*100 + (1<<20))/(1u<<DynamicRange_n);
// 		}
// 	};

// 	class PointRangeFilter {
// 	public:	
// 		using PCDType = const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr;
// 		using PointPollType = PointPoll<CompressedPoint>;
// 		PointRangeFilter(){}

// 		size_t feedPointCloud(PCDType& input, Eigen::Vector3f& base_pos) {
// 			constexpr int PatternVoxel = 8;

// 			current_area = RobotArea{base_pos};
// 			size_t _cnt_pt = 0;

// 			//注意：合理方法是仅排序新增包之后std::merge,但是此处没有相关merge接口。出现性能问题后要进行优化
// 			auto raw_points = static_cloud.ExtractMember<uint64_t>(&CompressedPoint::morton_code);
// 			std::sort(raw_points.begin(), raw_points.end());

// 			for (const auto& pt : cloud->points) {
// 				if (!std::isfinite(pt.x)) continue;

// 				if(RobotArea{pt.x, pt.y, pt.z} == current_area){
// 					CompressedPoint base_pt{Eigen::Vector4f( pt.x, pt.y, pt.z, pt.intensity ), 
// 						intensityToHeatmapRGBA( pt.intensity),MASK_DYNAMIC};
// 					result_cloud.AddPoint(base_pt);
// 					_cnt_pt++;
// 				} else {
// 					size_t neighborPointCount = RangeSearchCnt<PatternVoxel>(raw_points, 
// 						Eigen::Vector3f{pt.x, pt.y, pt.z}, 
// 						neighbor_range);
	
// 					if(neighborPointCount <= neighbor_limit_cnt){
// 						CompressedPoint base_pt{Eigen::Vector4f( pt.x, pt.y, pt.z, pt.intensity ), intensityToHeatmapRGBA( pt.intensity), MASK_STATIC};

						
						
// 					}
// 				}

// 			}
			
// 		}

// 		PointPollType getResult(){}


// 	private:
// 		static constexpr uint32_t MASK_DYNAMIC = 0xA0'00'00'00;
// 		static constexpr uint32_t MASK_STATIC = 0x00'00'00'00;

// 		using PointType = CompressedPoint;
	
// 		PointPollType find_in_area(RobotArea area){

// 		}



// 		PointPollType getIncrease() {
// 			// std::cout << "Returning increase" << std::endl;
// 			// return {1, 2, 3};
// 		}

// 		PointPollType getReduction() {
// 			// std::cout << "Returning reduction" << std::endl;
// 			// return {0};
// 		}


// 		uint32_t intensityToHeatmapRGBA(const float& _intensity) {
// 			// 将 intensity 限定到 [0, 255]
// 			float intensity = std::max(0.0f, std::min(255.0f, _intensity));
		
// 			int r, g, b;
// 			if (intensity < 128.0f) {
// 				r = 0;
// 				g = static_cast<int>(255.0f * intensity / 128.0f);
// 				b = 255;
// 			} else {
// 				float scaled = (intensity - 128.0f) / 127.0f;
// 				r = static_cast<int>(255.0f * scaled);
// 				g = 255;
// 				b = static_cast<int>(255.0f - 255.0f * scaled);
// 			}
		
// 			const uint8_t alpha = 255;  // 不透明
// 			// 按 0xRRGGBBAA 打包（大端意义上的 RGBA）
// 			return
// 				(static_cast<uint32_t>(r) << 24) |
// 				(static_cast<uint32_t>(g) << 16) |
// 				(static_cast<uint32_t>(b) <<  8) |
// 				static_cast<uint32_t>(alpha);
// 		}

// 		PointPollType static_cloud;
// 		PointPollType result_cloud;
// 		RobotArea current_area;
// 		RobotArea last_area;
		

// 	};


// };//namespace WHU_ROBOT
