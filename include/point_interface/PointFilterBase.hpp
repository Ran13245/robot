// #pragma once

// #include <concepts>
// #include <utility>
// #include <vector>

// namespace WHU_ROBOT{

// 	template<typename Derived, typename PointCloudT, typename... Args>
// 	concept FilterImplRequirements = requires(Derived& d, const PointCloudT& input, Args... args) {
// 		{ d.feedPointCloud(input, args...) } -> std::same_as<size_t>;
// 		{ d.getIncrease(args...) } -> std::same_as<PointCloudT>;
// 		{ d.getReduction(args...) } -> std::same_as<PointCloudT>;
// 	};

// 	template<typename Derived, typename PointCloudT>
// 	class FilterInterface {
// 	public:
// 		using PointCloudType = PointCloudT;

// 		template<typename... Args>
// 		requires FilterImplRequirements<Derived, PointCloudT, Args...>
// 		size_t feedPointCloud(const PointCloudT& input, Args&&... args) {
// 			return static_cast<Derived*>(this)->feedPointCloud(input, std::forward<Args>(args)...);
// 		}

// 		template<typename... Args>
// 		requires FilterImplRequirements<Derived, PointCloudT, Args...>
// 		PointCloudT getIncrease(Args&&... args) {
// 			return static_cast<Derived*>(this)->getIncrease(std::forward<Args>(args)...);
// 		}

// 		template<typename... Args>
// 		requires FilterImplRequirements<Derived, PointCloudT, Args...>
// 		PointCloudT getReduction(Args&&... args) {
// 			return static_cast<Derived*>(this)->getReduction(std::forward<Args>(args)...);
// 		}
// 	};

// };//namespace WHU_ROBOT
