#pragma once

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace WHU_ROBOT{

	/*
	mode:
	odom:
		0 - no cnt, no vel
		1 - no cnt, has vel
		2- has cnt , no vel
		3 - both has
	target:
		same to odom
	control:
		0 - disable
		1 - stop
		2 - free
		6 - 4T4D
		8 - parallel
	*/
	struct CarState{
		uint8_t mode;
		uint32_t cnt;
		uint64_t time;
		Eigen::Vector3f base_pos;
		Eigen::Quaternionf base_quat;
		Eigen::Vector3f base_lin_vel;//m/s, 0.001m/s
		Eigen::Vector3f base_ang_vel;//degree/s, 0.01deg/s
	};

	class CarControllerBase{
	public:
		CarControllerBase() = default;
		virtual ~CarControllerBase() = 0;

		virtual void feedTarget(const CarState& targetState) = 0;
		virtual void feedCurrent(const CarState& currentState) = 0;
		virtual CarState computeControlOnce(void) = 0;
	};

	inline CarControllerBase::~CarControllerBase() { }

};//namespace WHU_ROBOT