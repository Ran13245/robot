#pragma once

#include "CarControlBase.hpp"
#include "configParser.hpp"
#include <iostream>
#include "pid.hpp"
#include <cmath>
#include <chrono>

namespace WHU_ROBOT{

	class SimpleCtrl final : public CarControllerBase{
	public:
		SimpleCtrl(const ctl_param_t& _param):
			param{_param},
			pid_vx{param.p_vx, 0, param.d_vx, -param.max_v, param.max_v},
			pid_vy{param.p_vy, 0, param.d_vy, -param.max_v, param.max_v},
			pid_wz{param.p_wz, 0, param.d_wz, -param.max_w, param.max_w}
		{
			lastTime_ = Clock::now();

		}

		~SimpleCtrl() = default;

		void feedTarget(const CarState& targetState) override {
			__flag_get_target = 1;
			target_pos = targetState.base_pos;
			target_pos.z() = 0.0;
			target_yaw = getYawDegrees(targetState.base_quat);
		}

		void feedCurrent(const CarState& currentState) override {
			__flag_get_current = 1;
			current_pos = currentState.base_pos;
			current_pos.z() = 0.0;
			current_yaw = getYawDegrees(currentState.base_quat);
		}

		CarState computeControlOnce(void) override {
			Clock::time_point now = Clock::now();
			std::chrono::duration<double> delta = now - lastTime_;
			double dt = delta.count();
			lastTime_ = now;

			if(__flag_get_target && __flag_get_current){
				if(euclideanDistance(current_pos, target_pos) >= param.brake_distance){
					std::cout<<"\033[31m" << "brake_distance:" << "\033[0m" << std::endl;
					std::cout<<"current position: "<< current_pos.transpose() <<std::endl;
					std::cout<<"target position: "<< target_pos.transpose() <<std::endl;
					pid_wz.reset();
					pid_vx.reset();
					pid_vy.reset();
					return dataCapsu(1, 0,0,0);
				}

				if(std::abs(current_yaw - target_yaw) >= param.delta_yaw){
					//4T4D
					float wz = pid_wz.update(target_yaw, current_yaw, dt);
					return dataCapsu(6, 0,0,wz);
				} else if(euclideanDistance(current_pos, target_pos) >= param.delta_distance){
					//parallel
					float vx = pid_vx.update(target_pos.x(), current_pos.x(), dt);
					float vy = pid_vy.update(target_pos.y(), current_pos.y(), dt);
					return dataCapsu(8, vx,vy,0);
				}else {
					//free
					pid_wz.reset();
					pid_vx.reset();
					pid_vy.reset();
					return dataCapsu(2, 0,0,0);
				}
			}

			std::cout<<"target or current never got"<<std::endl;
			return dataCapsu(1, 0,0,0);
		};
	private:
		//------------------------const-----------
		static constexpr float __vx_when_spin = 0.1;
		//-----------------------obj-------------
		ctl_param_t param;
		PID pid_vx;
		PID pid_vy;
		PID pid_wz;

		//-----------------------state-------------
		Eigen::Vector3f current_pos;
		float current_yaw;
		Eigen::Vector3f target_pos;
		float target_yaw;

		//----------------------utils--------------
		bool __flag_get_target = 0;
		bool __flag_get_current = 0;
		using Clock = std::chrono::steady_clock;
		Clock::time_point lastTime_;
		
		//---------------------fcn-------------------
		CarState dataCapsu(uint8_t mode, float vx, float vy, float wz){
			static uint32_t __cnt = 0;
			CarState rtn;
			if(mode == 6){//4T4D
				rtn.mode = 6;
				
				rtn.base_lin_vel.setZero();
				rtn.base_lin_vel.x() = __vx_when_spin;

				rtn.base_ang_vel.setZero();
				rtn.base_ang_vel.z() = wz;
			} else if(mode == 8){//parallel
				rtn.mode = 8;
				
				rtn.base_lin_vel.setZero();
				rtn.base_lin_vel.x() = vx;
				rtn.base_lin_vel.y() = vy;

				rtn.base_ang_vel.setZero();
			} else if (mode == 2){//free
				rtn.mode = 2;
				rtn.base_lin_vel.setZero();
				rtn.base_ang_vel.setZero();
			} else {//stop
				rtn.mode = 1;
				rtn.base_lin_vel.setZero();
				rtn.base_ang_vel.setZero();
			}
			return rtn;
		}
	};

};//namespace WHU_ROBOT

