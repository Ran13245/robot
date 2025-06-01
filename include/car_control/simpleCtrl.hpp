#pragma once

#include "CarControlBase.hpp"
#include "configParser.hpp"
#include <iostream>

namespace WHU_ROBOT{

	class SimpleCtrl : public CarControllerBase{
	public:
		SimpleCtrl(const ctl_param_t& _param):param{_param}{

		}

		~SimpleCtrl() = default;

		void feedTarget(const CarState& targetState) override {

		}

		void feedCurrent(const CarState& currentState) override {

		}

		CarState computeControlOnce(void) override {
			std::cout<<"compute in SimpleCtrl"<<std::endl;
			return CarState{};
		};
	private:
		ctl_param_t param;
	};

};//namespace WHU_ROBOT

