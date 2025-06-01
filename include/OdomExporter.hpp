#pragma once

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


#include "TaskBase.hpp"
#include "CtrlParser.hpp"
#include "RingBuf.hpp"
#include "UdpChannel.hpp"

#include "TransmitBuffer.hpp"

namespace Schedule{

	using namespace IO_Comm;
	using namespace Protocol;

	class OdomTransmitTask : public TaskBase
	{
	public:
		using channelType = UdpChannel<SocketParser<FullbodyState, FullbodyState>>;
		OdomTransmitTask(std::string remote_ip, int remote_port, std::string local_ip, int local_port):
			remote_ip_(remote_ip),
			remote_port_(remote_port),
			local_ip_(local_ip),
			local_port_(local_port)
		{}

		~OdomTransmitTask(){}
	protected:
		void task() override	
		{
			channelType channel(io_context,
					local_ip_, local_port_,
					remote_ip_, remote_port_);
			auto odom_msg_queue = quiry_msg_queue("OdomMsg");
			auto typed_ptr = std::static_pointer_cast<RingBuffer<std::shared_ptr<FullbodyState>>>(odom_msg_queue->getRawBuffer());
			channel.register_sender_buffer(typed_ptr);
			channel.enable_sender();
			io_context.run();
		
		}

		void stop() override
		{
			io_context.stop();
			TaskBase::stop();
			std::cout << "OdomTransmitTask Stopped" << std::endl;
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



class OdomExporter {
public:
	explicit OdomExporter(TaskBase& transmit_task,bool enable_odom_trans = false);

	void init(void);
	void stop(void);

	void addOdom(const uint64_t& time, const Eigen::Vector3f& base_pos, const Eigen::Quaternionf& base_quat);

private:

	/*-----------------------------consts-----------------------------------------*/
	static constexpr size_t trans_buffer_size = 5;

	/*-----------------------------functions-----------------------------------------*/
	void transmitOdom(const FullbodyState& data);
	/*-----------------------------objs-----------------------------------------*/

	TransmitBuffer<FullbodyState> trans_buffer;
	TaskBase* transmit_task_ptr_ = nullptr; 
	IMsgQueue* transmit_odom_msg_queue_ptr_ = nullptr; 

	bool enable_odom_trans_; 
};

inline void OdomExporter::addOdom(const uint64_t& time, 
	const Eigen::Vector3f& base_pos, const Eigen::Quaternionf& base_quat)
{
	static uint32_t __cnt = 0;
	// 新建 FullbodyState 对象
	FullbodyState state;

	// 1. 填入传参
	state.time = time;
	state.base_pos = base_pos;
	state.base_quat = base_quat;

	// 2. mask、cnt 及输入（left_input、right_input）都置 0
	state.mask = 0;
	state.cnt = __cnt++;
	state.left_input = 0.0f;
	state.right_input = 0.0f;

	// 3. 其余所有位置、旋转、速度都置零（单位四元数）
	state.left_hand_pos.setZero();
	state.right_hand_pos.setZero();
	state.left_hand_quat = Eigen::Quaternionf::Identity();
	state.right_hand_quat = Eigen::Quaternionf::Identity();

	state.base_lin_vel.setZero();
	state.base_ang_vel.setZero();
	state.left_hand_lin_vel.setZero();
	state.left_hand_ang_vel.setZero();
	state.right_hand_lin_vel.setZero();
	state.right_hand_ang_vel.setZero();


	transmitOdom(state);
}
 


inline void OdomExporter::transmitOdom(const FullbodyState& data){

	std::shared_ptr<FullbodyState> data_ptr = trans_buffer.push_T_rtn_shared(data);

	std::cout << "odom transmitting" << std::endl;
	if(data_ptr && transmit_task_ptr_){
		transmit_odom_msg_queue_ptr_->enqueue(&data_ptr);
	}
}


inline OdomExporter::OdomExporter(TaskBase& transmit_task,bool enable_odom_trans):
	transmit_task_ptr_{&transmit_task},
	enable_odom_trans_{enable_odom_trans},
	trans_buffer{trans_buffer_size}
{

}

inline void OdomExporter::init(void){
	std::cout << "OdomExporter Starting..." << std::endl;

	if(enable_odom_trans_ && transmit_task_ptr_){

		std::cout << "odom_trans enabled" << std::endl;
		transmit_odom_msg_queue_ptr_ = new MsgQueueImpl<std::shared_ptr<FullbodyState>>(1024);
		if(transmit_odom_msg_queue_ptr_ != nullptr){
			transmit_task_ptr_->bind_msg_queue(std::string("OdomMsg"), transmit_odom_msg_queue_ptr_);
		} else{
			std::cout << "transmit_odom_msg_queue_ptr_ nullptr" << std::endl;
		}

		std::cout << "odom transmit_task Starting..." << std::endl;
		transmit_task_ptr_->start();
	} else {
		std::cout << "odom_trans not enable" << std::endl;
		transmit_odom_msg_queue_ptr_ = nullptr;
		transmit_task_ptr_ = nullptr;
	}
	std::cout << "PointCloudExporter Started" << std::endl;
}

inline void OdomExporter::stop(void){
	std::cout << "PointCloudExporter Stopping..." << std::endl;
	if(transmit_task_ptr_)transmit_task_ptr_->stop();
	if(transmit_odom_msg_queue_ptr_) delete transmit_odom_msg_queue_ptr_;
}


} // namespace WHU_ROBOT