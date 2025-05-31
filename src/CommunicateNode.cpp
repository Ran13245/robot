#include <ros/ros.h>
#include "SLAMROSHandler.hpp"
#include "configParser.hpp"

int main(int argc, char** argv)
{
	std::cout << "WHU Robot Communication Node Starting..." << std::endl;

    ros::init(argc, argv, "WHUrobot_communicate_node");
    ros::NodeHandle nh;

    // // 读取配置文件
    std::string config_file = "/workspace/src/robot/config/config.yaml"; // 替换为实际的配置文件路径
    WHU_ROBOT::param_t params(config_file);

    // 创建 SLAMROSHandler 实例
    WHU_ROBOT::SLAMROSHandler slam_handler(params, nh);

    // 进入 ROS 循环
    ros::spin();

// Schedule::PCDTransmitTask testTask{"127.0.0.1", 12345, "127.0.0.1", 12345};
// IMsgQueue *msg_queue = new MsgQueueImpl<std::shared_ptr<std::span<std::byte>>>(100);
// testTask.bind_msg_queue("PoindCloudPacket", msg_queue);
// testTask.start();
// std::cout << "33344" << std::endl;
//             std::cout.flush();


// // std::shared_ptr<std::span<std::byte>> ptr = std::make_shared<std::span<std::byte>>();
// // msg_queue->enqueue(&ptr);
// testTask.stop();


    return 0;
}