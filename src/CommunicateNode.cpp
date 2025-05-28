#include <ros/ros.h>
#include "SLAMROSHandler.hpp"
#include "configParser.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "WHUrobot_communicate_node");
    ros::NodeHandle nh;

    // 读取配置文件
    std::string config_file = "/workspace/src/robot/config/config.yaml"; // 替换为实际的配置文件路径
    WHU_ROBOT::param_t params(config_file);

    // 创建 SLAMROSHandler 实例
    WHU_ROBOT::SLAMROSHandler slam_handler(params, nh);

    // 进入 ROS 循环
    ros::spin();

    return 0;
}