#include <ros/ros.h>
#include "SLAMROSHandler.hpp"
#include "configParser.hpp"

int main(int argc, char** argv)
{
	std::cout << "WHU Robot Communication Node Starting..." << std::endl;

    ros::init(argc, argv, "WHUrobot_communicate_node");
    ros::NodeHandle nh;

    std::string config_file = "/workspace/src/robot/config/config.yaml";
    WHU_ROBOT::param_t params(config_file);

    WHU_ROBOT::SLAMROSHandler slam_handler(params, nh);
    slam_handler.init();

    ros::spin();

    slam_handler.stop();

    return 0;
}