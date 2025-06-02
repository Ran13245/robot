#include <ros/ros.h>
#include "SLAMROSHandler.hpp"
#include "configParser.hpp"
#include "CarCmd2ROS.hpp"

int main(int argc, char** argv)
{
	std::cout << "WHU Robot Communication Node Starting..." << std::endl;

    ros::init(argc, argv, "WHUrobot_communicate_node");
    ros::NodeHandle nh;

    std::string config_file = "/workspace/src/robot/config/config.yaml";
    WHU_ROBOT::param_t params(config_file);

    WHU_ROBOT::SLAMROSHandler slam_handler(params, nh);
    slam_handler.init();


    WHU_ROBOT::CarCmd2ROSHandler cmd_handler(params, nh);
    cmd_handler.init();

    // ros::spin();

    ros::Rate rate(100.0);

    while(ros::ok()){
        cmd_handler.exec();
        ros::spinOnce();
        rate.sleep();
    }

    slam_handler.stop();
    cmd_handler.stop();

    return 0;
}