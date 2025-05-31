#pragma once

#define PCL_NO_PRECOMPILE


#include <ros/ros.h>
#include <Eigen/Eigen>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#include "configParser.hpp"
#include "PointCloudExporter.hpp"

#include <chrono>

namespace WHU_ROBOT {

class SLAMROSHandler {
public:
	SLAMROSHandler(const param_t& _param, const ros::NodeHandle& _nh): 
		nh{_nh}, 
		param{_param},
		transmit_task{param.remote_ip, param.remote_port, 
				param.local_ip, param.local_port},
		exporter{transmit_task,param.enable_pcd_trans, param.enable_bin_save, 1024}
	{
		cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(param.cloud_topic, 10, &SLAMROSHandler::cloudCallback, this);
		odom_sub = nh.subscribe<nav_msgs::Odometry>(param.odom_topic, 10, &SLAMROSHandler::odomCallback, this);
	}

	~SLAMROSHandler() {
		ROS_INFO("Saving point cloud data to binary file: %s", param.cloud_export_path.c_str());
		exporter.saveToBinaryFile(param.cloud_export_path,4);
	}

private:
	ros::NodeHandle nh;
	param_t param;
	ros::Subscriber cloud_sub;
	ros::Subscriber odom_sub;
	PointCloudExporter exporter;
	PCDTransmitTask transmit_task;

	void init(void);
	void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
	void odomCallback(const nav_msgs::OdometryConstPtr& msg);
};

inline void SLAMROSHandler::init(void){
	
}

inline void SLAMROSHandler::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);


	pcl::fromROSMsg(*msg, *cloud);

	ROS_INFO("Received point cloud with %zu points", cloud->size());
	
	auto t0 = std::chrono::steady_clock::now();

	exporter.addPoints(cloud);
	
	auto t1 = std::chrono::steady_clock::now();
	ROS_INFO("Added points to exporter in %.3f ms", std::chrono::duration<double, std::milli>(t1 - t0).count());
}

inline void SLAMROSHandler::odomCallback(const nav_msgs::OdometryConstPtr& msg){
}

}// namespace WHU_ROBOT