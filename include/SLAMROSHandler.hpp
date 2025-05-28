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

namespace WHU_ROBOT {

class SLAMROSHandler {
public:
	SLAMROSHandler(const param_t& _param, const ros::NodeHandle& _nh): nh{_nh}, param{_param} {
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

	void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
	void odomCallback(const nav_msgs::OdometryConstPtr& msg);
};

inline void SLAMROSHandler::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);


	pcl::fromROSMsg(*msg, *cloud);

	ROS_INFO("Received point cloud with %zu points", cloud->size());

	exporter.addPoints(cloud);
}

inline void SLAMROSHandler::odomCallback(const nav_msgs::OdometryConstPtr& msg){
}

}// namespace WHU_ROBOT