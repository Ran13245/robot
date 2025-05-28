#pragma once

#include <yaml-cpp/yaml.h>
#include <string>

namespace WHU_ROBOT {

class param_t {
public:
	std::string cloud_topic;
	std::string odom_topic;

	std::string cloud_export_path;
	
	std::string remote_ip;
	std::string remote_port;
	std::string local_ip;
	std::string local_port;

	param_t(const std::string& filename){
		try {
			YAML::Node config = YAML::LoadFile(filename);
			cloud_topic = config["cloud_topic"].as<std::string>();
			odom_topic = config["odom_topic"].as<std::string>();
			cloud_export_path = config["cloud_export_path"].as<std::string>();
			remote_ip = config["remote_ip"].as<std::string>();
			remote_port = config["remote_port"].as<std::string>();
			local_ip = config["local_ip"].as<std::string>();
			local_port = config["local_port"].as<std::string>();
		} catch (const YAML::Exception& e) {
			throw std::runtime_error("Error parsing configuration file: " + std::string(e.what()));
		}
	}
};

}//namespace WHU_robot