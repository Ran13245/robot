#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include <string>
#include <memory>
#include <iostream>
#include <tuple>
#include <algorithm>

#include <Eigen/Eigen>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "configParser.hpp"


#include "itc/backend/RingBuf.hpp"
#include "comm_channel.hpp"
#include "image_msg/image_msg.h"
#include "image_msg/image_sender.hpp"

class ImageProcessor {
public:
    ImageProcessor(ros::NodeHandle& nh, const param_t& _param)
        : nh_(nh),
	io_context{},
	channel(io_context, param.image_local_ip, param.image_local_port, 
		param.image_remote_ip, param.image_remote_port),
	send_mq(RingBuffer<image_msg>{10})
    {
        // 订阅图像话题
        image_sub_ = nh_.subscribe("/camera/color/image_raw", 1,
                                   &ImageProcessor::imageCallback, this);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        // 检查图像编码是否为 RGB8（即每个像素3字节）
        if (msg->encoding != "rgb8") {
            ROS_WARN("Unsupported image encoding: %s", msg->encoding.c_str());
            return;
        }

        // 检查图像数据长度是否正确
        if (msg->step != msg->width * 3) {
            ROS_WARN("Image step is not aligned with width * 3. Ignoring.");
            return;
        }

        const uint8_t* raw_data = &(msg->data[0]);
        size_t src_width = msg->width;
        size_t src_height = msg->height;

        // 计算缩放比例（整数倍降采样）
        size_t x_step = src_width / IMG_WIDTH;
        size_t y_step = src_height / IMG_HEIGHT;

        if (x_step == 0 || y_step == 0) {
            ROS_WARN("Source image is smaller than target size.");
            return;
        }

        // 清空目标结构体
        image_msg img;

        // 降采样逻辑：像素抽取（只取每个 x_step, y_step 处的像素）
        for (size_t y = 0; y < IMG_HEIGHT; ++y) {
            size_t src_y = y * y_step;
            for (size_t x = 0; x < IMG_WIDTH; ++x) {
                size_t src_x = x * x_step;
                size_t src_index = (src_y * src_width + src_x) * 3;

                // 存入 RGB 三个通道数据（按行存储）
                img.data[y][x][0] = raw_data[src_index + 0]; // R
                img.data[y][x][1] = raw_data[src_index + 1]; // G
                img.data[y][x][2] = raw_data[src_index + 2]; // B
            }
        }

        // 此处可以添加后续处理逻辑
        ROS_INFO("Image processed and stored in image_msg struct.");

        // 示例：访问某个像素 (x=100, y=50) 的 R 值：
        // uint8_t r = img.data[50][100 * 3 + 0];
	send_mq.enqueue(img);
    }

    void init(void) {
	std::cout << "ImageOutport Starting..." << std::endl;

 		channel.bind_message_queue("image_sender", ParserType::Sender, send_mq);
		
  		while((!channel.enable_sender())&&(ros::ok())){std::cout<<
			"OdomExporter: waiting enable_sender"<<std::endl;}
		
  		t = std::thread([this]() { io_context.run(); });

	std::cout << "ImageOutport Started" << std::endl;
    }

	void stop(void){
	
	std::cout << "ImageOutport Stopping..." << std::endl;
	io_context.stop();
		// while(!t.joinable()) {std::cout<<"CarCmd2ROSHandler: waiting thread joinable"<<std::endl;}
		t.join();
	std::cout << "ImageOutport Stopped" << std::endl;
	
}

private:
	ros::NodeHandle& nh_;
	ros::Subscriber image_sub_;

	param_t param;


	asio::io_context io_context;
	std::thread t;
	CommChannel<ChannelMode::UDP, ImageSender> channel;
	MsgQueue send_mq;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_processor_node");
    ros::NodeHandle nh;

    std::string config_file = "/workspace/src/robot/config/config.yaml";
	WHU_ROBOT::param_t params(config_file);

    ImageProcessor processor(nh,params);

    processor.init();
    ros::spin();
    processor.stop();


    return 0;
}