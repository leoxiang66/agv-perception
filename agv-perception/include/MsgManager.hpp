#ifndef MSGMANAGER_HPP
#define MSGMANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "utils.hpp"
#include "SizedDict.hpp"
#include <string>
#include "MsgGroup.hpp"

class MsgManager : public rclcpp::Node {
    public:
        MsgManager();
        ~MsgManager();

    private:
        SizedDict<std::string, MsgGroup> all_msgs;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_sub_;



        void compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);                                                
        void pc2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};

#endif // !MSGMANAGER_HPP
