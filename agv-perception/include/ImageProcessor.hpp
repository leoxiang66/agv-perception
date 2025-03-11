#ifndef IMAGEPROCESSOR_HPP
#define IMAGEPROCESSOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "utils.hpp"

class ImageProcessor : public rclcpp::Node {
public:
    ImageProcessor();

private:
    // 订阅压缩图像的 callback
    void compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    // 发布解压缩后的图像
    void publishImage(const cv::Mat& image);

    // ROS2 订阅器和发布器
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

#endif // IMAGEPROCESSOR_HPP