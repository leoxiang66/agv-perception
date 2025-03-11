#ifndef UTILS_HPP
#define UTILS_HPP

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>



std::shared_ptr<cv::Mat> uncompress_image(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

#endif // !UTILS_HPP
