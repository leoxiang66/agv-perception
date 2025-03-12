#include "ImageProcessor.hpp"

ImageProcessor::ImageProcessor() : Node("image_processor") {
    // 订阅压缩图像的 topic
    compressed_image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/hikvision/camera/image_raw/compressed", 10,
        std::bind(&ImageProcessor::compressedImageCallback, this, std::placeholders::_1));

    // 发布解压缩后的图像的 topic
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image/uncompressed", 10);

    RCLCPP_INFO(this->get_logger(), "ImageProcessor node started.");
}

void ImageProcessor::compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    try {
        // 将压缩图像转换为 OpenCV 格式
        std::shared_ptr<cv::Mat> img = uncompress_image(msg);

        if (img)
        {
            // 发布解压缩后的图像
            publishImage(*img);
        }
        
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV Exception: %s", e.what());
    }
}

void ImageProcessor::publishImage(const cv::Mat& image) {
    // 将 OpenCV 图像转换为 ROS2 Image 消息
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
    msg->header.stamp = this->now(); // 设置时间戳

    // 发布图像
    image_pub_->publish(*msg);
    // RCLCPP_INFO(this->get_logger(), "Published uncompressed image.");
}