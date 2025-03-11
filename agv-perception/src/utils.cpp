#include "utils.hpp"


std::shared_ptr<cv::Mat> uncompress_image(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    try {
        cv::Mat raw_data(1, msg->data.size(), CV_8UC1, const_cast<uint8_t*>(msg->data.data()));
        cv::Mat image = cv::imdecode(raw_data, cv::IMREAD_COLOR);

        if (image.empty()) {
            std::cerr << "Failed to decode compressed image." << std::endl;
            return nullptr;
        }

        return std::make_shared<cv::Mat>(image);  // 使用智能指针自动管理内存

    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV Exception: " << e.what() << std::endl;
        return nullptr;
    }
}

