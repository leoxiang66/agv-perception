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

        return std::make_shared<cv::Mat>(image); 

    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV Exception: " << e.what() << std::endl;
        return nullptr;
    }
}

std::string nanosec2date(uint64_t nanoseconds)
{
    // Convert nanoseconds to seconds since epoch
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::nanoseconds(nanoseconds));

    // Create a time_point from the seconds
    auto time_point = std::chrono::system_clock::time_point(seconds);

    // Convert to a std::time_t for formatting
    std::time_t time_t_value = std::chrono::system_clock::to_time_t(time_point);

    // Format the time as a string (use std::strftime as an alternative to std::put_time)
    char buffer[100];
    if (std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", std::localtime(&time_t_value)))
    {
        // Add nanoseconds part for precision
        int64_t remaining_nanos = nanoseconds % 1000000000;

        // Use a stringstream for constructing the final result
        std::ostringstream oss;
        oss << buffer << "." << std::setw(9) << std::setfill('0') << remaining_nanos;

        return oss.str();
    }

    // If formatting fails, return an empty string
    return "";
}

