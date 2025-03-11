#include "utils.hpp"
#include "ImageProcessor.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    // 初始化 ROS2
    rclcpp::init(argc, argv);

    // 创建 ImageProcessor 节点
    auto node = std::make_shared<ImageProcessor>();

    // 运行节点
    rclcpp::spin(node);

    // 关闭 ROS2
    rclcpp::shutdown();
    return 0;
}


