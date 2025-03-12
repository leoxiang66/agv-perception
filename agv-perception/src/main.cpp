#include "utils.hpp"
#include "ImageProcessor.hpp"
#include "MsgManager.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    // 初始化 ROS2
    rclcpp::init(argc, argv);

    // 创建 ImageProcessor 节点
    auto node1 = std::make_shared<ImageProcessor>();

    auto node2 = std::make_shared<MsgManager>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node1);
    executor.add_node(node2);
    executor.spin();


    // 关闭 ROS2
    rclcpp::shutdown();
    return 0;
}


