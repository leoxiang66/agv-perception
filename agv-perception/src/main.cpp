#include "utils.hpp"
#include "ImageProcessor.hpp"
#include "sensing/MsgManager.hpp"
#include <rclcpp/rclcpp.hpp> 
#include "global.hpp"
#include <thread>
#include "sensing/inference.hpp"
#include "yolov8/yolov8.h"

BlockingQueue<std::shared_ptr<sensor_msgs::msg::Image>> img_chan{3};
BlockingQueue<std::shared_ptr<sensor_msgs::msg::PointCloud2>> pc2_chan{3};

int main(int argc, char **argv)
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);

    YoloV8Config config;

    // Create the YoloV8 engine
    YoloV8 yoloV8("/home/git/YOLOv8-TensorRT-CPP/yolov8n.onnx", config);

    // 创建 ImageProcessor 节点
    auto node1 = std::make_shared<ImageProcessor>();

    auto node2 = std::make_shared<MsgManager>();

    std::thread m{consume_img, &yoloV8, &config}, n{consume_pc2};

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node1);
    executor.add_node(node2);
    executor.spin();


    // 关闭 ROS2
    rclcpp::shutdown();
    return 0;
}
