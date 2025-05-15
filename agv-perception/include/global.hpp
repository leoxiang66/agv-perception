#include "blocking_queue.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// 只做声明，不分配内存

extern BlockingQueue<std::shared_ptr<sensor_msgs::msg::Image>> img_chan;
extern BlockingQueue<std::shared_ptr<sensor_msgs::msg::PointCloud2>> pc2_chan;
