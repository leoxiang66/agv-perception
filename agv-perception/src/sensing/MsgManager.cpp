#include "sensing/MsgManager.hpp"

MsgManager::MsgManager() : Node("msg_manager"), all_msgs(10)
{
    compressed_image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/hikvision/camera/image_raw/compressed", 10,
        std::bind(&MsgManager::compressedImageCallback, this, std::placeholders::_1));

    pc2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", 10,
        std::bind(&MsgManager::pc2Callback, this, std::placeholders::_1));


    // dev
    pc2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/clustered_cloud", 10);

    RCLCPP_INFO(this->get_logger(), "Msg Manager node started.");
}

MsgManager::~MsgManager() {}

void MsgManager::compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // 回调处理逻辑
    rclcpp::Time timestamp =  msg->header.stamp;
    uint64_t nanosec =  timestamp.nanoseconds();

    auto key = nanosec2date(nanosec);
    key = key.substr(0, key.size() - 8);

    // TODO: 在实际中直接处理原图
    cv_bridge::CvImage cv_img;
    cv_img.header = msg->header;
    cv_img.encoding = "bgr8";
    cv_img.image = *uncompress_image(msg);
    auto img_msg = cv_img.toImageMsg();

    if (this->all_msgs.contains(key))
    {
        auto msg_group = all_msgs.get(key);
        msg_group->add_image(img_msg);
    }else
    {
        auto msg_group = std::make_shared<MsgGroup>();
        msg_group->add_image(img_msg);
        all_msgs.insert(key,msg_group);
    }


}

void MsgManager::pc2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // 回调处理逻辑
    rclcpp::Time timestamp =  msg->header.stamp;
    uint64_t nanosec =  timestamp.nanoseconds();

  
    auto key = nanosec2date(nanosec);
    key = key.substr(0, key.size() - 8);

    if (this->all_msgs.contains(key))
    {
        auto msg_group = all_msgs.get(key);
        msg_group->add_pc2(msg);
    }else
    {
        auto msg_group = std::make_shared<MsgGroup>();
        msg_group->add_pc2(msg);
        all_msgs.insert(key,msg_group);
    }

    // dev
    auto temp1 = pc2ToPCL(msg);
    auto temp2 = buildKdTree(temp1);
    auto temp3 = performClustering(temp1, temp2);
    auto temp4 = extractClusterPoints(temp1, temp3);
    publishClusters(temp4);
}


void MsgManager::publishClusters(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    std::vector<uint8_t> colors = {255, 0, 0, 0, 255, 0, 0, 0, 255};  // RGB colors
    for (size_t i = 0; i < clusters.size(); ++i) {
        uint8_t r = colors[(i % 3) * 3];
        uint8_t g = colors[(i % 3) * 3 + 1];
        uint8_t b = colors[(i % 3) * 3 + 2];

        for (const auto &point : clusters[i]->points) {
            pcl::PointXYZRGB p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            p.r = r;
            p.g = g;
            p.b = b;
            colored_cloud->push_back(p);
        }
    }

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*colored_cloud, output_msg);
    output_msg.header.frame_id = "livox_frame";  
    pc2_pub_->publish(output_msg);
}