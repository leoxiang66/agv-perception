#include "sensing/MsgManager.hpp"

MsgManager::MsgManager() : Node("msg_manager"), all_msgs(10)
{
    compressed_image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/hikvision/camera/image_raw/compressed", 10,
        std::bind(&MsgManager::compressedImageCallback, this, std::placeholders::_1));

    pc2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", 10,
        std::bind(&MsgManager::pc2Callback, this, std::placeholders::_1));

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

    // for(auto key : all_msgs.get_keys())
    // {
    //     all_msgs.get(key).print_info();
    // }
}
