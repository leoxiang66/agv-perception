#include "MsgManager.hpp"

MsgManager::MsgManager() : Node("msg_manager"), all_msgs(10) {}  

MsgManager::~MsgManager(){}

void MsgManager::compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    // 回调处理逻辑
    (void)msg;
}

void MsgManager::pc2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 回调处理逻辑
    (void)msg;
}
