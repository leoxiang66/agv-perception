#include "MsgGroup.hpp"

MsgGroup::MsgGroup() : point_cloud_(nullptr) {}

MsgGroup::~MsgGroup() {}

void MsgGroup::add_pc2(std::shared_ptr<sensor_msgs::msg::PointCloud2> pc2) {
    this->point_cloud_ = pc2;

    if (this->check_integrity()) {
        // TODO: 触发回调或事件处理
    }
}

void MsgGroup::add_image(const sensor_msgs::msg::Image& img) {
    this->imgs_.push_back(img);

    if (this->check_integrity()) {
        // TODO: 触发回调或事件处理
    }
}

std::shared_ptr<sensor_msgs::msg::PointCloud2> MsgGroup::get_point_cloud() const {
    return this->point_cloud_;
}

const std::vector<sensor_msgs::msg::Image>& MsgGroup::get_images() const {
    return imgs_;
}

bool MsgGroup::check_integrity() const {
    return this->imgs_.size() == 2 && this->point_cloud_ != nullptr;
}
