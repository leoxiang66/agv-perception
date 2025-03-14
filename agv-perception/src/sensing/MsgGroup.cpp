#include "sensing/MsgGroup.hpp"

MsgGroup::MsgGroup() : point_cloud_(nullptr)
{
}

MsgGroup::~MsgGroup() {}

void MsgGroup::add_pc2(std::shared_ptr<sensor_msgs::msg::PointCloud2> pc2)
{
    mtx.lock();
    this->point_cloud_ = pc2;
    mtx.unlock();

    if (this->check_integrity())
    {
    }
}

void MsgGroup::add_image(const sensor_msgs::msg::Image::SharedPtr img)
{
    mtx.lock();
    this->imgs_.push_back(img);
    mtx.unlock();

    if (this->check_integrity())
    {
        // TODO: 触发回调或事件处理
        print_info();
    
    }
}

std::shared_ptr<sensor_msgs::msg::PointCloud2> MsgGroup::get_point_cloud() const
{

    return this->point_cloud_;
}

const std::vector<sensor_msgs::msg::Image::SharedPtr> &MsgGroup::get_images() const
{

    return imgs_;
}

bool MsgGroup::check_integrity()
{

    // std::cout << imgs_.size() << std::endl;
    // bool temp = this->point_cloud_ != nullptr;
    // std::cout <<"PC2: " << temp << std::endl;
    mtx.lock();
    auto temp = this->imgs_.size() == 2 && this->point_cloud_ != nullptr;
    mtx.unlock();

    return temp;
}

void MsgGroup::print_info() const
{

    rclcpp::Time pc_stamp = point_cloud_->header.stamp;

    std::cout << "Info of this msg group:" << std::endl;
    std::cout << "Size: " << size() << std::endl;
    std::cout << "PC2 timestamp: " << nanosec2date(pc_stamp.nanoseconds()) << std::endl;
    for (auto img : imgs_)
    {
        rclcpp::Time img_stamp = img->header.stamp;
        std::cout << "Image timestamp: " << nanosec2date(img_stamp.nanoseconds()) << std::endl;
    }
    std::cout << "" << std::endl;
}

size_t MsgGroup::size() const
{

    size_t ret = 0;
    if (this->point_cloud_ != nullptr)
    {
        ret += 1;
    }
    ret += imgs_.size();
    return ret;
}