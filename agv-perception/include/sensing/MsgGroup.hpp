#ifndef MSGGROUP_HPP
#define MSGGROUP_HPP

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>
#include <memory>
#include <iostream>
#include "utils.hpp"
#include <mutex>
#include "pc-clustering/ece.hpp"

class MsgGroup {
public:
    MsgGroup();
    ~MsgGroup();

    void add_image(const sensor_msgs::msg::Image::SharedPtr img);
    void add_pc2(std::shared_ptr<sensor_msgs::msg::PointCloud2> pc2);

    std::shared_ptr<sensor_msgs::msg::PointCloud2> get_point_cloud() const;
    const std::vector<sensor_msgs::msg::Image::SharedPtr>& get_images() const;

    void print_info() const;

    size_t size() const;

private:
    std::shared_ptr<sensor_msgs::msg::PointCloud2> point_cloud_;
    std::vector<sensor_msgs::msg::Image::SharedPtr> imgs_;

    bool check_integrity();
    std::mutex mtx;
};

#endif // MSGGROUP_HPP
