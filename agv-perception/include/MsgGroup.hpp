#ifndef MSGGROUP_HPP
#define MSGGROUP_HPP

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>
#include <memory>

class MsgGroup {
public:
    MsgGroup();
    ~MsgGroup();

    void add_image(const sensor_msgs::msg::Image& img);
    void add_pc2(std::shared_ptr<sensor_msgs::msg::PointCloud2> pc2);

    std::shared_ptr<sensor_msgs::msg::PointCloud2> get_point_cloud() const;
    const std::vector<sensor_msgs::msg::Image>& get_images() const;

private:
    std::shared_ptr<sensor_msgs::msg::PointCloud2> point_cloud_;
    std::vector<sensor_msgs::msg::Image> imgs_;

    bool check_integrity() const;
};

#endif // MSGGROUP_HPP
