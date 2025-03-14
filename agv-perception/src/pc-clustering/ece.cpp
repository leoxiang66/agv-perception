#include "pc-clustering/ece.hpp"


/**
 * @brief Converts a ROS 2 PointCloud2 message to a PCL point cloud.
 * 
 * @param msg Shared pointer to the ROS 2 PointCloud2 message.
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr Converted PCL point cloud.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr pc2ToPCL(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // 检查转换后的点云是否为空
    if (cloud->points.empty()) {
        std::cout << "Error: Converted point cloud is empty!" << std::endl;
    } else {
        std::cout << "Successfully converted PointCloud2! Total points: " << cloud->points.size() << std::endl;
    }

    // 删除 (0, 0, 0) 的点
    cloud->points.erase(
        std::remove_if(cloud->points.begin(), cloud->points.end(), [](const pcl::PointXYZ& point) {
            return (point.x == 0.0f && point.y == 0.0f && point.z == 0.0f) || (std::abs(point.z) >=3.0f) || (std::abs(point.x)>=5.0f) || (std::abs(point.y) >= 3.0f);
        }),
        cloud->points.end()
    );

    // 打印删除后的点云信息
    std::cout << "After removing (0, 0, 0) points, Total points: " << cloud->points.size() << std::endl;

    return cloud;
}


/**
 * @brief Builds a KdTree from a given point cloud.
 * 
 * @param cloud Input point cloud.
 * @return pcl::search::KdTree<pcl::PointXYZ>::Ptr KdTree used for clustering.
 */
pcl::search::KdTree<pcl::PointXYZ>::Ptr buildKdTree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);  // Set input cloud for nearest neighbor search

    return tree;
}

/**
 * @brief Performs Euclidean Cluster Extraction on the input point cloud.
 * 
 * @param cloud Input point cloud.
 * @param tree KdTree for efficient nearest neighbor search.
 * @return std::vector<pcl::PointIndices> A vector of cluster indices, each representing a detected cluster.
 */
std::vector<pcl::PointIndices> performClustering(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree) {

    std::vector<pcl::PointIndices> cluster_indices;

    // Euclidean Cluster Extraction
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.5);  // Set cluster distance threshold (meters)
    ec.setMinClusterSize(100);     // Minimum number of points in a cluster
    ec.setMaxClusterSize(5000);   // Maximum number of points in a cluster
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);

    std::cout << "Performing clustering..." << std::endl;

    // Perform clustering
    ec.extract(cluster_indices);

    std::cout << "Detected " << cluster_indices.size() << " clusters." << std::endl;

    // Calculate the centroid of each cluster (average of all points' xyz)
    for (size_t i = 0; i < cluster_indices.size(); ++i) {
        std::cout << "Cluster " << i + 1 << ": Size = " << cluster_indices[i].indices.size() << " points." << std::endl;

        // Extract the points for the current cluster
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : cluster_indices[i].indices) {
            cluster->push_back((*cloud)[idx]);
        }

        // Manually calculate the centroid (average of all points' xyz)
        float x_sum = 0.0f, y_sum = 0.0f, z_sum = 0.0f;
        for (const auto& point : cluster->points) {
            x_sum += point.x;
            y_sum += point.y;
            z_sum += point.z;
        }

        // Calculate the average
        size_t num_points = cluster->points.size();
        float x_avg = x_sum / num_points;
        float y_avg = y_sum / num_points;
        float z_avg = z_sum / num_points;

        std::cout << "Cluster " << i + 1 << " centroid: ("
                  << x_avg << ", "
                  << y_avg << ", "
                  << z_avg << ")" << std::endl;
    }

    return cluster_indices;
}


/**
 * @brief Extracts individual clusters from a point cloud based on clustering indices.
 * 
 * @param cloud Input point cloud.
 * @param cluster_indices Vector of cluster indices obtained from `performClustering()`.
 * @return std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> A vector of extracted clusters as separate point clouds.
 */
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> extractClusterPoints(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    std::vector<pcl::PointIndices> cluster_indices) {

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

    for (size_t i = 0; i < cluster_indices.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);

        // Print size of the cluster
        std::cout << "Extracting cluster " << i + 1 << " with " 
                  << cluster_indices[i].indices.size() << " points." << std::endl;

        for (const auto& idx : cluster_indices[i].indices) {
            cluster->push_back((*cloud)[idx]);  // Extract each point in the cluster
        }

        clusters.push_back(cluster);
    }

    std::cout << "Extracted " << clusters.size() << " clusters." << std::endl;

    return clusters;
}
