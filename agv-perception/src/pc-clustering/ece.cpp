/**
 * @file ece.cpp
 * @brief Implementation of Euclidean Cluster Extraction for point cloud processing in ROS 2.
 *
 * This file implements functions to:
 * 1. Convert ROS 2 `sensor_msgs::msg::PointCloud2` to PCL format.
 * 2. Build a KdTree for nearest neighbor search.
 * 3. Perform Euclidean clustering on the input point cloud.
 * 4. Extract individual clusters as separate PCL point clouds.
 *
 * @author Leon
 * @date 2025.03.13
 */

 #include "pc-clustering/ece.hpp"

 /**
  * @brief Converts a ROS 2 PointCloud2 message to a PCL point cloud.
  * 
  * @param msg Shared pointer to the ROS 2 PointCloud2 message.
  * @return pcl::PointCloud<pcl::PointXYZ>::Ptr Converted PCL point cloud.
  */
 pcl::PointCloud<pcl::PointXYZ>::Ptr pc2ToPCL(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
     pcl::fromROSMsg(*msg, *cloud);  // Convert ROS 2 message to PCL format
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
     ec.setClusterTolerance(0.05);  // Set cluster distance threshold (meters)
     ec.setMinClusterSize(100);     // Minimum number of points in a cluster
     ec.setMaxClusterSize(25000);   // Maximum number of points in a cluster
     ec.setSearchMethod(tree);
     ec.setInputCloud(cloud);
     
     // Perform clustering
     ec.extract(cluster_indices);
 
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
 
     for (const auto& indices : cluster_indices) {
         pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
 
         for (const auto& idx : indices.indices) {
             cluster->push_back((*cloud)[idx]);  // Extract each point in the cluster
         }
         
         clusters.push_back(cluster);
     }
 
     return clusters;
 }