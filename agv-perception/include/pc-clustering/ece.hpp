/**
 * @file ece.hpp
 * @brief Euclidean Cluster Extraction (ECE) for point cloud clustering using PCL and ROS 2.
 *
 * This header defines functions to process a ROS 2 PointCloud2 message using
 * Euclidean Cluster Extraction from the Point Cloud Library (PCL).
 * The clustering pipeline includes:
 * 1. Converting `sensor_msgs::msg::PointCloud2` to `pcl::PointCloud<pcl::PointXYZ>`.
 * 2. Building a KdTree for nearest neighbor search.
 * 3. Performing Euclidean clustering.
 * 4. Extracting individual clusters as separate point clouds.
 *
 * @author Leon
 * @date 2025.03.13
 */

#ifndef ECE_HPP
#define ECE_HPP

#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>

/**
 * @brief Converts a ROS 2 PointCloud2 message to a PCL point cloud.
 *
 * @param msg Shared pointer to the ROS 2 PointCloud2 message.
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr Converted PCL point cloud.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr pc2ToPCL(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

/**
 * @brief Builds a KdTree from a given point cloud.
 *
 * @param cloud Input point cloud.
 * @return pcl::search::KdTree<pcl::PointXYZ>::Ptr KdTree used for clustering.
 */
pcl::search::KdTree<pcl::PointXYZ>::Ptr buildKdTree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

/**
 * @brief Performs Euclidean Cluster Extraction on the input point cloud.
 *
 * @param cloud Input point cloud.
 * @param tree KdTree for efficient nearest neighbor search.
 * @return std::vector<pcl::PointIndices> A vector of cluster indices, each representing a detected cluster.
 */
std::vector<pcl::PointIndices> performClustering(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree);

/**
 * @brief Extracts individual clusters from a point cloud based on clustering indices.
 *
 * @param cloud Input point cloud.
 * @param cluster_indices Vector of cluster indices obtained from `performClustering()`.
 * @return std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> A vector of extracted clusters as separate point clouds.
 */
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> extractClusterPoints(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    std::vector<pcl::PointIndices> cluster_indices);


#endif // ECE_HPP