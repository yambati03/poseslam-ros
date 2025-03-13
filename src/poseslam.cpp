#include "poseslam.hpp"

/**
 * This is an example ROS 2 node that implements a simple pose SLAM algorithm using GTSAM.
 *
 * The node subscribes to a pointcloud topic and performs ICP to estimate the transformation between consecutive point clouds.
 * These transformations are used to add pose constraints to the factor graph in GTSAM. The estimated transformation is then used to update
 * the factor graph in GTSAM. The node also publishes the optimized poses as a path.
 */

namespace poseslam
{
    void PoseSlam::PoseSlam()
    {
        odom_sub = this->create_subscription<nav_msgs::Odometry>("odom", 1, std::bind(&PoseSlam::odom_callback, this, std::placeholders::_1));
        pointcloud_sub = this->create_subscription<sensor_msgs::PointCloud2>("pointcloud", 1, std::bind(&PoseSlam::pointcloud_callback, this, std::placeholders::_1));
    }

    void PoseSlam::pointcloud_callback(const sensor_msgs::PointCloud2::SharedPtr msg)
    {
        // Convert the point cloud to PCL format
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
        pcl::fromROSMsg(*msg, *cloud);

        if (!pointcloudBuf.empty())
        {
            // Convert pointcloud to Eigen::MatrixXd for further processing
            Eigen::MatrixXd X_mov(cloud->size(), 3);
            for (uint i = 0; i < cloud->size(); i++)
            {
                X(i, 0) = cloud->points[i].x;
                X(i, 1) = cloud->points[i].y;
                X(i, 2) = cloud->points[i].z;
            }

            // Get the last point cloud from the queue
            pBufLock.lock();
            auto X_mov = pointcloudBuf.front();
            pointcloudBuf.pop();
            pBufLock.unlock();

            Eigen::Matrix<double, 4, 4> H = SimpleICP(X_fix,
                                                      X_mov,
                                                      result["correspondences"].as<int>(),
                                                      result["neighbors"].as<int>(),
                                                      result["min_planarity"].as<double>(),
                                                      result["max_overlap_distance"].as<double>(),
                                                      result["min_change"].as<double>(),
                                                      result["max_iterations"].as<int>());

            // Save the transformation matrix to the queue
            transformBuf.push(H);

            // Save the point cloud data to the queue
            pBufLock.lock();
            pointcloudBuf.push(X_mov);
            pBufLock.unlock();
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(poseslam::PoseSlam)
