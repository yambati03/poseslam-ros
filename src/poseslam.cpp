#include "poseslam.hpp"
#include "simpleicp.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/PCLPointCloud2.h>

/**
 * This is an example ROS 2 node that implements a simple pose SLAM algorithm using GTSAM.
 *
 * The node subscribes to a pointcloud topic and performs ICP to estimate the transformation between consecutive point clouds.
 * These transformations are used to add pose constraints to the factor graph in GTSAM. The estimated transformation is then used to update
 * the factor graph in GTSAM. The node also publishes the optimized poses as a path.
 */

namespace poseslam
{
    PoseSlam::PoseSlam(const rclcpp::NodeOptions &options) : Node("pose_slam", options)
    {
        pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("pointcloud", 1, std::bind(&PoseSlam::pointcloud_callback, this, std::placeholders::_1));
    }

    void PoseSlam::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert the point cloud to PCL format
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (!pointcloudBuf.empty())
        {
            // Convert pointcloud to Eigen::MatrixXd for further processing
            Eigen::MatrixXd X_fix(cloud->size(), 3);
            for (uint i = 0; i < cloud->size(); i++)
            {
                X_fix(i, 0) = cloud->points[i].x;
                X_fix(i, 1) = cloud->points[i].y;
                X_fix(i, 2) = cloud->points[i].z;
            }

            // Get the last point cloud from the queue
            pBufLock.lock();
            auto X_mov = pointcloudBuf.front();
            pointcloudBuf.pop();
            pBufLock.unlock();

            Eigen::Matrix<double, 4, 4> H = SimpleICP(
                X_fix,
                X_mov,
                1000, // Number of correspondences
                10,   // Number of neighbors
                0.3,  // Minimum planarity
                -1,   // Maximum overlap distance
                1,    // Minimum change
                100   // Maximum iterations
            );

            // Save the transformation matrix to the queue
            transformBuf.push(H);

            // Save the point cloud data to the queue
            pBufLock.lock();
            pointcloudBuf.push(X_mov);
            pBufLock.unlock();
        }
    }

    void PoseSlam::add_pose_constraint(Eigen::Matrix<double, 4, 4> H)
    {
        gtsam::noiseModel::Diagonal::shared_ptr noise_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(poseslam::PoseSlam)
