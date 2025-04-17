#include "poseslam.hpp"
#include "simpleicp.h"

#include <tf2/LinearMath/Quaternion.h>

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
            pointcloudBuf.push(X_fix);
            pBufLock.unlock();
        }
    }

    void PoseSlam::add_pose_constraint(Eigen::Matrix<double, 4, 4> H)
    {
        if (key_poses_3d->points.empty())
        {
            gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector6(6) << 1e-2, 1e-2, M_PI * M_PI, 1e4, 1e4, 1e4).finished());
            auto prior_pose = gtsam::Pose3(H);
            f_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(0, prior_pose, prior_noise));
            isam_initial_estimate.insert(0, prior_pose);
        }
        else
        {
            gtsam::noiseModel::Diagonal::shared_ptr noise_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            f_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(key_poses_3d->size() - 1, key_poses_3d->size(), gtsam::Pose3(H), noise_model));

            gtsam::Pose3 lp = point_type_to_gtsam_pose(last_pose_);
            isam_initial_estimate.insert(key_poses_3d->size(), gtsam::Pose3(H * lp.matrix()));
        }
    }

    void PoseSlam::append_to_path(const PointTypePose &pose_in)
    {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = rclcpp::Time(pose_in.time);
        msg.pose.position.x = pose_in.x;
        msg.pose.position.y = pose_in.y;
        msg.pose.position.z = pose_in.z;

        tf2::Quaternion q;
        q.setRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();

        global_path.poses.push_back(msg);
    }

    gtsam::Pose3 PoseSlam::point_type_to_gtsam_pose(PointTypePose &pose_in)
    {
        return gtsam::Pose3(gtsam::Rot3::Ypr(pose_in.yaw, pose_in.pitch, pose_in.roll), gtsam::Point3(pose_in.x, pose_in.y, pose_in.z));
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(poseslam::PoseSlam)
