#ifndef POSE_SLAM_HPP_
#define POSE_SLAM_HPP_

#include <mutex>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/linear/NoiseModel.h>

namespace poseslam
{
    class PoseSlam : public rclcpp::Node
    {
    public:
        PoseSlam();
        ~PoseSlam();

    private:
        void odom_callback(const nav_msgs::Odometry::SharedPtr msg);
        void pointcloud_callback(const sensor_msgs::PointCloud2::SharedPtr msg);
        void add_odom_factor();

        std::mutex pBufLock;
        std::queue<Eigen::MatrixXd> pointcloudBuf;
        std::queue<Eigen::Matrix<double, 4, 4>> transformBuf;

        ISAM2 *gisam;
        Values gisamCurrentEstimate;
        ISAM2Params gparameters;
        NonlinearFactorGraph f_graph_;

        rclcpp::Subscription<nav_msgs::Odometry>::SharedPtr odom_sub;
        rclcpp::Subscription<sensor_msgs::PointCloud2>::SharedPtr pointcloud_sub;
    }
} // namespace poseslam

#endif // POSE_SLAM_HPP_