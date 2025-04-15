#ifndef POSE_SLAM_HPP_
#define POSE_SLAM_HPP_

#include <mutex>
#include <iostream>
#include <queue>
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

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace poseslam
{
    class PoseSlam : public rclcpp::Node
    {
    public:
        explicit PoseSlam(const rclcpp::NodeOptions &options);
        ~PoseSlam();

    private:
        void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void add_pose_constraint(Eigen::Matrix<double, 4, 4> H);

        std::mutex pBufLock;
        std::queue<Eigen::MatrixXd> pointcloudBuf;
        std::queue<Eigen::Matrix<double, 4, 4>> transformBuf;

        gtsam::ISAM2 *gisam;
        gtsam::Values gisamCurrentEstimate;
        gtsam::ISAM2Params gparameters;
        gtsam::NonlinearFactorGraph f_graph_;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;
    };
} // namespace poseslam

#endif // POSE_SLAM_HPP_