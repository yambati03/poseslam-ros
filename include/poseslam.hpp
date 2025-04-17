#ifndef POSE_SLAM_HPP_
#define POSE_SLAM_HPP_

#include <mutex>
#include <iostream>
#include <queue>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

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

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/eigen.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/pcl_macros.h>

#include <Eigen/Dense>

struct PointTypePose
{
    PCL_ADD_POINT4D; // PCL macro to add x, y, and z fields to the custom point type
    float intensity;
    float roll, pitch, yaw;
    double time;

    PointTypePose()
        : x(0), y(0), z(0), intensity(0), roll(0), pitch(0), yaw(0), time(0) {}

    PointTypePose(float _x, float _y, float _z, float _intensity,
                  float _roll, float _pitch, float _yaw, double _time)
        : x(_x), y(_y), z(_z), intensity(_intensity),
          roll(_roll), pitch(_pitch), yaw(_yaw), time(_time) {}

    PointTypePose(const gtsam::Pose3 &pose, float _intensity, double _time)
        : x(pose.translation().x()), y(pose.translation().y()), z(pose.translation().z()),
          intensity(_intensity),
          roll(pose.rotation().roll()), pitch(pose.rotation().pitch()), yaw(pose.rotation().yaw()),
          time(_time) {}
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointTypePose,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

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

        void append_to_path(const PointTypePose &pose_in);
        gtsam::Pose3 point_type_to_gtsam_pose(PointTypePose &pose_in);

        std::mutex pBufLock;
        std::queue<Eigen::MatrixXd> pointcloudBuf;
        std::queue<Eigen::Matrix<double, 4, 4>> transformBuf;

        gtsam::ISAM2 *isam;
        gtsam::Values isam_initial_estimate;
        gtsam::Values isam_current_estimate;
        gtsam::ISAM2Params isam_params;
        gtsam::NonlinearFactorGraph f_graph_;

        // We hold key poses in a PCL pointcloud type since PCL has nice helper functions
        // to construct a KD tree from a pointcloud. This allows us to efficiently query the
        // spatial distance between two poses.
        pcl::PointCloud<pcl::PointXYZ>::Ptr key_poses_3d;
        pcl::PointCloud<PointTypePose>::Ptr key_poses_6d;
        PointTypePose last_pose_;

        nav_msgs::msg::Path global_path;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    };
} // namespace poseslam

#endif // POSE_SLAM_HPP_