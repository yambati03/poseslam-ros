#include "poseslam.hpp"
#include "simpleicp.h"

#include <tf2/LinearMath/Quaternion.h>
#include <string>

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
        RCLCPP_INFO(this->get_logger(), "Poseslam node started...");
        read_parameters();

        pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, 1, std::bind(&PoseSlam::pointcloud_callback, this, std::placeholders::_1));
        path_pub = this->create_publisher<nav_msgs::msg::Path>("/optimized_path", 1);

        // Initialize the iSAM2 optimizer
        isam_params = gtsam::ISAM2Params();
        isam_params.relinearizeThreshold = 0.1;
        isam_params.relinearizeSkip = 1;

        isam = new gtsam::ISAM2(isam_params);

        // Initialize the key poses
        key_poses_3d = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
        key_poses_6d = pcl::PointCloud<PointTypePose>::Ptr(new pcl::PointCloud<PointTypePose>());

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Call update_and_correct() in a separate thread. This is the main loop that will
        // update the factor graph and optimize the poses.
        std::thread([this]()
                    {
                        int update_rate_ms = 1000 / update_rate_hz_;
                        while (rclcpp::ok())
                        {
                            update_and_correct();
                            std::this_thread::sleep_for(std::chrono::milliseconds(update_rate_ms));
                        } })
            .detach();
    }

    void PoseSlam::update_and_correct()
    {
        if (pointcloud_buf.size() < 2)
            return;

        // Get the most recent point cloud from the queue
        p_buf_lock.lock();
        auto X = pointcloud_buf.front();
        pointcloud_buf.pop();
        while (!pointcloud_buf.empty())
        {
            pointcloud_buf.pop();
        }
        p_buf_lock.unlock();

        Eigen::Matrix<double, 4, 4> H = SimpleICP(
            last_pointcloud_,
            X,
            1000, // Number of correspondences
            10,   // Number of neighbors
            0.3,  // Minimum planarity
            1,    // Maximum overlap distance
            1,    // Minimum change
            100   // Maximum iterations
        );

        // Add the pose constraint to the factor graph
        add_pose_constraint(H);

        // Add the factor graph to the optimizer
        isam->update(f_graph_, isam_initial_estimate);

        // Clear the factor graph and initial estimate for the next iteration
        f_graph_.resize(0);
        isam_initial_estimate.clear();

        pcl::PointXYZI this_pose_3d;
        gtsam::Pose3 latest_estimate;

        // Get the latest estimate from iSAM2 optimizer
        isam_current_estimate = isam->calculateEstimate();
        latest_estimate = isam_current_estimate.at<gtsam::Pose3>(isam_current_estimate.size() - 1);

        // Populate thisPose3D with translation data
        this_pose_3d.x = latest_estimate.translation().x();
        this_pose_3d.y = latest_estimate.translation().y();
        this_pose_3d.z = latest_estimate.translation().z();
        this_pose_3d.intensity = key_poses_3d->size(); // Use intensity as index
        key_poses_3d->push_back(this_pose_3d);

        // Use the new constructor to create thisPose6D directly
        PointTypePose this_pose_6d(latest_estimate, this_pose_3d.intensity, curr_time_);
        key_poses_6d->push_back(this_pose_6d);

        // Update the global path with the latest pose and publish it
        append_to_path(this_pose_6d);
        global_path.header.stamp = rclcpp::Time(curr_time_);
        global_path.header.frame_id = "map";
        path_pub->publish(global_path);
        publish_transform(this_pose_6d, "map", "world");

        last_pointcloud_ = X;
        last_pose_ = this_pose_6d;

        correct_poses();
    }

    void PoseSlam::publish_transform(PointTypePose pose, std::string frame_id, std::string child_frame_id)
    {
        // Broadcast the transform from map -> world based on the latest pose
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = rclcpp::Time(curr_time_);
        transform.header.frame_id = frame_id;
        transform.child_frame_id = child_frame_id;
        transform.transform.translation.x = pose.x;
        transform.transform.translation.y = pose.y;
        transform.transform.translation.z = pose.z;

        tf2::Quaternion q;
        q.setRPY(pose.roll, pose.pitch, pose.yaw);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform);
    }

    void PoseSlam::correct_poses()
    {
        if (key_poses_3d->points.empty())
            return;

        int num_poses = isam_current_estimate.size();
        global_path.poses.clear();

        for (int i = 0; i < num_poses; ++i)
        {
            key_poses_3d->points[i].x = isam_current_estimate.at<gtsam::Pose3>(i).translation().x();
            key_poses_3d->points[i].y = isam_current_estimate.at<gtsam::Pose3>(i).translation().y();
            key_poses_3d->points[i].z = isam_current_estimate.at<gtsam::Pose3>(i).translation().z();

            key_poses_6d->points[i].x = key_poses_3d->points[i].x;
            key_poses_6d->points[i].y = key_poses_3d->points[i].y;
            key_poses_6d->points[i].z = key_poses_3d->points[i].z;
            key_poses_6d->points[i].roll = isam_current_estimate.at<gtsam::Pose3>(i).rotation().roll();
            key_poses_6d->points[i].pitch = isam_current_estimate.at<gtsam::Pose3>(i).rotation().pitch();
            key_poses_6d->points[i].yaw = isam_current_estimate.at<gtsam::Pose3>(i).rotation().yaw();

            append_to_path(key_poses_6d->points[i]);
        }
    }

    PoseSlam::~PoseSlam()
    {
        delete isam;
    }

    void PoseSlam::filterSphere(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius)
    {
        double radiusSquared = radius * radius;
        pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>());
        filteredCloud->points.reserve(cloud->points.size());

        for (const auto &point : cloud->points)
        {
            float distSquared = point.x * point.x + point.y * point.y + point.z * point.z;
            if (distSquared <= radiusSquared && distSquared >= 0.5)
            {
                filteredCloud->points.push_back(point);
            }
        }

        *cloud = std::move(*filteredCloud);
    }

    void PoseSlam::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert the point cloud to PCL format
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Filter the point cloud to remove points outside the specified radius
        filterSphere(cloud, 5.0);

        // Convert pointcloud to Eigen::MatrixXd for further processing
        Eigen::MatrixXd X(cloud->size(), 3);
        for (uint i = 0; i < cloud->size(); i++)
        {
            X(i, 0) = cloud->points[i].x;
            X(i, 1) = cloud->points[i].y;
            X(i, 2) = cloud->points[i].z;
        }

        // Add the current point cloud to the buffer
        p_buf_lock.lock();
        if (last_pointcloud_.size() == 0)
        {
            last_pointcloud_ = X;
        }
        pointcloud_buf.push(X);
        p_buf_lock.unlock();

        curr_time_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
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

    void PoseSlam::read_parameters()
    {
        this->declare_parameter("pointcloud_topic", "/pointcloud");
        this->declare_parameter("update_rate_hz", 10.0);

        if (!this->get_parameter("pointcloud_topic", pointcloud_topic_))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not read parameter pointcloud_topic.");
            return;
        }

        if (!this->get_parameter("update_rate_hz", update_rate_hz_))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not read parameter update_rate_hz.");
            return;
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(poseslam::PoseSlam)
