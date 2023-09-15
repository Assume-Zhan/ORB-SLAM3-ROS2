#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>

/* ORB-SLAM3 Library */
#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

/* ROS2 Message Library */
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

/* TF2 Library */
#include "tf2/exceptions.h"
#include "tf2/convert.h"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "utility.hpp"


class MonocularSlamNode : public rclcpp::Node {

public:

    MonocularSlamNode(ORB_SLAM3::System* pSLAM);
    ~MonocularSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;

    // Setup
    void setup_ros_publishers(Eigen::Vector3d rpy_rad);

    // Main Callback function
    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

    // Main publish message function
    void publish_ros_tf_transform(Sophus::SE3f Twc_SE3f, std_msgs::msg::Header header);
    void publish_ros_tracked_mappoints(std::vector<ORB_SLAM3::MapPoint*> map_points, rclcpp::Time msg_time);

    // ORB-SLAM3 map point to point cloud
    sensor_msgs::msg::PointCloud2 tracked_mappoints_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*> map_points, rclcpp::Time msg_time);

    // Transformation
    tf2::Transform SE3f_to_tfTransform(Sophus::SE3f T_SE3f);

    geometry_msgs::msg::PoseArray GetAllKfsPts(rclcpp::Time msg_time);

    ORB_SLAM3::System* m_SLAM;
    ORB_SLAM3::System::eSensor sensor_type;

    Sophus::SE3f Tc0w;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr all_kfs_pts_pub;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    geometry_msgs::msg::TransformStamped stamped_transform;

    string child_frame_id;
    string world_frame_id;
};

#endif
