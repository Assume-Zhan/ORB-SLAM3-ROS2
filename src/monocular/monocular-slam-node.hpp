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
#include "nav_msgs/msg/occupancy_grid.hpp"

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
    geometry_msgs::msg::PoseArray GetSingleKfPts(rclcpp::Time msg_time);
    void PoseToCostmap (geometry_msgs::msg::PoseArray& kf_pts_array, rclcpp::Time msg_time);
    void UpdateGridMap (geometry_msgs::msg::PoseArray& kf_pts_array);
    void ProcessPts (const std::vector<geometry_msgs::msg::Pose> &pts, unsigned int n_pts, unsigned int start_id);
    void ProcessPt (const geometry_msgs::msg::Point &curr_pt, cv::Mat &occupied, cv::Mat &visited, cv::Mat &pt_mask);
    void SetGridOrigin (nav_msgs::msg::OccupancyGrid &grid, float &grid_min_x, float &grid_min_y);
    void CreateCvMat (const unsigned int h, const unsigned int w);
    void GetGridMap();

    ORB_SLAM3::System* m_SLAM;
    ORB_SLAM3::System::eSensor sensor_type;

    Sophus::SE3f Tc0w;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr all_kfs_pts_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr single_kf_pts_pub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    geometry_msgs::msg::TransformStamped stamped_transform;

    string child_frame_id;
    string world_frame_id;

    nav_msgs::msg::OccupancyGrid grid_map_cost_msg_;

    float scale_fac_ = 5.0;
    float cloud_lim_[4] = {-20.0, 20.0, -20.0, 20.0};
    float grid_lim_[4];
    float norm_fac_[2];
    unsigned int h_, w_;
    unsigned int n_kf_received_ = 0;
    unsigned int visit_thresh_ = 0;
    float kf_pos_x_, kf_pos_y_;
    int kf_pos_grid_x_, kf_pos_grid_y_;
    cv::Mat global_occupied_counter_, global_visit_counter_;
    cv::Mat local_occupied_counter_, local_visit_counter_;
    cv::Mat local_map_pt_mask_;
    cv::Mat grid_map_int_; 
    cv::Mat grid_map_; 
    cv::Mat grid_map_thresh_; 
};

#endif
