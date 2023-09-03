#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
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

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

    ORB_SLAM3::System* m_SLAM;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
};

#endif
