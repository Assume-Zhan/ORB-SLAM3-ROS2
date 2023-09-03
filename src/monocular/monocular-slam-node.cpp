#include "monocular-slam-node.hpp"

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM) : Node("ORB_SLAM3_ROS2") {

    m_SLAM = pSLAM;

    // Original orientation
    Eigen::Vector3d rpy_rad;
    rpy_rad << 1.57079632679, 0, 0;

    this->setup_ros_publishers(rpy_rad);

    sensor_type = ORB_SLAM3::System::MONOCULAR;

    world_frame_id = "map";
    child_frame_id = "base_footprint";
}

MonocularSlamNode::~MonocularSlamNode() {
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::setup_ros_publishers(Eigen::Vector3d rpy_rad) {

    m_image_subscriber = this->create_subscription<ImageMsg>("camera", 10, std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));

    tf_broadcaster  = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    if (!rpy_rad.isZero(0)) {
        Eigen::AngleAxisf AngleR(rpy_rad(0), Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf AngleP(rpy_rad(1), Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf AngleY(rpy_rad(2), Eigen::Vector3f::UnitZ());

        Eigen::Quaternionf qRPY = AngleR * AngleP * AngleY;

        Eigen::Matrix3f RotRPY = qRPY.matrix();
        Tc0w = Sophus::SE3f(RotRPY, Eigen::Vector3f::Zero());
    }

    RCLCPP_INFO_STREAM(get_logger(), "[ORB-SLAM3 Mono] : ROS2 setup ok");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg) {

    // Copy the ros image message to cv::Mat.
    try {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    // SE3f -> Rigid body with rotation and translation
    Sophus::SE3f Tcc0 = m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
    Sophus::SE3f Twc = (Tcc0 * Tc0w).inverse();

    // Broadcast TF "map frame" to "camera pose"
    // TODO : Listen to base_link -> camera frame and only publish on "map frame" to "base_footprint"
    publish_ros_tf_transform(Twc, msg->header);
}

void MonocularSlamNode::publish_ros_tf_transform(Sophus::SE3f Twc_SE3f, std_msgs::msg::Header header) {

}

tf2::Transform MonocularSlamNode::SE3f_to_tfTransform(Sophus::SE3f T_SE3f) {

}