#include "monocular-slam-node.hpp"

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM) : Node("ORB_SLAM3_ROS2") {

    m_SLAM = pSLAM;

    // Original orientation
    Eigen::Vector3d rpy_rad;
    rpy_rad << 0, 0, 0;

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

    map_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_in", 1);

    all_kfs_pts_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("all_kfs", 1);
    single_kf_pts_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("single_kfs", 1);
    
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
    publish_ros_tracked_mappoints(m_SLAM->GetAllMapPoints(), msg->header.stamp);

    all_kfs_pts_pub->publish(GetAllKfsPts(msg->header.stamp));
    single_kf_pts_pub->publish(GetSingleKfPts(msg->header.stamp));
}

void MonocularSlamNode::publish_ros_tf_transform(Sophus::SE3f Twc_SE3f, std_msgs::msg::Header header) {

    tf2::Transform tf_transform = SE3f_to_tfTransform(Twc_SE3f);

    stamped_transform.header = header;
    stamped_transform.header.frame_id = world_frame_id;

    // TODO : child frame id should be the header frame
    // Transform map_to_camera to map_to_baselink
    stamped_transform.child_frame_id = child_frame_id;

    tf2::toMsg(tf_transform, stamped_transform.transform);

    tf_broadcaster->sendTransform(stamped_transform);
}

void MonocularSlamNode::publish_ros_tracked_mappoints(std::vector<ORB_SLAM3::MapPoint*> map_points, rclcpp::Time msg_time) {

    sensor_msgs::msg::PointCloud2 cloud = tracked_mappoints_to_pointcloud(map_points, msg_time);
    
    map_points_pub->publish(cloud);
}

sensor_msgs::msg::PointCloud2 MonocularSlamNode::tracked_mappoints_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*> map_points, rclcpp::Time msg_time) {

    const int num_channels = 3;

    if (map_points.size() == 0) {
        RCLCPP_INFO_STREAM(get_logger(), "[ORB-SLAM3 Mono] : map point is empty");
    }

    // TODO : Putting each point cloud in each callback -> just update each point
    //        For reduce computation time
    // TODO : Remove ground point
    sensor_msgs::msg::PointCloud2 cloud;

    cloud.header.stamp = msg_time;
    cloud.header.frame_id = world_frame_id;
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = {"x", "y", "z"};

    for (int i = 0; i < num_channels; i++) {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);


    for (unsigned int i = 0; i < cloud.width; i++) {
        if (map_points[i]) {
            // Original data
            Eigen::Vector3f pMPw = map_points[i]->GetWorldPos();
            
            // Apply world frame orientation for non-IMU cases
            if (sensor_type == ORB_SLAM3::System::MONOCULAR || sensor_type == ORB_SLAM3::System::STEREO) {
                Sophus::SE3f Tc0mp(Eigen::Matrix3f::Identity(), pMPw);
                Sophus::SE3f Twmp = Tc0w.inverse() * Tc0mp;
                pMPw = Twmp.translation();
            }

            float data_array[num_channels] = {pMPw.x(), pMPw.y(), pMPw.z()};

            memcpy(cloud_data_ptr + (i * cloud.point_step), data_array, num_channels * sizeof(float));
        }
    }

    return cloud;
}

tf2::Transform MonocularSlamNode::SE3f_to_tfTransform(Sophus::SE3f T_SE3f) {

    Eigen::Matrix3f R_mat = T_SE3f.rotationMatrix();
    Eigen::Vector3f t_vec = T_SE3f.translation();

    tf2::Matrix3x3 R_tf(
        R_mat(0, 0), R_mat(0, 1), R_mat(0, 2),
        R_mat(1, 0), R_mat(1, 1), R_mat(1, 2),
        R_mat(2, 0), R_mat(2, 1), R_mat(2, 2)
    );

    tf2::Vector3 t_tf(
        t_vec(0),
        t_vec(1),
        t_vec(2)
    );

    return tf2::Transform(R_tf, t_tf);
}

geometry_msgs::msg::PoseArray MonocularSlamNode::GetAllKfsPts(rclcpp::Time msg_time) {
    
    geometry_msgs::msg::PoseArray kfs_pts_array_;

    kfs_pts_array_.header.frame_id = world_frame_id;
    kfs_pts_array_.header.stamp = msg_time;

    vector<ORB_SLAM3::KeyFrame*> key_frames_ = m_SLAM->getMap()->GetAllKeyFrames();

    kfs_pts_array_.poses.push_back(geometry_msgs::msg::Pose());
    
    sort(key_frames_.begin(), key_frames_.end(), ORB_SLAM3::KeyFrame::lId);
    
    unsigned int n_kf_ = 0;

    for (auto kf_ : key_frames_) {

        if ( kf_->isBad() ) continue;

        // get rotation information
        cv::Mat R_ = ORB_SLAM3::Converter::toCvMat(kf_->GetRotation()).t();

        // get camera position
        cv::Mat T_ = ORB_SLAM3::Converter::toCvMat(kf_->GetCameraCenter());
        
        // convert to pose in world_ros, store it
        vector<float> q_ = ORB_SLAM3::Converter::toQuaternion(R_);

        geometry_msgs::msg::Pose c_pose;
        // TODO : check the position is correct
        c_pose.position.x = T_.at<float>(0);
        c_pose.position.y = T_.at<float>(1);
        c_pose.position.z = T_.at<float>(2);
        c_pose.orientation.x = q_[0];
        c_pose.orientation.y = q_[1];
        c_pose.orientation.z = q_[2];
        c_pose.orientation.w = q_[3];
        kfs_pts_array_.poses.push_back(c_pose);

        /* ------------------------------------------------
        add position(x,y,z) of all map points of current key_frame
        ------------------------------------------------ */
        unsigned int n_pts_id_ = kfs_pts_array_.poses.size();
        // placeholder for number of points
        kfs_pts_array_.poses.push_back(geometry_msgs::msg::Pose());
        std::set<ORB_SLAM3::MapPoint*> map_pts_ = kf_->GetMapPoints();
        unsigned int n_pts_ = 0;
        for (auto pt_ : map_pts_) {
            if (!pt_ || pt_->isBad()) continue;

            cv::Mat pt_position_ = ORB_SLAM3::Converter::toCvMat(pt_->GetWorldPos());
            if (pt_position_.empty()) continue;

            geometry_msgs::msg::Pose curr_pt;
            curr_pt.position.x = pt_position_.at<float>(0);
            curr_pt.position.y = pt_position_.at<float>(1);
            curr_pt.position.z = pt_position_.at<float>(2);
            kfs_pts_array_.poses.push_back(curr_pt);

            ++n_pts_;
        }
        kfs_pts_array_.poses[n_pts_id_].position.x = n_pts_;
        kfs_pts_array_.poses[n_pts_id_].position.y = n_pts_;
        kfs_pts_array_.poses[n_pts_id_].position.z = n_pts_;

        ++n_kf_;
    }
    kfs_pts_array_.poses[0].position.x = n_kf_;
    kfs_pts_array_.poses[0].position.y = n_kf_;
    kfs_pts_array_.poses[0].position.z = n_kf_;

    return kfs_pts_array_;
}

geometry_msgs::msg::PoseArray MonocularSlamNode::GetSingleKfPts (rclcpp::Time msg_time) {

    ORB_SLAM3::KeyFrame* kf_ = m_SLAM->getTracker()->mCurrentFrame.mpReferenceKF;

    // camera_pose, pts, ...
    geometry_msgs::msg::PoseArray kf_pts_array_;

    kf_pts_array_.header.frame_id = world_frame_id;
    kf_pts_array_.header.stamp = msg_time;

    if (kf_->isBad()) return kf_pts_array_;

    // get rotation information
    cv::Mat R_ = ORB_SLAM3::Converter::toCvMat(kf_->GetRotation()).t();
    cv::Mat T_ = ORB_SLAM3::Converter::toCvMat(kf_->GetCameraCenter());
    vector<float> q_ = ORB_SLAM3::Converter::toQuaternion(R_);

    // get camera position

    geometry_msgs::msg::Pose c_pose;
    c_pose.position.x = T_.at<float>(0);
    c_pose.position.y = T_.at<float>(1);
    c_pose.position.z = T_.at<float>(2);
    c_pose.orientation.x = q_[0];
    c_pose.orientation.y = q_[1];
    c_pose.orientation.z = q_[2];
    c_pose.orientation.w = q_[3];
    kf_pts_array_.poses.push_back(c_pose);
    
    std::vector<ORB_SLAM3::MapPoint*> map_pts_ = m_SLAM->GetTrackedMapPoints();

    for (auto pt_ : map_pts_) {
        if (!pt_ || pt_->isBad()) continue;

        cv::Mat pt_position_ = ORB_SLAM3::Converter::toCvMat(pt_->GetWorldPos());

        if (pt_position_.empty()) continue;

        geometry_msgs::msg::Pose curr_pt;
        curr_pt.position.x = pt_position_.at<float>(0);
        curr_pt.position.y = pt_position_.at<float>(1);
        curr_pt.position.z = pt_position_.at<float>(2);
        kf_pts_array_.poses.push_back(curr_pt);
    }

    return kf_pts_array_;
}