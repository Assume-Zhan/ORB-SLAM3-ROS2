#include "monocular-slam-node.hpp"


int main(int argc, char **argv) {
    if(argc < 3) {

        // Not received required files ( vocabulary, camera config )
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\nUsage: ros2 run orbslam mono path_to_vocabulary path_to_settings \n");
        return 1;
    }

    rclcpp::init(argc, argv);

    bool visualization = true;
    
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, visualization);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "============================ \n");

    auto node = std::make_shared<MonocularSlamNode>(&SLAM);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "============================ \n");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
