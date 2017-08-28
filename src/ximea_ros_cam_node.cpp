#include <nodelet/loader.h>
#include <ros/ros.h>
#include <string>

// Nodelet loading
int main(int argc, char** argv) {
    // Init node
    ros::init(argc, argv, "ximea_ros_cam_node");

    // Create a new instance of your nlp
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();

    // Namespace name / library name loading for nodelet
    nodelet.load(
        nodelet_name,
        "ximea_ros_cam/ximea_ros_cam",
        remap,
        nargv);

    // Run nodelet
    ros::spin();

    return 0;
}
