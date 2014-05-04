
#include <ros/ros.h>
#include <ollieRosTools/VoNode.hpp>
//#include <opencv2/gpu/gpu.hpp>

//Initialize ROS node.
int main(int argc, char** argv){
    //Eigen::initParallel();

//    if(cv::gpu::getCudaEnabledDeviceCount()) {
//        ROS_INFO("GPU AVAILABLE");
//    }


    ros::init(argc, argv, "VoNode");
    ros::NodeHandle n("~");

    VoNode node(n);

    ros::spin();
    return 0;
}
