
#include <ros/ros.h>
#include "ollieRosTools/VoNode.hpp"

//Initialize ROS node.
int main(int argc, char** argv){
    ros::init(argc, argv, "VoNode");
    ros::NodeHandle n("~");

    VoNode node(n);

    ros::spin();
    return 0;
}
