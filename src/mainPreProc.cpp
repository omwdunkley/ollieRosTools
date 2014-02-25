#include <ros/ros.h>
#include "ollieRosTools/PreProcNode.hpp"

//Initialize ROS node.
int main(int argc, char** argv){
    ros::init(argc, argv, "PreProcessingNode");
	ros::NodeHandle n;

    PreProcNode node(n);

	ros::spin();	
	return 0;
}
