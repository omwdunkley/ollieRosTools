/****************************************************
  TODO:
  ****************************************************/


#include "ollieRosTools/PreProcNode.hpp"
#include <boost/bind.hpp>
#include <cmath>

namespace enc = sensor_msgs::image_encodings;


/// Initialise ROS Node
PreProcNode::PreProcNode(ros::NodeHandle& _n):
    n(_n),
    imTransport(_n),
    timeAlpha(0.95),
    timeAvg(0){


    /// Subscribe to Topics
    inputTopic = n.resolveName("image");
    subImage = imTransport.subscribe(inputTopic, 1, &PreProcNode::incomingImage, this);

    /// Publish to Topics
    pubCamera  = imTransport.advertiseCamera("/preproc"+inputTopic, 1);

    colors[-1] = ""; //passthrough
    colors[ 0] = enc::BGR8;
    colors[ 1] = enc::RGB8;
    colors[ 2] = enc::MONO8;
    colors[ 3] = enc::YUV422;
    
    /// Dynamic Reconfigure
    nodeOn = true;
    dynamic_reconfigure::Server<ollieRosTools::PreProcNode_paramsConfig>::CallbackType f;
    f = boost::bind(&PreProcNode::setParameter, this,  _1, _2);
    srv.setCallback(f);

    ROS_INFO("Starting <%s> node with <%s> as image source", /*ros::this_node::getNamespace().c_str(),*/ ros::this_node::getName().c_str(), inputTopic.c_str());
}


PreProcNode::~PreProcNode() {
    ROS_INFO("Shutting <%s> node down", ros::this_node::getName().c_str());
}


/// PINHOLE
void PreProcNode::incomingImage(const sensor_msgs::ImageConstPtr& msg){

    if (pubCamera.getNumSubscribers()>0){

        /// Measure HZ, Processing time, Image acquisation time
        ros::WallTime time_s0 = ros::WallTime::now();

        /// Get CV Image
        cv_bridge::CvImageConstPtr cvPtr;
        //cv_bridge::CvImagePtr cvPtr;
        try {
            //cvPtr = cv_bridge::toCvShare(msg, enc::MONO8);
            cvPtr = cv_bridge::toCvShare(msg, colors[colorId]);
            //cvPtr = cv_bridge::toCvCopy(msg, enc::MONO8);
            //cvPtr = cv_bridge::toCvCopy(msg, enc::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR_STREAM_THROTTLE(1,"Failed to understand incoming image:" << e.what());
            return;
        }

        /// PreProcess Frame
        cv::Mat image = preproc.process(cvPtr->image);

        /// PTAM Rectification
        cv::Mat imageRect;
        sensor_msgs::CameraInfoPtr camInfoPtr;
        camModel.rectify(image, imageRect, camInfoPtr);


        /// Send out
        cv_bridge::CvImage cvi;
        cvi.header.stamp = msg->header.stamp;
        cvi.header.seq = msg->header.seq;
        cvi.header.frame_id = msg->header.frame_id;
        //cvi.encoding = enc::MONO8;
        cvi.encoding = colors[colorId];
        cvi.image = imageRect;
        camInfoPtr->header = cvi.header;
        pubCamera.publish(cvi.toImageMsg(), camInfoPtr);

        // Compute running average of processing time
        timeAvg = timeAvg*timeAlpha + (1.0 - timeAlpha)*(ros::WallTime::now()-time_s0).toSec();
        ROS_INFO_THROTTLE(1, "Processing Time: %.1fms", timeAvg*1000.);
    } else {
        //ROS_INFO_THROTTLE(5, "Not processing images as not being listened to");
    }

}






ollieRosTools::PreProcNode_paramsConfig&  PreProcNode::setParameter(ollieRosTools::PreProcNode_paramsConfig &config, uint32_t level){
    ROS_INFO("Setting PreProcNode param");

    /// Turn node on and off when settings change
    if (nodeOn!=config.nodeOn){
        nodeOn = config.nodeOn;
        if (nodeOn){
            // Node was just turned on
            subImage = imTransport.subscribe(inputTopic, 1, &PreProcNode::incomingImage, this);
            ROS_INFO("Node On");
        } else {
            // Node was just turned off
            subImage.shutdown();
            ROS_INFO("Node Off");
        }

    }

    colorId = config.color;
    config = preproc.setParameter(config, level);
    config = camModel.setParameter(config, level);
    return config;

}
