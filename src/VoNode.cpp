/****************************************************
  TODO:
  ****************************************************/

#include <ollieRosTools/VoNode.hpp>
#include <boost/bind.hpp>
#include <cmath>


/// ///// DELETE LATER

#include <opencv2/core/core.hpp>
#include <tf/tf.h>

#include <boost/lexical_cast.hpp> //boost::lexical_cast<std::string>(i);





/// Initialise ROS Node
VoNode::VoNode(ros::NodeHandle& _n):
    n(_n),
    imTransport(_n),
    timeAlpha(0.95),
    timeAvg(0),
    imgDelay(0),
    repeatOn(false){

    /// Set default values
    n.param("image", inputTopic, std::string("/cf/cam/image_raw"));
    n.param("useIMU", useIMU, true);
    n.param("imuFrame", imuFrame, std::string("/cf"));
    n.param("worldFrame", worldFrame, std::string("/world"));
    // Cam frame is specified from the header in the cam msg

    /// Subscribe to Topics    
    subImage = imTransport.subscribe(inputTopic, 1, &VoNode::incomingImage, this);

    /// Publish to Topics
    pubCamera  = imTransport.advertiseCamera("/vo" + inputTopic, 1);

    /// Dynamic Reconfigure
    nodeOn = true;
    dynamic_reconfigure::Server<ollieRosTools::VoNode_paramsConfig>::CallbackType f;
    f = boost::bind(&VoNode::setParameter, this,  _1, _2);
    srv.setCallback(f);


    ROS_INFO("Starting <%s> node with <%s> as image source", /*ros::this_node::getNamespace().c_str(),*/ ros::this_node::getName().c_str(), inputTopic.c_str());
    if (useIMU){
        ROS_INFO("Using <%s> as imu frame", imuFrame.c_str());
    } else {
        ROS_INFO("NOT Using imu!");
    }
    ROS_INFO("Using <%s> as world frame",  worldFrame.c_str());
}


VoNode::~VoNode() {
    ROS_INFO("Shutting <%s> node down", ros::this_node::getName().c_str());
}


/// PINHOLE
void VoNode::incomingImage(const sensor_msgs::ImageConstPtr& msg){

    /// Measure HZ, Processing time, Image acquisation time
    ros::WallTime time_s0 = ros::WallTime::now();

    /// Check TIME
    if (lastTime>msg->header.stamp){
        ROS_WARN("Detected negative time jump, resetting TF buffer");
        subTF.clear();
    }
    lastTime = msg->header.stamp;


    /// Get Image
    cv_bridge::CvImageConstPtr cvPtr;
    try {
        cvPtr = cv_bridge::toCvShare(msg, OVO::COLORS[colorId]);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR_STREAM_THROTTLE(1,"Failed to understand incoming image:" << e.what());
        return;
    }

    /// PreProcess Image



    /// GET IMU
    tf::StampedTransform imuStamped;

    if (useIMU){
        try{
            // sent out by the crazyflie driver driver.py
            subTF.waitForTransform(imuFrame, worldFrame, msg->header.stamp-imgDelay, ros::Duration(0.1) );
            subTF.lookupTransform(imuFrame, worldFrame, msg->header.stamp-imgDelay, imuStamped);
        } catch(tf::TransformException& ex){
            ROS_ERROR_THROTTLE(1,"TF exception. Could not get flie IMU transform: %s", ex.what());
            if (repeatOn){
                repeatOn=false;
                subImage = imTransport.subscribe(inputTopic, 1, &VoNode::incomingImage, this);
            }
            return;
        }
    } else {
        imuStamped.setRotation(tf::Quaternion(1.0, 0.0, 0.0, 0.0));
    }

    /// Make Frame
    cv::Ptr<Frame> frame(new Frame(cvPtr->image, imuStamped));


    frame->getDescriptors();

    cv::Mat drawImg = frame->getVisualImage();


//    /// ROTATE KEYPOINTS
//    std::vector<cv::KeyPoint> kpsRect = camModel.rectifyPoints(kps);
//    std::vector<cv::KeyPoint> kpsR;
//    if (useIMU){
//        kpsR = camModel.rotatePoints(kpsRect, -r, false);
//    } else {
//        kpsR = kps;
//    }

//    cv::Mat img = cv::Mat::zeros(image.size(), CV_8UC3);
//    cv::Mat imgRect;

//    int width=img.size().width;
//    int height=img.size().height;
//    for(int i=0; i<height; i+=height/8){
//        cv::line(img,cv::Point(0,i),cv::Point(width,i),CV_RGB(255,255,255),2);
//    }

//    for(int i=0; i<width; i+=width/8){
//        cv::line(img,cv::Point(i,0),cv::Point(i,height),CV_RGB(255,255,255), 2);
//    }


//    camModel.rectify(img, imgRect, camInfoPtr);
//    cv::imshow("grid", imgRect); cv::waitKey(10);









    /// BEARING VECTORS
    //    Eigen::MatrixXf bv;
    //    camModel.bearingVectors(kps, bv);
    //    const tf::Quaternion q(1,0,0,0);
    //    for (uint i=0; i<kps.size(); ++i){
    //        pubTF.sendTransform(tf::StampedTransform(tf::Transform(q,tf::Vector3(bv(i,0),bv(i,1),bv(i,2))), ros::Time::now(), "/cam", "/F"+boost::lexical_cast<std::string>(i)));
    //    }

    /// ADD TO MAP

    /// GET POSE

    /// OUTPUT POSE


    /// Compute running average of processing time
    timeAvg = timeAvg*timeAlpha + (1.0 - timeAlpha)*(ros::WallTime::now()-time_s0).toSec();
    ROS_INFO("Processing Time: %.1fms", timeAvg*1000.);


    /// publish debug image
    if (pubCamera.getNumSubscribers()>0){
        sensor_msgs::CameraInfoPtr camInfoPtr = frame->getCamInfo();

//        if (imageRect.channels() != image.channels()){
//            cv::cvtColor(imageRect, imageRect,CV_BGR2GRAY);
//        }

        /// Send out
        cv_bridge::CvImage cvi;
        cvi.header.stamp = msg->header.stamp;
        cvi.header.seq = msg->header.seq;
        cvi.header.frame_id = msg->header.frame_id;
        //cvi.encoding = enc::MONO8;
        cvi.encoding = sensor_msgs::image_encodings::BGR8;// OVO::COLORS[colorId];
        //cvi.image = imageRect;
        cvi.image = drawImg;
        camInfoPtr->header = cvi.header;
        pubCamera.publish(cvi.toImageMsg(), camInfoPtr);



        //ROS_INFO_THROTTLE(1, "Processing Time: %.1fms", timeAvg*1000.);

    } else {
        //ROS_INFO_THROTTLE(5, "Not processing images as not being listened to");
    }

    if (repeatOn!=configLast.repeatInput){
        if (configLast.repeatInput){
            subImage.shutdown();
            ROS_INFO("Repeat on");
        } else {
            ROS_INFO("Repeat off");
            subImage = imTransport.subscribe(inputTopic, 1, &VoNode::incomingImage, this);
        }
        repeatOn = configLast.repeatInput;
    }
    if (repeatOn){
        ROS_INFO_THROTTLE(1,"Repeating Input");
        ros::spinOnce(); // check for changes in dynamic reconfigure
        incomingImage(msg);
    }

}




ollieRosTools::VoNode_paramsConfig&  VoNode::setParameter(ollieRosTools::VoNode_paramsConfig &config, uint32_t level){
    ROS_INFO("Setting VoNode param");

    /// Turn node on and off when settings change
    if (nodeOn!=config.nodeOn){
        nodeOn = config.nodeOn;
        if (nodeOn){
            // Node was just turned on
            subImage = imTransport.subscribe(inputTopic, 1, &VoNode::incomingImage, this);
            ROS_INFO("Node On");
        } else {
            // Node was just turned off
            subImage.shutdown();
            ROS_INFO("Node Off");
        }
    }

    imgDelay = ros::Duration(config.imgDelay);
    colorId = config.color;

    Frame::setParameter(config, level);

    configLast = config;
    return config;

}
