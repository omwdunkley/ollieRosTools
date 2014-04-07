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
    repeatOn(false)
    {

    /// Set default values
    n.param("image", inputTopic, std::string("/cf/cam/image_raw"));
    n.param("useIMU", USEIMU, true);
    n.param("imuFrame", imuFrame, std::string("/cf0"));
    n.param("worldFrame", worldFrame, std::string("/world"));
    // Cam frame is specified from the header in the cam msg



    /// Subscribe to Topics    
    subImage = imTransport.subscribe(inputTopic, 1, &VoNode::incomingImage, this);

    /// Publish to Topics
    pubCamera  = imTransport.advertiseCamera("/vo/image_raw", 1);
    pubImage   = imTransport.advertise("/vo/debug_image_raw", 1);
    pubMarker  = n.advertise<visualization_msgs::Marker>("vo/worldPointsMarker", 1);

    /// Dynamic Reconfigure
    nodeOn = true;
    dynamic_reconfigure::Server<ollieRosTools::VoNode_paramsConfig>::CallbackType f;
    f = boost::bind(&VoNode::setParameter, this,  _1, _2);
    srv.setCallback(f);


    ROS_INFO("Starting <%s> node with <%s> as image source", /*ros::this_node::getNamespace().c_str(),*/ ros::this_node::getName().c_str(), inputTopic.c_str());
    if (USEIMU){
        ROS_INFO("Using <%s> as imu frame", imuFrame.c_str());
    } else {
        ROS_INFO("NOT Using imu!");
    }
    ROS_INFO("Using <%s> as world frame",  worldFrame.c_str());



//    // test color map
//    cv::Mat test = cv::Mat(100,1600,CV_8UC3);
//    for(int i=0; i<test.cols; ++i){
//        test.col(i) = OVO::getColor(100,test.cols-100,i);
//    }
//    cv::imshow("colmap",test);
//    cv::waitKey(1000);


}


VoNode::~VoNode() {
    ROS_INFO("Shutting <%s> node down", ros::this_node::getName().c_str());
}


/// PINHOLE
void VoNode::incomingImage(const sensor_msgs::ImageConstPtr& msg){
    ROS_INFO("\n");

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

    /// GET IMU
    tf::StampedTransform imuStamped;

    if (USEIMU){
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

    ROS_INFO("NODE > PROCESSING FRAME [%d]", frame->getId());

    /// Do visual odometry
    odometry.update(frame);



    /// Compute running average of processing time
    timeAvg = timeAvg*timeAlpha + (1.0 - timeAlpha)*(ros::WallTime::now()-time_s0).toSec();




    ROS_INFO_STREAM("NODE < FRAME ["<<frame->getId() << "] PROCESSED [" << std::setprecision(4) << timeAvg*1000. << "ms]");


    publishStuff();


    /// publish debug image
    if (pubCamera.getNumSubscribers()>0 || pubImage.getNumSubscribers()>0){
        sensor_msgs::CameraInfoPtr camInfoPtr = frame->getCamInfo();
        cv::Mat drawImg = odometry.getVisualImage();
        OVO::putInt(drawImg, timeAvg*1000., cv::Point(10,drawImg.rows-1*25), CV_RGB(200,0,200), false, "s:");

//        if (imageRect.channels() != image.channels()){
//            cv::cvtColor(imageRect, imageRect,CV_BGR2GRAY);
//        }

        /// Send out
        cv_bridge::CvImage cvi;
        cvi.header.stamp = msg->header.stamp;
        cvi.header.seq = msg->header.seq;
        cvi.header.frame_id = msg->header.frame_id;
        cvi.encoding = sensor_msgs::image_encodings::BGR8;// OVO::COLORS[colorId];
        //cvi.image = imageRect;
        cvi.image = drawImg;
        camInfoPtr->header = cvi.header;
        //pubCamera.publish(cvi.toImageMsg(), camInfoPtr);
        pubImage.publish(cvi.toImageMsg());

    }









    if (repeatOn!=configLast.repeatInput){
        if (configLast.repeatInput){
            subImage.shutdown();
            ROS_INFO("NODE = Repeat on");
        } else {
            ROS_INFO("NODE = Repeat off");
            subImage = imTransport.subscribe(inputTopic, 1, &VoNode::incomingImage, this);
        }
        repeatOn = configLast.repeatInput;
    }


    if (repeatOn){
        ROS_INFO_THROTTLE(1,"NODE = Repeating Input");
        ros::spinOnce(); // check for changes in dynamic reconfigure
        incomingImage(msg);
    }

}


ollieRosTools::VoNode_paramsConfig& VoNode::setParameter(ollieRosTools::VoNode_paramsConfig &config, uint32_t level){
    ROS_INFO("NODE > SETTING PARAM");

    /// Turn node on and off when settings change
    if (nodeOn!=config.nodeOn){
        nodeOn = config.nodeOn;
        if (nodeOn){
            // Node was just turned on
            subImage = imTransport.subscribe(inputTopic, 1, &VoNode::incomingImage, this);
            ROS_INFO("NODE = Node On");
        } else {
            // Node was just turned off
            subImage.shutdown();
            ROS_INFO("NODE = Node Off");
        }
    }


    // maxKp == 0 means dont cap size
    if (config.kp_max!=0){
        // make sure max>=min
        int minKp, maxKp;
        minKp=std::min(config.kp_min, config.kp_max);
        maxKp=std::max(config.kp_min, config.kp_max);
        config.kp_max = maxKp;
        config.kp_min = minKp;
    }

    imgDelay = ros::Duration(config.imgDelay);
    colorId = config.color;

    Frame::setParameter(config, level);
    odometry.setParameter(config, level);


    ROS_INFO("NODE < PARAM SET");
    return config;

}
