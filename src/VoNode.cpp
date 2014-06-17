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
    cameraModel(new CameraATAN()),
    detector(new Detector()),
    preproc(new PreProc()),
    n(_n),
    imTransport(_n),
    timeAlpha(0.95),
    timeAvg(0),
    imgDelay(0)
    {
    Frame::setCamera(cameraModel);
    Frame::setDetector(detector);
    Frame::setPreProc(preproc);

    ROS_INFO("Starting VO node\nAvailable params:\n\t_synth:=true\n\t_image:=/image_raw\n\t_useIMU:=true\n\t_imuFrame:=/cf_attitude\n\t_camFrame:=/cam\n\t_gt:=/cf_gt (empty string = dont use for init)");



    /// Publish to Topics
    pubCamera  = imTransport.advertiseCamera("/vo/image_raw", 1);
    pubImage   = imTransport.advertise("/vo/debug_image_raw", 1);
    pubMarker  = n.advertise<visualization_msgs::Marker>("vo/worldPointsMarker", 1);
    pubTrack  = n.advertise<geometry_msgs::PoseArray>("vo/track", 1);



    /// Set default values
    n.param("synth", USE_SYNTHETIC, false);
    n.param("useIMU", USE_IMU, true);
    n.param("gt", GROUNDTRUTH_FRAME, std::string(""));


    /// Dynamic Reconfigure
    nodeOn = true;
    dynamic_reconfigure::Server<ollieRosTools::VoNode_paramsConfig>::CallbackType f;
    f = boost::bind(&VoNode::setParameter, this,  _1, _2);
    srv.setCallback(f);


    if (USE_SYNTHETIC){
        ROS_WARN("Using SYNTHETIC DATA");

        /// Init IMU transform
        initImu2Cam();

        ROS_INFO("Starting <%s> node with <SYNTHETIC DATA> as image source", ros::this_node::getName().c_str());
        subSynthetic = n.subscribe("/synthetic/frame" ,1, &VoNode::incomingSynthetic, this);


    } else {
        n.param("image", inputTopic, std::string("/image_raw"));
        n.param("imuFrame", IMU_FRAME, std::string("/cf_attitude"));
        n.param("worldFrame", WORLD_FRAME, std::string("/world"));
        n.param("camFrame", CAM_FRAME, std::string("/cam"));

        ROS_INFO("Using <%s> as world frame",  WORLD_FRAME.c_str());
        ROS_INFO("Using <%s> as camera frame",  CAM_FRAME.c_str());

        /// Init IMU transform
        initImu2Cam();

        ROS_INFO("Starting <%s> node with <%s> as image source", /*ros::this_node::getNamespace().c_str(),*/ ros::this_node::getName().c_str(), inputTopic.c_str());

        /// Subscribe to Topics
        subImage = imTransport.subscribe(inputTopic, 1, &VoNode::incomingImage, this);


    }

    if (USE_IMU){
        ROS_INFO("Using <%s> as imu frame", IMU_FRAME.c_str());
    } else {
        ROS_INFO("Not Using imu!");
    }

    std::string maskPath;
    n.param("mask", maskPath, std::string(""));
    if (maskPath.length()>0){
        ROS_INFO("Using <%s> as mask", maskPath.c_str());
        cv::Mat mask = cv::imread(maskPath, CV_LOAD_IMAGE_GRAYSCALE);
        Frame::setMask(mask);
        ROS_ASSERT_MSG(!mask.empty(), "Failed to load mask <%s>, incorrect path?", maskPath.c_str());

    } else {
        ROS_INFO("No mask Set");
    }


    SUBTF = &subTF;


}


VoNode::~VoNode() {
    ROS_INFO("Shutting <%s> node down", ros::this_node::getName().c_str());
}


/// Looks ithe IMU -> CAM transformation
void VoNode::initImu2Cam(){
    if (USE_IMU){
        tf::StampedTransform imu2cam;
        ROS_INFO("Lookingup  for [%s] -> [%s] transform...", IMU_FRAME.c_str(), CAM_FRAME.c_str());
        while(1){
            try{
                // sent out by the crazyflie driver driver.py
                subTF.lookupTransform(CAM_FRAME, IMU_FRAME, ros::Time(0), imu2cam); // get the latest
                tf::transformTFToEigen(imu2cam.inverse(), IMU2CAM);
                ROS_INFO("...IMU transform [%s] -> [%s] INVERSED and set:", IMU_FRAME.c_str(), CAM_FRAME.c_str());
                ROS_INFO_STREAM("IMU2CAM\n"<<IMU2CAM.matrix());
                break;
                ros::Rate(10).sleep();
            } catch(tf::TransformException& ex){
                ROS_ERROR_THROTTLE(3,"TF exception. Could not get IMU2CAM transform: %s", ex.what());
            }

        }
    } else {
        ROS_INFO("Not using IMU, so no [%s] -> [%s] transform needed", IMU_FRAME.c_str(), CAM_FRAME.c_str());
        IMU2CAM.setIdentity();
    }
}




void VoNode::incomingSynthetic(const ollieRosTools::synthFrameConstPtr& msg){
    ROS_INFO((std::string("\n")+OVO::colorise("============================================================================== FRAME %d ",OVO::FG_WHITE, OVO::BG_BLUE)).c_str(),msg->frameId);

    /// Measure HZ, Processing time, Image acquisation time
    ros::WallTime t0 = ros::WallTime::now();

    /// Check TIME
    if (lastTime>msg->header.stamp){
        ROS_WARN("Detected negative time jump, resetting TF buffer");
        subTF.clear();
    }
    lastTime = msg->header.stamp;
    /// Get Image
    cv_bridge::CvImageConstPtr cvPtr;
    try {
        cvPtr = cv_bridge::toCvShare(msg->img, msg, OVO::COLORS[colorId]);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR_STREAM_THROTTLE(1,"Failed to understand incoming image:" << e.what());
        return;
    }

    /// GET IMU
    tf::StampedTransform imuStamped;

    if (USE_IMU){
        try{
            // sent out by the crazyflie driver driver.py
            subTF.waitForTransform(IMU_FRAME, WORLD_FRAME, msg->header.stamp-imgDelay, ros::Duration(0.1) );
            subTF.lookupTransform(IMU_FRAME, WORLD_FRAME, msg->header.stamp-imgDelay, imuStamped);
        } catch(tf::TransformException& ex){
            ROS_ERROR_THROTTLE(1,"TF exception. Could not get flie IMU transform: %s", ex.what());
            return;
        }
    } else {
        imuStamped.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    }

    /// NEED TO SEE ABOUT THIS
    tf::StampedTransform imuRevStamped(imuStamped.inverse(), imuStamped.stamp_, imuStamped.frame_id_, imuStamped.child_frame_id_);

    /// Make Frame
    tf::Transform cam, imu;
    tf::transformMsgToTF(msg->camPose, cam);
    tf::transformMsgToTF(msg->imuPose, imu);    
    //cam = cam.inverse();
    //imu = imu.inverse();
    Frame::Ptr frame(new FrameSynthetic(cvPtr->image, imuRevStamped, msg->pts2d_noise, cam, imu, msg->camInfo));


    /// Process Frame
    ROS_INFO("NOD > PROCESSING FRAME [%d]", frame->getId());
    odometry.update(frame);

    /// Compute running average of processing time
    double time = (ros::WallTime::now()-t0).toSec();
    timeAvg = (timeAvg*timeAlpha) + (1.0 - timeAlpha)*time;

    /// Clean up and output
    ROS_INFO("NOD < FRAME [%d|%d] PROCESSED [%.1fms, Avg: %.1fms]", frame->getId(), frame->getKfId(), time*1000., timeAvg*1000.);
    publishStuff();

    /// publish debug image
    if (pubCamera.getNumSubscribers()>0 || pubImage.getNumSubscribers()>0){
        sensor_msgs::CameraInfoPtr camInfoPtr = frame->getCamInfo();
        cv::Mat drawImg = odometry.getVisualImage();
        OVO::putInt(drawImg, time*1000., cv::Point(10,drawImg.rows-1*25), CV_RGB(200,0,200), false, "s:");

//        if (imageRect.channels() != image.channels()){
//            cv::cvtColor(imageRect, imageRect,CV_BGR2GRAY);
//        }

        /// Send out
        cv_bridge::CvImage cvi;
        cvi.header.stamp = msg->header.stamp;
        cvi.header.seq = msg->header.seq;
        cvi.header.frame_id = CAM_FRAME;
        cvi.encoding = sensor_msgs::image_encodings::BGR8;// OVO::COLORS[colorId];
        //cvi.image = imageRect;
        cvi.image = drawImg;
        camInfoPtr->header = cvi.header;
        //pubCamera.publish(cvi.toImageMsg(), camInfoPtr);
        pubImage.publish(cvi.toImageMsg());
    }

}








/// PINHOLE
void VoNode::incomingImage(const sensor_msgs::ImageConstPtr& msg){

    /// Measure HZ, Processing time, Image acquisation time
    ros::WallTime t0 = ros::WallTime::now();

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

    if (USE_IMU){
        try{
            // sent out by the crazyflie driver driver.py
            subTF.waitForTransform(IMU_FRAME, WORLD_FRAME, msg->header.stamp-imgDelay, ros::Duration(0.1) );
            subTF.lookupTransform(IMU_FRAME, WORLD_FRAME, msg->header.stamp-imgDelay, imuStamped);
        } catch(tf::TransformException& ex){
            ROS_ERROR_THROTTLE(1,"TF exception. Could not get flie IMU transform: %s", ex.what());            
            return;
        }
    } else {
        imuStamped.setRotation(tf::Quaternion(1.0, 0.0, 0.0, 0.0));
    }

    ROS_INFO(OVO::colorise("\n==============================================================================",OVO::FG_WHITE, OVO::BG_BLUE).c_str());

    /// Make Frame
    ROS_ERROR("MIGHT NEED TO INVERSE IMU");
    Frame::Ptr frame(new Frame(cvPtr->image, imuStamped));

    if (frame->getQuality()>0.9 || frame->getQuality()<0){
        /// Process Frame
        ROS_INFO("NOD > PROCESSING FRAME [%d]", frame->getId());
        odometry.update(frame);
    } else {
        ROS_WARN("NOD = SKIPPING FRAME, BAD QUALITY");
    }

    /// Compute running average of processing time
    double time = (ros::WallTime::now()-t0).toSec();
    timeAvg = (timeAvg*timeAlpha) + (1.0 - timeAlpha)*time;

    /// Clean up and output
    ROS_INFO("NOD < FRAME [%d|%d] PROCESSED [%.1fms, Avg: %.1fms]", frame->getId(), frame->getKfId(), time*1000., timeAvg*1000.);


    if (frame->getQuality()>0.9 || frame->getQuality()<0){
        publishStuff();
    }

    /// publish debug image
    if (pubCamera.getNumSubscribers()>0 || pubImage.getNumSubscribers()>0){
        sensor_msgs::CameraInfoPtr camInfoPtr = frame->getCamInfo();
        cv::Mat drawImg;
        if (frame->getQuality()>0.9 || frame->getQuality()<0){
            drawImg = odometry.getVisualImage();
        } else {
            drawImg = odometry.getVisualImage(frame);
        }

        OVO::putInt(drawImg, time*1000., cv::Point(10,drawImg.rows-1*25), CV_RGB(200,0,200), false, "S:");

//        if (imageRect.channels() != image.channels()){
//            cv::cvtColor(imageRect, imageRect,CV_BGR2GRAY);
//        }

        /// Send out
        cv_bridge::CvImage cvi;
        cvi.header.stamp = msg->header.stamp;
        cvi.header.seq = msg->header.seq;
        cvi.header.frame_id = "cam_gt";//CAM_FRAME;
        cvi.encoding = sensor_msgs::image_encodings::BGR8;// OVO::COLORS[colorId];
        //cvi.image = imageRect;
        cvi.image = drawImg;
        camInfoPtr->header = cvi.header;
        pubImage.publish(cvi.toImageMsg());

        cvi.image=cvi.image.colRange(0, cvi.image.cols/2 -1);
        pubCamera.publish(cvi.toImageMsg(), camInfoPtr);

    } else {
        ROS_WARN_THROTTLE(1, "NOT DRAWING OUTPUT");
    }
}


ollieRosTools::VoNode_paramsConfig& VoNode::setParameter(ollieRosTools::VoNode_paramsConfig &config, uint32_t level){
    ROS_INFO("NOD > SETTING PARAM");


    if (USE_SYNTHETIC){
        // Hard code settings for synthetic data

        // Some hardcoded Values
        config.detector = -10;
        config.extractor = -10;



    }

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

    imgDelay = ros::Duration(config.imgDelay);
    colorId = config.color;



    preproc->setParam(config.doPreprocess,
                     config.doDeinterlace,
                     config.doEqualise,
                     config.doEqualiseColor,
                     config.kernelSize,
                     config.sigmaX,
                     config.sigmaY,
                     config.brightness,
                     config.contrast);



    cameraModel->setParams(config.zoomFactor, config.zoom,
                           config.PTAMRectify,
                           config.sameOutInSize,
                           config.width, config.height,
                           config.fx, config.fy,
                           config.cx, config.cy,
                           config.s);

    detector->setParameter(config, level);

    Frame::setParameter(config, level);

    odometry.setParameter(config, level);


    ROS_INFO("NODE < PARAM SET");
    return config;

}
