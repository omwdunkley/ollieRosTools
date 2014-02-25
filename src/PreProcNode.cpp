/****************************************************
  TODO:
  ****************************************************/


#include "ollieRosTools/PreProcNode.hpp"
#include <boost/bind.hpp>



namespace enc = sensor_msgs::image_encodings;


/// Initialise ROS Node
PreProcNode::PreProcNode(ros::NodeHandle& _n):
    n(_n),
    im_transport(_n){


    /// Subscribe to Topics
    input_topic = n.resolveName("image");
    //input_topic = "/image_raw";
    sub_camera = im_transport.subscribeCamera(input_topic, 1, &PreProcNode::incoming_camera, this);
    //sub_image = im_transport.subscribe(input_topic, 1, &PreProcNode::incoming_image, this);

    /// Publish to Topics
    pub_imageFlow  = im_transport.advertise("/preproc"+input_topic, 1);
    
/// Dynamic Reconfigure
    node_on = true;
    dynamic_reconfigure::Server<ollieRosTools::PreProcNode_paramsConfig>::CallbackType f;
    f = boost::bind(&PreProcNode::setParameter, this,  _1, _2);
    srv.setCallback(f);

    ROS_INFO("Starting <%s> node with <%s> as image source", /*ros::this_node::getNamespace().c_str(),*/ ros::this_node::getName().c_str(), input_topic.c_str());
}


PreProcNode::~PreProcNode() {
    ROS_INFO("Shutting <%s> node down", ros::this_node::getName().c_str());
}




/// PINHOLE
void PreProcNode::incoming_camera(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& caminfo){

    /// Extract frame and time stamps
    time_s0 = ros::WallTime::now();
    img_time = msg->header.stamp;
    img_frame = caminfo->header.frame_id;

    if (last_time>img_time){
        sub_tf.clear();
        ROS_WARN("Detected negative time jump, resetting TF buffer");
    }
    last_time = img_time;

    // Update camera model
    if (caminfo->distortion_model == ""){
        sensor_msgs::CameraInfo caminfo2 = *caminfo;
        caminfo2.distortion_model = "plumb_bob"; //some times thsi data is missing...
        camModel.fromCameraInfo(caminfo2);
        frameFactory->updateCameraMatrix(camModel);
    } else {
        camModel.fromCameraInfo(caminfo);
        frameFactory->updateCameraMatrix(camModel);
    }


    /// Extract image and get IMU transform
    cv::Mat img;
    if (!getImage(msg, img)){
        return;
    }

    /// Create and process frame
    Frame frame = frameFactory->getFrame(img);


    processFrame(frame);
}




void PreProcNode::processFrame(Frame& frame){

    vo.processFrame(frame);
    publishAllPoses();
    publishAllMarkers();

    double time_sum = (ros::WallTime::now()-time_s0).toSec();
    if (pub_imageFlow.getNumSubscribers()>0){
        cv_bridge::CvImage cvi;
        cvi.header.stamp = img_time;
        cvi.header.frame_id = img_frame;
        cvi.encoding = "bgr8";
        cvi.image = vo.getFlowImg();
        time_sum = (ros::WallTime::now()-time_s0).toSec();
        putInt(cvi.image, time_sum*1000. , cv::Point(10,cvi.image.rows-1*25), CV_RGB(255,20,255), false, "S:");
        putInt(cvi.image, 1./time_sum , cv::Point(70,cvi.image.rows-1*25), CV_RGB(255,20,255), false, "@","hz");
        pub_imageFlow.publish(cvi.toImageMsg());
    }


}



bool PreProcNode::getImage(const sensor_msgs::ImageConstPtr& msg, cv::Mat& img){
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR_STREAM_THROTTLE(1,"Failed to understand incoming image:" << e.what());
        return false;
    }
    img = cv_ptr->image;
    return true;
}

bool PreProcNode::getImage(const sensor_msgs::Image& msg, cv::Mat& img){
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR_STREAM_THROTTLE(1,"Failed to understand incoming image:" << e.what());
        return false;
    }
    img = cv_ptr->image;
    return true;
}





ollieRosTools::PreProcNode_paramsConfig&  PreProcNode::setParameter(ollieRosTools::PreProcNode_paramsConfig &config, uint32_t level){
    ROS_INFO("Setting PreProcNode param");

        node_on = config.node_on;

        config = frameFactory->setParameter(config, level);
        config = vo.setParameter(config, level);

        publishAllPoses();
        publishAllMarkers();

        return config;

}
