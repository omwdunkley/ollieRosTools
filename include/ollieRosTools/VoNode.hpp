
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef VONODE_HPP_
#define VONODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <string>
#include <map>


#include <opencv2/opencv.hpp>

#include <ros/package.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>


#include <ollieRosTools/VoNode_paramsConfig.h>
#include <ollieRosTools/Frame.hpp>
#include <ollieRosTools/Odometry.hpp>
#include <ollieRosTools/aux.hpp>


/*****************************************************************************
 ** Class
 *****************************************************************************/


class VoNode{
	public:
        VoNode(ros::NodeHandle& _n);
        ~VoNode();

	private:

        cv::Ptr<CameraATAN> cameraModel;
        cv::Ptr<Detector> detector;
        cv::Ptr<PreProc> preproc;

        /// ROS specific stuff
        ros::NodeHandle& n;
        image_transport::ImageTransport imTransport;
        image_transport::CameraPublisher pubCamera;
        image_transport::Publisher pubImage;
        image_transport::Subscriber subImage;
        ros::Publisher pubMarker;
        tf::TransformBroadcaster pubTF;
        tf::TransformListener subTF;

        /// Dynamic Reconfigure
        ollieRosTools::VoNode_paramsConfig configLast;
        dynamic_reconfigure::Server<ollieRosTools::VoNode_paramsConfig> srv;        

        /// Members
        float timeAlpha;
        ros::Time lastTime; // Keep track of time to detect loops in bag files
        Odometry odometry;


        /// Dynamic parameters
        bool nodeOn;
        float timeAvg;
        int colorId;
        ros::Duration imgDelay;

        /// Display stuff
        void publishStuff(){
            /// TODO
//            FramePtrs kfs = odometry.getKeyFrames();
//            for(uint i=0; i<kfs.size(); ++i){
//                pubTF.sendTransform(kfs[i]->getStampedTransform());
//            }
//            pubTF.sendTransform(odometry.getLastFrame()->getStampedTransform());
//            pubMarker.publish(kfs.back()->getWorldPointsMarker(0,1.2,0.5));
//            pubMarker.publish(odometry.getLastFrame()->getWorldPointsMarker(1,0.8,1.0));
            ROS_ERROR("NOT IMPLEMENTED VoNode.publishStuff()");
        }


        /// Parameters
        std::string inputTopic;

        /// Callbacks
        void incomingImage(const sensor_msgs::ImageConstPtr& msg);
        ollieRosTools::VoNode_paramsConfig&  setParameter(ollieRosTools::VoNode_paramsConfig &config, uint32_t level);

        /// Utility Functions
        void initImu2Cam();




};

#endif
