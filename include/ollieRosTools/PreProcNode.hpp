
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef PreProcNode_HPP_
#define PreProcNode_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <dynamic_reconfigure/server.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ollieRosTools/PreProc.hpp>
#include <ollieRosTools/PreProcNode_paramsConfig.h>
#include <opencv2/opencv.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <ollieRosTools/CameraATAN.hpp>
/*****************************************************************************
 ** Class
 *****************************************************************************/


class PreProcNode{



	public:
        PreProcNode(ros::NodeHandle& _n);
        ~PreProcNode();

	private:

        /// ROS specific stuff
        ros::NodeHandle& n;
        image_transport::ImageTransport imTransport;
        image_transport::CameraPublisher pubCamera;
        image_transport::Subscriber subImage;

        /// Dynamic Reconfigure
        ollieRosTools::PreProcNode_paramsConfig configLast;
        dynamic_reconfigure::Server<ollieRosTools::PreProcNode_paramsConfig> srv;        
        ollieRosTools::PreProcNode_paramsConfig&  setParameter(ollieRosTools::PreProcNode_paramsConfig &config, uint32_t level);

        /// Members
        CameraATAN camModel;
        PreProc preproc;
        bool nodeOn;
        std::string inputTopic;        
        double timeAlpha;
        double timeAvg;


        void incomingImage(const sensor_msgs::ImageConstPtr& msg);
        bool getImage(const sensor_msgs::ImageConstPtr& msg, cv::Mat& img);


};

#endif
