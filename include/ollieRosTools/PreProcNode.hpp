
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
#include <ollieRosTools/Aux.hpp>
/*****************************************************************************
 ** Class
 *****************************************************************************/


class PreProcNode{



	public:
        PreProcNode(ros::NodeHandle& _n);
        ~PreProcNode();

	private:

        ros::NodeHandle& n;
        image_transport::ImageTransport im_transport;
        image_transport::Publisher pub_imageFlow;

        ollieRosTools::PreProcNode_paramsConfig config_last;
        dynamic_reconfigure::Server<ollieRosTools::PreProcNode_paramsConfig> srv;        
        ollieRosTools::PreProcNode_paramsConfig&  setParameter(ollieRosTools::PreProcNode_paramsConfig &config, uint32_t level);


        std::string input_topic;

        image_transport::CameraSubscriber sub_camera;
        void incoming_camera(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& caminfo);


        void processFrame(Frame& frame);

        // Helper functions
        bool getImage(const sensor_msgs::ImageConstPtr& msg, cv::Mat& img);
        //bool getImage(const sensor_msgs::Image& msg, cv::Mat& img);

        image_geometry::PinholeCameraModel camModel;

        PreProc preproc;
        bool node_on;






};

#endif
