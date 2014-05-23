
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef CAMLATENCYSUB_HPP_
#define CAMLATENCYSUB_HPP_

#include <string>

#include <opencv2/opencv.hpp>

#include <ros/package.h>
#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>


/*****************************************************************************
 ** Class
 *****************************************************************************/

class CamLatencySub{
	public:
        CamLatencySub(ros::NodeHandle& _n);
        ~CamLatencySub();

	private:

        /// ROS specific stuff
        ros::NodeHandle& n;
        image_transport::Publisher pubImage;
        image_transport::Subscriber subImage;
        ros::Subscriber subSequence;
        image_transport::ImageTransport imTransport;
        int preMean;

        ros::Time tSent, tReceived;
        int tSentSeq, preSeq;

        double sCount;
        double sSum;

        /// Parameters
        std::string inputTopic;

        void reset();

        /// Callbacks
        void incomingImage(const sensor_msgs::ImageConstPtr& msg);
        void incomingSequence(const std_msgs::HeaderConstPtr& msg);

};

#endif
