#ifndef AUX_HPP
#define AUX_HPP

#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/assign.hpp>
#include <sensor_msgs/image_encodings.h>
#include <string>
//#include <ollieRosTools/Frame.hpp> //forward declated
class Frame;

#define __SHORTFILE__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

typedef std::vector< cv::KeyPoint > KeyPoints;
typedef std::vector< cv::DMatch > DMatches;
typedef std::vector< DMatches > DMatchesKNN;
//typedef std::vector< cv::Point3d > Points3d;
//typedef std::vector< cv::Point2d > Points2d;
typedef std::vector< cv::Point2f > Points2f;
typedef Points2f Points2;
typedef std::vector< int > Ints;
typedef std::vector< bool > Bools;
typedef std::vector< float > Floats;
typedef std::vector< cv::Mat > Mats;
typedef cv::Ptr<Frame> FramePtr;

const float toRad = M_PI/180;
const float toDeg = 180/M_PI;

namespace OVO {

void tf2RPY(const tf::Transform& T, float& R, float& P, float& Y);
void drawTextCenter(cv::Mat& img, const std::string& text, const CvScalar RGB, const float textScale, const int textThickness);


static std::map<int, std::string> COLORS= boost::assign::map_list_of
        (-1, std::string(""))
        (0, sensor_msgs::image_encodings::BGR8)
        (1, sensor_msgs::image_encodings::RGB8)
        (2, sensor_msgs::image_encodings::MONO8)
        (3, sensor_msgs::image_encodings::YUV422);

cv::Mat getRosImage(const sensor_msgs::ImageConstPtr& msg, int colorId = 0);


// Draw a number onto an image
void putInt(cv::Mat& img,  const float nr, const cv::Point& p, const CvScalar& col, const bool round, const std::string& str,const std::string& post="");



}

#endif // AUX_HPP
