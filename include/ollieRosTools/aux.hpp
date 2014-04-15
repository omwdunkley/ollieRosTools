#ifndef AUX_HPP
#define AUX_HPP

extern bool USEIMU;


#include <string>
#include <boost/assign.hpp>

#include <Eigen/Core>
#include <opengv/types.hpp>
#include <opencv2/opencv.hpp>

#include <tf/tf.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>

//#include <ollieRosTools/Landmark.hpp>
//#include <ollieRosTools/Frame.hpp>
class Landmark;
class Frame;


#define __SHORTFILE__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

// 3d
typedef opengv::bearingVector_t Bearing;
typedef opengv::bearingVectors_t Bearings;
typedef opengv::point_t Point3d;
typedef opengv::points_t Points3d;
typedef Eigen::Affine3d Pose;

// 2d
typedef std::vector< cv::KeyPoint > KeyPoints;
typedef std::vector< cv::Point2d > Points2d;
typedef std::vector< cv::Point2f > Points2f;
typedef Points2d Points2;

// Associations
typedef std::vector< cv::DMatch > DMatches;
typedef std::vector< DMatches > DMatchesKNN;
typedef std::vector< bool > Bools;
typedef std::vector< int > Ints;
typedef std::vector< float > Floats;
typedef std::vector< double > Doubles;



// Containers
typedef std::vector< cv::Mat > Mats;
typedef cv::Ptr<Landmark> LandmarkPtr;
typedef std::deque<LandmarkPtr> LandMarkPtrs;

typedef cv::Ptr<Frame> FramePtr;
typedef std::deque<FramePtr> FramePtrs;




const float toRad = M_PI/180;
const float toDeg = 180/M_PI;



namespace OVO {

    enum BEARING_ERROR {BVERR_OneMinusAdotB, // 1-A.B
                    BVERR_ATAN2,             // ATAN2(||AxB||, A.B)
                    BVERR_NormAminusB,       // ||A-B||
                    BVERR_SUM_AminusBSqr     // sum((A-B)**2
                   };

    const BEARING_ERROR DEFAULT_BV_ERROR = BVERR_OneMinusAdotB;

    /// TODO: put this in a look up table
    CvScalar getColor(const float range_min, const float range_max, const float depth, bool reverse = false);

    void tf2RPY(const tf::Transform& T, double& R, double& P, double& Y);
    void drawTextCenter(cv::Mat& img, const std::string& text, const CvScalar RGB, const float textScale, const int textThickness);



    // Removes rows of Mat if index is not in ind. Can also change the order of elements.
    void matReduceInd (const cv::Mat& matIn, cv::Mat& matOut, const Ints& ind);
    void matReduceInd (cv::Mat& matInOut, const Ints& ind);
    void matReduceInd (const Eigen::MatrixXd& bvm1, Bearings& bv1, const Ints& ind);

    void match2ind(const DMatches& ms, Ints& query, Ints& train);

    static std::map<int, std::string> COLORS= boost::assign::map_list_of
            (-1, std::string(""))
            (0, sensor_msgs::image_encodings::BGR8)
            (1, sensor_msgs::image_encodings::RGB8)
            (2, sensor_msgs::image_encodings::MONO8)
            (3, sensor_msgs::image_encodings::YUV422);

    cv::Mat getRosImage(const sensor_msgs::ImageConstPtr& msg, int colorId = 0);

    visualization_msgs::Marker getPointsMarker(const Points3d& worldPoints);



    // Draw a number onto an image
    void putInt(cv::Mat& img,  const float nr, const cv::Point& p, const CvScalar& col, const bool round, const std::string& str,const std::string& post="");

    // align bearing vectors
    void alignedBV (const Eigen::MatrixXd& bvm1, const Eigen::MatrixXd& bvm2, const DMatches& ms, Bearings& bv1, Bearings& bv2);


    void transformPoints(const Pose& transform, Points3d& points);

    /// points in world frame, model in world frame, bv in model frame
    Eigen::VectorXd reprojectErrPointsVsBV(const Pose& model, const Points3d& points,const Bearings& bv, const BEARING_ERROR method = DEFAULT_BV_ERROR);
    /// Same as above but points are already in frame of bv
    Eigen::VectorXd reprojectErrPointsVsBV(const Points3d& points, const Bearings& bv, const BEARING_ERROR method = DEFAULT_BV_ERROR);
    /// Computes an error given an angular difference (between two bearing vectors)
    double angle2error(const double angleDeg, const BEARING_ERROR method = DEFAULT_BV_ERROR );
    /// Compute the error between bearing vectors
    double errorBV(const Bearing bv1, const Bearing bv2, const BEARING_ERROR method = DEFAULT_BV_ERROR );
    /// returns the error given px dist on image plane with focal length f
    double px2error(const double px, const double horiFovDeg = 110., const double width = 720, const BEARING_ERROR method = DEFAULT_BV_ERROR);


    template <class T, class T2> void vecAlignMatch (const T& vec1in, const T2& vec2in,
                                                     T& vec1out,      T2& vec2out,
                                                     const DMatches& ms){
        vec1out.clear();
        vec2out.clear();
        vec1out.reserve(ms.size());
        vec2out.reserve(ms.size());
        for(uint i=0;i<ms.size(); ++i){
            vec1out.push_back(vec1in[ms[i].queryIdx]);
            vec2out.push_back(vec2in[ms[i].trainIdx]);
        }
    }

    //reduce vector using only indicies. eg [z a b c d e f],[1 2 5 3] -> [a b e c]
    template <class T> void vecReduceInd (const T& vecIn, T& vecOut, const Ints& ind){
        vecOut.clear();
        vecOut.reserve(ind.size());
        for(uint i=0;i<ind.size(); ++i){
            vecOut.push_back(vecIn[ind[i]]);
        }
    }
    // same as above but replaces contents
    template <class T> void vecReduceInd (T& vecInOut, const Ints& ind){
        T temp;
        temp.reserve(ind.size());
        for(uint i=0;i<ind.size(); ++i){
            temp.push_back(vecInOut[ind[i]]);
        }
        std::swap(vecInOut, temp);
    }


    cv::Mat rotateImage(const cv::Mat& in, const double angleRad, const int interpolation=CV_INTER_LINEAR, const double scale=1.0, const double threshDeg=0.1);

}

#endif // AUX_HPP
