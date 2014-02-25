#ifndef AUX_HPP
#define AUX_HPP


#include <vector>
#include <deque>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <sstream>
#include <iomanip>
#include <string.h>
#include <opengv/types.hpp>

#define __SHORTFILE__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)


const std::string world_frame = "/world";
//const std::string imu_frame = "/cf_q";


typedef std::vector< cv::KeyPoint > KeyPoints;
typedef std::vector< cv::DMatch > DMatches;
typedef std::vector< DMatches > DMatchesKNN;
typedef std::vector< cv::Point3d > Points3d;
typedef std::vector< cv::Point2d > Points2d;
typedef std::vector< cv::Point2f > Points2f;
typedef std::vector< int > Ints;
typedef std::vector< bool > Bools;
typedef std::vector< double > Doubles;
typedef std::vector< cv::Mat > Mats;

//typedef std::deque< cv::Mat > ImagesQ;
//typedef std::deque< cv::Mat > DescriptionsQ;
//typedef std::deque< KeyPoints > KeypointsQ;
//typedef std::deque< tf::Transform > TransformsQ;
//typedef std::deque< DMatches > MatchesQ;
//typedef std::deque< Points3d > UnitFeatsQ;

const double toRad = M_PI/180;
const double toDeg = 180/M_PI;

/// HELPERS
void tf2RPY(const tf::Transform& T, double& R, double& P, double& Y);

double tf2Roll(const tf::Transform& T);


// Draw a number onto an image
void putInt(cv::Mat& img,  const double nr, const cv::Point& p, const CvScalar& col, const bool round, const std::string& str,const std::string& post="");

// Draw text in middle of image
void drawTextCenter(cv::Mat& img, const std::string& text, const CvScalar RGB, const float textScale=1.0, const int textThickness=1.0);

// Print TF
void printRPYXYZ(const tf::Transform& t, const std::string str="");





//template <class T> void vecReduceInd (const std::vector<T>& vec, std::vector<T> out, const std::vector<int>& ind);

//// Takes two vectors and aligns then
//template <class T> void vecAlignMatch (const std::vector<T>& vec1in, const std::vector<T>& vec2in,
//                                             std::vector<T>& vec1out,      std::vector<T>& vec2out,
//                                       const DMatches& ms);




//template <class T> void vecReduceInd (const T& vecIn, T& vecOut, const std::vector<int>& ind);

//// Takes two vectors and aligns then
//template <class T> void vecAlignMatch (const T& vec1in, const T& vec2in,
//                                             T& vec1out,      T& vec2out,
//                                       const DMatches& ms);


//reduce vector using only indicies. eg [z a b c d e f],[1 2 5 3] -> [a b e c]
template <class T> void vecReduceInd (const T& vecIn, T& vecOut, const Ints& ind){
    vecOut.reserve(ind.size());
    for(uint i=0;i<ind.size(); ++i){
        vecOut.push_back(vecIn[ind[i]]);
    }
}
template<typename T> void vecRemoveInd(const std::vector<T>& vecIn, std::vector<T>& vecOut, Ints& ind)
{
    if(ind.empty()){
        vecOut = vecIn;
        return;
    }

    vecOut.clear();
    vecOut.reserve(vecIn.size() - ind.size());

    std::sort(ind.begin(), ind.end());

    // new we can assume there is at least 1 element to delete. copy blocks at a time.
    typename std::vector<T>::const_iterator itBlockBegin = vecIn.begin();
    for(Ints::const_iterator it = ind.begin(); it != ind.end(); ++ it)
    {
        typename std::vector<T>::const_iterator itBlockEnd = vecIn.begin() + *it;
        if(itBlockBegin != itBlockEnd)
        {
            std::copy(itBlockBegin, itBlockEnd, std::back_inserter(vecOut));
        }
        itBlockBegin = itBlockEnd + 1;
    }

    // copy last block.
    if(itBlockBegin != vecIn.end())
    {
        std::copy(itBlockBegin, vecIn.end(), std::back_inserter(vecOut));
    }
}

// Takes two vectors and aligns then
template <class T> void vecAlignMatch (const T& vec1in, const T& vec2in,
                                             T& vec1out,      T& vec2out,
                                       const DMatches& ms){
    vec1out.reserve(ms.size());
    vec2out.reserve(ms.size());
    for(uint i=0;i<ms.size(); ++i){
        vec1out.push_back(vec1in[ms[i].queryIdx]);
        vec2out.push_back(vec2in[ms[i].trainIdx]);
    }
}

template <class T, class T2> void vecAlignMatch (const T& vec1in, const T2& vec2in,
                                                       T& vec1out,      T2& vec2out,
                                       const DMatches& ms){
    vec1out.reserve(ms.size());
    vec2out.reserve(ms.size());
    for(uint i=0;i<ms.size(); ++i){
        vec1out.push_back(vec1in[ms[i].queryIdx]);
        vec2out.push_back(vec2in[ms[i].trainIdx]);
    }
}


std::vector<int> alignedMatch(const int s);









void matAlignMatch (const cv::Mat& vec1in, const cv::Mat& vec2in,
                   cv::Mat& vec1out,      cv::Mat& vec2out,
                    const DMatches& ms);
void matReduceInd (const cv::Mat& vecIn, cv::Mat& vecOut, const Ints& ind);

void match2ind(const DMatches& ms, Ints& query, Ints& train);


void rotatePoint(cv::Point2d& inout, const cv::Point2d& center, const double angle_radians);
void rotatePoint(cv::Point2f& inout, const cv::Point2f& center, const double angle_radians);

void rotateKeyPoint(cv::KeyPoint& inout, const cv::Point2d& center, const double angle_radians);
void rotateKeyPoints(const KeyPoints kps_in, KeyPoints& kps_out, const cv::Point2d& center, const double angle_radians);
cv::Mat rotateImage(const cv::Mat& in, const cv::Point2d &center, const double angleRad, const int interpolation);










tf::Transform eigen2tf(const opengv::transformation_t& t);

void tf2eigen(const tf::Transform& transform, opengv::translation_t& t, opengv::rotation_t& r);









#endif // AUX_HPP
