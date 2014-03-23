#ifndef DETECTOR_HPP
#define DETECTOR_HPP

#include <agast/agast5_8.h>
#include <agast/agast7_12d.h>
#include <agast/agast7_12s.h>
#include <agast/oast9_16.h>
#include "akaze_features.h"
//#include <opengsurf/surflib.h>


#include <algorithm>    // std::max
#include <ollieRosTools/VoNode_paramsConfig.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/opencv.hpp>

#include <rbrief/rbrief.h>
#include <ollieRosTools/aux.hpp>
//#include <opencv2/flann/flann.hpp>




class Detector {
public:
    Detector();
    void setParameter(ollieRosTools::VoNode_paramsConfig &config, uint32_t level);

    void detect(const cv::Mat& img, KeyPoints& kps_out,const cv::Mat& mask=cv::Mat()) const;
    void extract(const cv::Mat& img, KeyPoints& kps_inout, cv::Mat& descs_out, double angleRad = 0.0) const;

    int get_detector_id() const {return detector_nr;}
    int get_extractor_id() const {return extractor_nr;}



private:

    // Cache settings to notice changes
    ollieRosTools::VoNode_paramsConfig config_pre;

    cv::Ptr< agast::AstDetector> ast_detector;
    cv::Ptr< cv::FeatureDetector> cv_detector;
    cv::Ptr< cv::DescriptorExtractor> cv_extractor;
    cv::RBriefDescriptorExtractor rBRIEF;

    cv::Ptr<cv::Algorithm> getAlgo(const int id, const float thresh);

    /// Deal with foreign detector/descriptor
    void ast_detect(const cv::Ptr<agast::AstDetector>,const cv::Mat& image, KeyPoints& keypoints, const cv::Mat& mask=cv::Mat()) const;



    int detector_nr;
    int extractor_nr;

    double kp_thresh;
    uint kp_max;
    int kp_octaves;
    int kp_octaveLayers;
    bool kp_nms;
    int kp_border;
    int kp_subPix;
    int kp_removeDouble;
    bool kp_imuRotate;
    cv::KeyPointsFilter kp_filter;

    cv::Mat KF_img;
    cv::Mat KF_desc;
    KeyPoints KF_kps;
};




#endif // DETECTOR_HPP
