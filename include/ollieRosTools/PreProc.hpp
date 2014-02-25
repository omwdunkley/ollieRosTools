#ifndef PREPROC_HPP
#define PREPROC_HPP

#include <rosToolsOllie/PreProcNode_paramsConfig.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rosToolsOllie/Aux.hpp>

#include <boost/format.hpp>





class PreProc {
public:
    // Construct and use path to load camera model
    PreProc();

    // Set all the options
    rosToolsOllie::PreProcNode_paramsConfig& setParameter(rosToolsOllie::PreProcNode_paramsConfig &config, uint32_t level);


    // does all the processing and rectifying according to the settings
    void process(const cv::Mat& in, cv::Mat& out) const;


private:

    //cv::Mat     rectify(const cv::Mat& in) const;
    //cv::Mat     rotate(const cv::Mat& in, const cv::Point2d& center, const double rotRad) const;
    //cv::Point2d rotate_point(const cv::Point2d& in, const cv::Point2d& center, const double rotRad) const;

    cv::Mat deinterlaceCut(const cv::Mat& in, const bool even = false) const;
    cv::Mat deinterlaceJakob(const cv::Mat& in) const;
    cv::Mat deinterlace(const cv::Mat& in, const int interpolation) const;
    cv::Mat preprocess(const cv::Mat& in) const;

    // ON/OFF
    int doDeinterlace;
    bool doPreprocess;
    bool doEqualise;

    // Smoothing variables
    int smoother_nr;
    double sigmaX, sigmaY;
    cv::Size kSize;
    int k;

    // For saving images to disk
    boost::format g_format;
    unsigned long int frameNr;
};

#endif // PREPROC_HPP
