#ifndef PREPROC_HPP
#define PREPROC_HPP

#include <ollieRosTools/PreProcNode_paramsConfig.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>





class PreProc {
public:
    // Construct and use path to load camera model
    PreProc();

    // Set all the options
    ollieRosTools::PreProcNode_paramsConfig& setParameter(ollieRosTools::PreProcNode_paramsConfig &config, uint32_t level);

    // does all the processing and rectifying according to the settings
    cv::Mat process(const cv::Mat& in) const;


private:

    cv::Mat deinterlaceCut(const cv::Mat& in, const bool even = false) const;
    cv::Mat deinterlace(const cv::Mat& in, const int interpolation) const;
    cv::Mat preprocess(const cv::Mat& in) const;
    void recomputeLUT(const float brightness, const float contrast);

    // ON/OFF
    int doDeinterlace;
    bool doPreprocess;
    int doEqualise;
    bool doEqualiseColor;
    bool nodeOn;

    // Smoothing variables
    int smootherNr;
    double sigmaX, sigmaY;
    cv::Size kSize;
    int k;

    // Members
    cv::Mat lut;
};

#endif // PREPROC_HPP
