#include "ollieRosTools/PreProc.hpp"



PreProc::PreProc(){

    // DEFAULT VALUES
    doEqualise = false;
    doEqualiseColor = true;
    doDeinterlace = false;
    doPreprocess = false;
    smootherNr = -1;
    k=1;
    kSize = cv::Size(k,k);
    sigmaX = 1;
    sigmaY = 1;
    recomputeLUT(0.0, 0.0);

}


cv::Mat PreProc::deinterlaceCut(const cv::Mat& in, const bool even) const {
    /// Return an image with either the even or the odd rows missing
    cv::Mat out = in.reshape(0,in.rows/2);
    if (even){
        out = out.colRange(0,out.cols/2);
    } else {
        out = out.colRange(out.cols/2,out.cols);
    }
    return out;
}

cv::Mat PreProc::deinterlace(const cv::Mat& in, const int interpolation) const {
    cv::Mat half = deinterlaceCut(in);
    // make continuous again
    cv::Mat out(half.size(), half.type());
    half.copyTo(out);

    resize(half, out, cv::Size(), 1, 2, interpolation);
    return out;
}

cv::Mat PreProc::preprocess(const cv::Mat& in) const {
    cv::Mat out;

    switch(smootherNr){
    case -1: /* OFF */
        out = in;
        break; // No smoothing
    case  0:
        cv::medianBlur(in, out, k);
        break;
    case  1:
        cv::GaussianBlur(in, out, kSize, sigmaX, sigmaY);
        break;
    case  2:
        cv::blur(in, out, kSize);
        break;
    case  3:
        cv::bilateralFilter(in, out, cv::max(1,static_cast<int>(floor(k/2.0))), sigmaX, sigmaY);
        break;
    }

    return out;
}



cv::Mat PreProc::process(const cv::Mat& in) const {
    cv::Mat out;
    /// Interlacing
    switch(doDeinterlace){
    case -2:
        out = deinterlaceCut(in);
        break;
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
        out = deinterlace(in, doDeinterlace);
        break;
    default:// Leave as is
        out = in;
        break;
    }


    /// Equalisation
    // BUG: opencv does not support case 1-4, ie the interpolation method specified by doEqualise must be 0.
    //      For now the user case 1-4 defaults to case 0, thanks to the dynamic_reconfigure settings
    switch(doEqualise){
    case -2:
        if(out.channels() >= 3){
            if (doEqualiseColor){
                // Equalise each colour channel
                std::vector<cv::Mat> channels;
                split(out,channels);
                cv::equalizeHist(channels[0], channels[0]);
                cv::equalizeHist(channels[1], channels[1]);
                cv::equalizeHist(channels[2], channels[2]);
                cv::merge(channels,out);
            } else {
                // Convert to different color space and equalise the intensities only
                cv::Mat ycrcb;
                cv::cvtColor(out,ycrcb,CV_BGR2YCrCb);
                std::vector<cv::Mat> channels;
                split(ycrcb,channels);
                cv::equalizeHist(channels[0], channels[0]);
                cv::merge(channels,ycrcb);
                cv::cvtColor(ycrcb,out,CV_YCrCb2BGR);
            }
        }  else {
            cv::equalizeHist(out, out);
        }
        break;
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:

        if(out.channels() >= 3){
            if (doEqualiseColor){
                // Equalise each colour channel
                cv::LUT(out, lut, out, doEqualise);
            } else {
                // Convert to different color space and equalise the intensities only
                cv::Mat ycrcb;
                cv::cvtColor(out,ycrcb,CV_BGR2YCrCb);
                std::vector<cv::Mat> channels;
                split(ycrcb,channels);
                cv::LUT(channels[0], lut, channels[0], doEqualise);
                cv::merge(channels,ycrcb);
                cv::cvtColor(ycrcb,out,CV_YCrCb2BGR);
            }
        }  else {
            cv::LUT(out, lut, out, doEqualise);
        }
        break;
    default:// Leave as is
        break;
    }

    /// Smoothing/Filtering
    if (doPreprocess){
         out = preprocess(out);
    }

    return out;
}

void PreProc::recomputeLUT(const float brightness, const float contrast){
    /// Recomputes the brightness/contrast look up table. Maps intensity values 0-255 to 0-255
    lut = cv::Mat (1, 256, CV_8U);

    if( contrast > 0 ) {
        const float delta = 127.*contrast;
        const float a = 255./(255. - delta*2);
        const float b = a*(brightness*100 - delta);
        for (int i = 0; i < 256; ++i ) {
            int v = round(a*i + b);
            if(v < 0){
                v = 0;
            }
            if(v > 255){
                v = 255;
            }
            lut.at<uchar>(i) = static_cast<uchar>(v);
        }
    } else {
        const float delta = -128.*contrast;
        const float a = (256.-delta*2)/255.;
        const float b = a*brightness*100. + delta;
        for (int i = 0; i < 256; ++i ) {
            int v = round(a*i + b);
            if (v < 0){
                v = 0;
            }
            if (v > 255){
                v = 255;
            }
            lut.at<uchar>(i) = static_cast<uchar>(v);
        }
    }
}

//ollieRosTools::PreProcNode_paramsConfig& PreProc::setParameter(ollieRosTools::PreProcNode_paramsConfig &config, uint32_t level){
void PreProc::setParam(const int doPreprocess,
              const int doDeinterlace,
              const int doEqualise,
              const bool doEqualiseColor,
              const int kernelSize,
              const double sigmaX,
              const double sigmaY,
              const double brightness,
              const double contrast){

    smootherNr = doPreprocess;
    this->doPreprocess = smootherNr >=0;
    this->doDeinterlace = doDeinterlace;
    this->doEqualise = doEqualise;
    this->doEqualiseColor = doEqualiseColor;
    recomputeLUT(brightness, contrast);


    // Smoothing stuff
    k = kernelSize*2+1;
    kSize = cv::Size(k,k);
    this->sigmaX = sigmaX;
    this->sigmaY = sigmaY;

    //return config;
}
