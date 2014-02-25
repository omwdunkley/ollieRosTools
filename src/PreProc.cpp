#include "ollie_vo/PreProc.hpp"



PreProc::PreProc(){

    // DEFAULT VALUES
    doEqualise = false;
    doDeinterlace = false;
    doPreprocess = false;
    smoother_nr = -1;
    k=1;
    kSize = cv::Size(k,k);
    sigmaX = 1;
    sigmaY = 1;
    frameNr = 0;

}


cv::Mat PreProc::deinterlaceCut(const cv::Mat& in, const bool even) const {
    //O(1)
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


cv::Mat PreProc::deinterlaceJakob(const cv::Mat& in) const {
    cv::Mat uneven = in;

    /// JAKOB CODE
    const int step = uneven.step;
    const int w = in.cols;
    const int h = in.rows;
    // border condition
    memcpy(uneven.data, uneven.data+step, step);	// in uneven: 0'th row is clone of 1'th row.

    for(int i=0;i<h/2-1;i++){
        for(int j=0;j<w;j++){
            // even: (2i+1)'th row = mean(2i, 2i+2)
            uneven.data[j + step*(2*i+2)] = ((int)uneven.data[j + step*(2*i+1)] + (int)uneven.data[j + step*(2*i+3)]) >> 1;
        }
    }
    return uneven;
}

cv::Mat PreProc::preprocess(const cv::Mat& in) const {
    cv::Mat out;

    switch(smoother_nr){
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



void PreProc::process(const cv::Mat& in, cv::Mat& out) const {
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
    case 5: // Replace odd rows with even ones
        out = deinterlaceJakob(in);
        break;
    default:// Leave as is
        out = in;
        break;
    }


    /// Equalisation
    if (doEqualise){
        cv::equalizeHist( out, out );
    }

    /// Smoothing/Filtering
    if (doPreprocess){
         out = preprocess(out);
    }


}

ollie_vo::VoNode_paramsConfig& PreProc::setParameter(ollie_vo::VoNode_paramsConfig &config, uint32_t level){


    smoother_nr = config.doPreprocess;
    doPreprocess = smoother_nr >=0;
    doDeinterlace = config.doDeinterlace;
    doEqualise = config.doEqualise;

    // Smoothing stuff
    k = config.PreProc_kernel*2+1;
    kSize = cv::Size(k,k);
    sigmaX = config.PreProc_SigX;
    sigmaY = config.PreProc_SigY;

    return config;
}
