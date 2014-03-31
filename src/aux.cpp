#include <ollieRosTools/aux.hpp>

// Extern var changed once when starting the node depending on the parameters
bool USEIMU = false;

CvScalar OVO::getColor(const float range_min, const float range_max, float depth, bool reverse){

    if (depth < range_min){
        depth = range_min;
    } else  if (depth > range_max){
        depth = range_max;
    }
    float diff_intensity = range_max - range_min;
    if( diff_intensity == 0 ){
        diff_intensity = 1e20;
    }
    float value = 1.f - (depth - range_min)/diff_intensity;
    value = std::min(value, 1.0f);
    value = std::max(value, 0.0f);
    const float h = value * 4.0f + 1.0f;
    int i = floor(h);

    float f = h - i;
    if ( (i&1) ) f = 1 - f; // if i is even
    const float n = 1 - f;
    cv::Vec3f color;
    //    if      (i <= 1) color[0] = n, color[1] = 0, color[2] = 1; //R
    //    else if (i == 2) color[0] = 0, color[1] = n, color[2] = 1; //Y
    //    else if (i == 3) color[0] = 0, color[1] = 1, color[2] = n; //G
    //    else if (i == 4) color[0] = n, color[1] = 1, color[2] = 0; //B
    //    else if (i == 5) color[0] = 1, color[1] = n, color[2] = 0; //P
    //    else if (i >= 6) color[0] = 1, color[1] = 0, color[2] = n; //M
    if (i == 2) color[0] = n, color[1] = 0, color[2] = 1; //R
    if (i == 3) color[0] = 0, color[1] = n, color[2] = 1; //Y
    if (i >= 4) color[0] = 0, color[1] = 1, color[2] = n; //G
    //if (i >= 5) color[0] = n, color[1] = 1, color[2] = 0; //B
    //if (i <= 1) color[0] = 1, color[1] = n, color[2] = 0; //P
    if (i <= 1) color[0] = 1, color[1] = 0, color[2] = n; //M

    color *= 255;
    if (reverse){
        return CV_RGB(255-color[0],255-color[1],255-color[2]);
    } else {
        return CV_RGB(color[0],color[1],color[2]);
    }

}

void OVO::tf2RPY(const tf::Transform& T, double& R, double& P, double& Y){
    const tf::Quaternion q(T.getRotation());
    tf::Matrix3x3(q).getRPY(R, P, Y);
}

void OVO::drawTextCenter(cv::Mat& img, const std::string& text, const CvScalar RGB, const float textScale, const int textThickness){
    cv::Size textSize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, textScale, textThickness, NULL);
    cv::Point textOrg((img.cols - textSize.width)/2,(img.rows + textSize.height)/2);
    cv::putText(img, text, textOrg, cv::FONT_HERSHEY_SIMPLEX, textScale, RGB, textThickness, CV_AA);
}

// Draw a number onto an image
void OVO::putInt(cv::Mat& img, const float nr, const cv::Point& p, const CvScalar& col, const bool round, const std::string& str, const std::string& post ){
    std::ostringstream ss;
    ss << (nr<0?"-":" ") << std::setfill('0') << std::setiosflags(std::ios::fixed) << std::setprecision(1) << std::setw(3);
    //ss << (nr<0?"-":" ") << std::setprecision(1);

    if (round){
        ss << static_cast<int>(std::abs(nr +0.5* (nr<0?-1:1)));
    } else {
        ss << std::abs(nr);
    }

    cv::putText(img, str + ss.str() +post, p ,cv::FONT_HERSHEY_SIMPLEX, 0.5, col,1,CV_AA,false);
}

// matOut[i] = matIn[ind[i]]
void OVO::matReduceInd (const cv::Mat& matIn, cv::Mat& matOut, const Ints& ind){
    matOut = cv::Mat();
    matOut.reserve(ind.size());
    for(uint i=0;i<ind.size(); ++i){
        matOut.push_back(matIn.row(ind[i]));
    }
}

void OVO::alignedBV (const Eigen::MatrixXd& bvm1, const Eigen::MatrixXd& bvm2, const DMatches& ms, opengv::bearingVectors_t& bv1, opengv::bearingVectors_t& bv2){
    bv1.clear();
    bv2.clear();
    bv1.reserve(ms.size());
    bv2.reserve(ms.size());
    for(uint i=0;i<ms.size(); ++i){
        bv1.push_back(bvm1.row(ms[i].queryIdx));
        bv2.push_back(bvm2.row(ms[i].trainIdx));
    }
}

void OVO::transformPoints(const Eigen::Affine3d& transform, opengv::points_t& points){
    for (uint i =0; i<points.size(); ++i){
        points[i] = transform * points[i];
    }
}


