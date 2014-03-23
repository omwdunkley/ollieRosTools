#include <ollieRosTools/aux.hpp>


void OVO::tf2RPY(const tf::Transform& T, float& R, float& P, float& Y){
    const tf::Quaternion q(T.getRotation());
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    R = static_cast<float>(roll);
    P = static_cast<float>(pitch);
    Y = static_cast<float>(yaw);
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

