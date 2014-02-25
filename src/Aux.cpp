#include <ollie_vo/Aux.hpp>

//typedef std::vector<cv::KeyPoint> KeyPoints;
//typedef std::vector<cv::DMatch> DMatches;
//typedef std::vector<DMatches> DMatchesKNN;
//typedef std::deque<cv::Mat> ImagesQ;
//typedef std::deque<cv::Mat> DescriptionsQ;
//typedef std::deque< std::vector<cv::KeyPoint> > KeypointsQ;
//typedef std::deque<tf::StampedTransform> TransformsQ;
//typedef std::deque<std::vector <cv::DMatch> > MatchesQ;

//const double toRad = M_PI/180;
//const double toDeg = 180/M_PI;



void tf2RPY(const tf::Transform& T, double& R, double& P, double& Y){
    tf::Quaternion q(T.getRotation());
    tf::Matrix3x3(q).getRPY(R,P,Y);
}


double tf2Roll(const tf::Transform& T){
    double R,P,Y;
    tf2RPY(T, R, P, Y);
    return R;
}


// Draw a number onto an image
void putInt(cv::Mat& img, const double nr, const cv::Point& p, const CvScalar& col, const bool round, const std::string& str, const std::string& post ){
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


void drawTextCenter(cv::Mat& img, const std::string& text, const CvScalar RGB, const float textScale, const int textThickness){
    cv::Size textSize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, textScale, textThickness, NULL);
    cv::Point textOrg((img.cols - textSize.width)/2,(img.rows + textSize.height)/2);
    cv::putText(img, text, textOrg, cv::FONT_HERSHEY_SIMPLEX, textScale, RGB, textThickness, CV_AA);
}

void printRPYXYZ(const tf::Transform& t, const std::string str){
    double R,P,Y;
    tf2RPY(t, R,P,Y);
    ROS_INFO("%sRPY %3.2f %3.2f %3.2f XYZ %3.3f %3.3f %3.3f",str.c_str(), R*toDeg, P*toDeg, Y*toDeg, t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z());

}


std::vector<int> alignedMatch(const int s){
    std::vector<int> m;
    m.reserve(s);
    for (int i=0; i<s; ++i) m.push_back(i);
    return m;
}


void matAlignMatch (const cv::Mat& vec1in, const cv::Mat& vec2in,
                   cv::Mat& vec1out,      cv::Mat& vec2out,
                    const DMatches& ms){
    vec1out = cv::Mat();
    vec2out = cv::Mat();
    vec1out.reserve(ms.size());
    vec2out.reserve(ms.size());
    for(uint i=0;i<ms.size(); ++i){
        vec1out.push_back(vec1in.row(ms[i].queryIdx));
        vec2out.push_back(vec2in.row(ms[i].trainIdx));
    }
}
void matReduceInd (const cv::Mat& vecIn, cv::Mat& vecOut, const Ints& ind){
    vecOut = cv::Mat();
    vecOut.reserve(ind.size());

    for(uint i=0;i<ind.size(); ++i){
        vecOut.push_back(vecIn.row(ind[i]));
    }
}

// Spits a dmatch list into two int lists
void match2ind(const DMatches& ms, Ints& query, Ints& train){
    query.clear();
    train.clear();

    query.reserve(ms.size());
    train.reserve(ms.size());
    for(uint i=0;i<ms.size(); ++i){
        query.push_back(ms[i].queryIdx);
        train.push_back(ms[i].trainIdx);
    }
}



// Rotate a point in place
void rotatePoint(cv::Point2d& inout, const cv::Point2d& center, const double angle_radians) {
    cv::Point2d shifted = inout - center;
    inout = cv::Point2d(cos(angle_radians)*shifted.x - sin(angle_radians)*shifted.y, sin(angle_radians)*shifted.x + cos(angle_radians)*shifted.y)+ center;
}
void rotatePoint(cv::Point2f& inout, const cv::Point2f& center, const double angle_radians) {
    cv::Point2d shifted = inout - center;
    inout = cv::Point2f(cos(angle_radians)*shifted.x - sin(angle_radians)*shifted.y, sin(angle_radians)*shifted.x + cos(angle_radians)*shifted.y)+ center;
}
// Rotate a kp in place
void rotateKeyPoint(cv::KeyPoint& inout, const cv::Point2d& center, const double angle_radians) {
    rotatePoint(inout.pt, center, angle_radians);
}

void rotateKeyPoints(const KeyPoints kps_in, KeyPoints& kps_out, const cv::Point2d& center, const double angle_radians) {
    kps_out = kps_in;
    for (uint i=0; i<kps_in.size();++i){
        rotateKeyPoint(kps_out[i], center, angle_radians); // We need to rotate BACK, hence negative R
    }
}

cv::Mat rotateImage(const cv::Mat& in, const cv::Point2d &center, const double angleRad, const int interpolation) {
    double angleDeg = angleRad * toDeg;
    cv::Mat out;
    if (std::abs(angleDeg)>0.1){
        cv::Mat rot_mat = cv::getRotationMatrix2D(center, angleDeg, 1.0);
        cv::warpAffine(in, out, rot_mat, in.size(), 0, 0, interpolation);
    } else {
        out = in;
    }
    return out;
}



tf::Transform eigen2tf(const opengv::transformation_t& t){
    tf::Transform trans;
    tf::transformEigenToTF(AffineCompact3d(t), trans); /// HYDRO
    //tf::TransformEigenToTF(AffineCompact3d(t), trans); /// FUERTE
    return trans;
}

void tf2eigen(const tf::Transform& transform, opengv::translation_t& t, opengv::rotation_t& r){
    Affine3d se3;
    tf::transformTFToEigen(transform, se3); //HYDRO+
    //tf::TransformTFToEigen(transform, se3); //FUERTE
    t = se3.translation();
    r = se3.rotation();
}







////reduce vector using only indicies. eg [z a b c d e f],[1 2 5 3] -> [a b e c]
//template <class T> void vecReduceInd (const std::vector<T>& vec, std::vector<T> out, const std::vector<int>& ind) {
//    out.clear();
//    out.reserve(ind.size());
//    for(uint i=0;i<ind.size(); ++i){
//        out.push_back(vec[ind[i]]);
//    }
//    return out;
//}

//// Takes two vectors and aligns then
//template <class T> void vecAlignMatch (const std::vector<T>& vec1in, const std::vector<T>& vec2in,
//                                              std::vector<T>& vec1out,      std::vector<T>& vec2out,
//                                        const DMatches& ms) {
//    vec1out.clear();
//    vec2out.clear();
//    vec1out.reserve(ms.size());
//    vec2out.reserve(ms.size());
//    for(uint i=0;i<ms.size(); ++i){
//        vec1out.push_back(vec1in[ms[i].queryIdx]);
//        vec2out.push_back(vec2in[ms[i].trainIdx]);
//    }
//}










