#include <ollieRosTools/aux.hpp>

// Extern var changed once when starting the node depending on the parameters
// ugly but well..few days left to code..

bool USE_IMU = false;
bool USE_SYNTHETIC = false;
std::string IMU_FRAME = "/cf_attitude";
std::string WORLD_FRAME = "/world";
std::string CAM_FRAME = "/cam";
Eigen::Affine3d IMU2CAM = Eigen::Affine3d::Identity();


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
    if (i <= 1) color[0] = 1, color[1] = 0, color[2] = n; //M
    if (i == 2) color[0] = n, color[1] = 0, color[2] = 1; //R
    if (i == 3) color[0] = 0, color[1] = n, color[2] = 1; //Y
    if (i >= 4) color[0] = 0, color[1] = 1, color[2] = n; //G
    //if (i >= 5) color[0] = n, color[1] = 1, color[2] = 0; //B
    //if (i <= 1) color[0] = 1, color[1] = n, color[2] = 0; //P


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
void OVO::tf2RPY(const Eigen::Matrix3d& T, double& R, double& P, double& Y){

    Eigen::Matrix<double,3,1> euler = T.eulerAngles(2, 1, 0);
    Y = euler(0,0);
    P = euler(1,0);
    R = euler(2,0);
}

void OVO::testColorMap(){
        // test color map
        cv::Mat test = cv::Mat(100,1600,CV_8UC3);
        for(int i=0; i<test.cols; ++i){
            test.col(i) = OVO::getColor(100,test.cols-100,i);
        }
        cv::imshow("colmap",test);
        cv::waitKey(1000);

}

std::string OVO::colorise(const std::string& str, const FG& fg, const BG& bg){
    std::stringstream ss;
    ss<<"\033["<<fg<<"m"<<str<<"\033["<<bg<<"m";
    return ss.str();
}


void OVO::drawTextCenter(cv::Mat img, const std::string& text, const CvScalar RGB, const float textScale, const int textThickness){
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
void OVO::matReduceInd (cv::Mat& matInOut, const Ints& ind){
    cv::Mat matTemp;
    matTemp = cv::Mat();
    matTemp.reserve(ind.size());
    for(uint i=0;i<ind.size(); ++i){
        matTemp.push_back(matInOut.row(ind[i]));
    }
    std::swap(matTemp, matInOut);
}

void OVO::match2ind(const DMatches& ms, Ints& query, Ints& train){
    query.clear();
    train.clear();

    query.reserve(ms.size());
    train.reserve(ms.size());
    for(uint i=0;i<ms.size(); ++i){
        query.push_back(ms[i].queryIdx);
        train.push_back(ms[i].trainIdx);
    }
}


void OVO::alignedBV (const Eigen::MatrixXd& bvm1, const Eigen::MatrixXd& bvm2, const DMatches& ms, Bearings& bv1, Bearings& bv2){
    bv1.clear();
    bv2.clear();
    bv1.reserve(ms.size());
    bv2.reserve(ms.size());
    for(uint i=0;i<ms.size(); ++i){
        bv1.push_back(bvm1.row(ms[i].queryIdx));
        bv2.push_back(bvm2.row(ms[i].trainIdx));
    }
}

void OVO::matReduceInd (const Eigen::MatrixXd& bvm1, Bearings& bv1, const Ints& ind){
    bv1.clear();
    bv1.reserve(ind.size());
    for(uint i=0; i<ind.size(); ++i){
        bv1.push_back(bvm1.row(ind[i]));
    }
}

Bearings OVO::eigenBearing2Vector(const Eigen::MatrixXd& bvm){
    ROS_WARN("REIMPLEMENT EigenBearing2Vector to share memory");
    Bearings bvs;
    bvs.reserve(bvm.rows());
    for (int i=0; i<bvm.rows(); ++i){
        bvs.push_back(bvm.row(i));
    }
    return bvs;
}



void OVO::transformPoints(const Eigen::Affine3d& transform, opengv::points_t& points){
    for (uint i =0; i<points.size(); ++i){
        points[i] = transform * points[i];
    }
}

cv::Mat OVO::rotateImage(const cv::Mat& in, const double angleRad, const int interpolation, const double scale, const double thresh) {
    double angleDeg = angleRad * toDeg;
    cv::Mat out;
    if (scale!= 1.0 || std::abs(angleDeg)>thresh){
        cv::Mat rot_mat = cv::getRotationMatrix2D(cv::Point(in.cols/2., in.rows/2.), angleDeg, scale);
        cv::warpAffine(in, out, rot_mat, in.size(), 0, 0, interpolation);
    } else {
        out = in;
    }
    return out;
}

void OVO::drawFlow(cv::Mat img, const Points2f& q, const Points2f& t, const DMatches& ms, const CvScalar& col, const double oppacity){
    Points2f kfPts, fPts;
    OVO::vecAlignMatch<Points2f>(q,t, fPts, kfPts, ms);
    drawFlowAligned(img, fPts, kfPts, col, oppacity);
}
void OVO::drawFlowAligned(cv::Mat img, const Points2f& fPts, const Points2f& kfPts, const CvScalar& col, const double oppacity){
    ROS_ASSERT(fPts.size()==kfPts.size());
    cv::Mat alpha;
    if (oppacity>=1.f){
        alpha = img;
    } else {
        alpha = cv::Mat::zeros(img.size(), img.type());
    }

    for (uint i=0; i<fPts.size(); ++i){
        cv::line(alpha, fPts[i], kfPts[i], col, 1, CV_AA);
    }

    if (oppacity>=1.f){
    } else {
        cv::addWeighted(img, 1.0, alpha, oppacity, 0.0, img);
    }
}



Eigen::VectorXd OVO::reprojectErrPointsVsBV(
        const Pose& model,
        const Points3d& points,
        const Bearings& bv,
        const BEARING_ERROR method){
    /// model -> bearing vectors are in this frame which is in the world frame (same as points)
    /// points -> world points

    Eigen::Affine3d inverseSolution;
    inverseSolution = model.inverse();
    Eigen::VectorXd scores(points.size());
    for(uint i = 0; i < points.size(); ++i){
        scores[i] = errorBV(inverseSolution * points[i], bv[i], method);
    }
    return scores;
}

Eigen::VectorXd OVO::reprojectErrPointsVsBV(
        const Points3d& points,
        const Bearings& bv,
        const BEARING_ERROR method){
    /// points -> points in the frame of bv

    Eigen::VectorXd scores(points.size());
    for(uint i = 0; i < points.size(); ++i){
        scores[i] = errorBV(points[i], bv[i], method);
    }
    return scores;
}



double OVO::angle2error(const double angleDeg, const BEARING_ERROR method ){
   Bearing bv1 = Eigen::Vector3d(1,0,0);
   Bearing bv2 = Eigen::Vector3d(0,0,0);
   bv2.head(2) = Eigen::Rotation2Dd(angleDeg*toRad)*Eigen::Vector2d(1,0);
   return errorBV(bv1, bv2, method);
}

// Compute the error between bearing vectors
double OVO::errorBV(const Bearing& bv1, const Bearing& bv2, const BEARING_ERROR method ){
    const Bearing bva = bv1.normalized();
    const Bearing bvb = bv2.normalized();
    switch(method){
        case BVERR_OneMinusAdotB:  return 1.0 - bva.dot(bvb);
        case BVERR_ATAN2:          return atan2((bva.cross(bvb).norm()), bva.dot(bvb));
        case BVERR_NormAminusB:    return (bva-bvb).norm();
        case BVERR_SUM_AminusBSqr: return ((bva-bvb).array().pow(2)).sum();
        default: ROS_ASSERT_MSG(0, "Unknown bearing method type [%d]", method); return 0;
    }
}
double OVO::errorNormalisedBV(const Bearing& bva, const Bearing& bvb, const BEARING_ERROR method ){
    switch(method){
        case BVERR_OneMinusAdotB:  return 1.0 - bva.dot(bvb);
        case BVERR_ATAN2:          return atan2((bva.cross(bvb).norm()), bva.dot(bvb));
        case BVERR_NormAminusB:    return (bva-bvb).norm();
        case BVERR_SUM_AminusBSqr: return ((bva-bvb).array().pow(2)).sum();
        default: ROS_ASSERT_MSG(0, "Unknown bearing method type [%d]", method); return 0;
    }
}

// returns the error given px dist on image plane with focal length f
double OVO::px2error(const double px, const double horiFovDeg, const double width, const BEARING_ERROR method){ // 252px = 110° horizontal FOV with 720px width
    return angle2error(OVO::px2degrees(px, horiFovDeg, width), method);
}

double OVO::px2degrees(const double px, const double horiFovDeg, const double width){
    const double focal_px = (horiFovDeg*toRad/2.) / atan(width/2.);
    return atan(px/focal_px)*toDeg;
}


// Returns a rotation R such that BV_from = R*BV_to
void OVO::relativeRotation(const Eigen::Matrix3d& ImuRotFrom, const Eigen::Matrix3d& ImuRotTo, Eigen::Matrix3d& rotRelative){
    ROS_ASSERT_MSG(USE_IMU, "This function should probably not be called if we are not using an IMU");
    rotRelative = IMU2CAM.linear().transpose() * ImuRotFrom.transpose() * ImuRotTo * IMU2CAM.linear();

    //adapter.setR12(IMU2CAM.linear().transpose() * kf->getImuRotationCam().transpose() * f->getImuRotationCam() * IMU2CAM.linear());
    //adapter.setR12(kf->getImuRotation().transpose() * f->getImuRotation());
    //adapter.setR12(f->getImuRotation() * kf->getImuRotation().transpose());
    //adapter.setR12(f->getImuRotation().transpose() * kf->getImuRotation());
//            adapter.setR12(Eigen::Matrix3d::Identity());
//            ROS_INFO_STREAM("kf' * f\n" << kf->getImuRotation().transpose() * f->getImuRotation());
//            ROS_INFO_STREAM("f * kf'\n" << f->getImuRotation() * kf->getImuRotation().transpose());
//            ROS_INFO_STREAM("f' * kf\n" << f->getImuRotation().transpose() * kf->getImuRotation());
}






/*
visualization_msgs::Marker OVO::getPointsMarker(const opengv::points_t& worldPoints){
    visualization_msgs::Marker ms;
    ms.header.stamp = ros::Time::now();
    ms.ns = "worldPoints";
    ms.id = 0;
    ms.header.frame_id = "/cf_xyz";
    ms.type = visualization_msgs::Marker::POINTS;
    ms.action = visualization_msgs::Marker::ADD;
    ms.scale.x = 0.05;
    ms.scale.y = 0.05;

    ms.color.a = 1.0;
    ms.color.r = 1.0;
    ms.color.g = 1.0;
    ms.color.b = 1.0;

    //tf::poseTFToMsg(f2->getTFPose(), ms.pose);
    uint i;
    for (i=0; i< worldPoints.size(); ++i){
        geometry_msgs::Point p;
        tf::pointEigenToMsg(worldPoints[i],p);
        ms.points.push_back(p);
    }
    return ms;

}
*/

//Doubles OVO::reprojectErrBvVsBv(
//        const Eigen::Affine3d& model,
//        const opengv::points_t& points,
//        const Bearings& bv){
////    const model_t & model,
////    const std::vector<int> & indices,
////    std::vector<double> & scores) {

//    Eigen::Affine3d inverseSolution;
//    //inverseSolution.block<3,3>(0,0) = model.block<3,3>(0,0).transpose();
//    //inverseSolution.col(3) = -inverseSolution.block<3,3>(0,0)*model.col(3);
//    inverseSolution = model.inverse();

//  Eigen::Matrix<double,4,1> p_hom;
//  p_hom[3] = 1.0;

//  for( size_t i = 0; i < points.size(); ++i ) {
//    p_hom.block<3,1>(0,0) = opengv::triangulation::triangulate2(_adapter,indices[i]);
//    bearingVector_t reprojection1 = p_hom.block<3,1>(0,0);
//    bearingVector_t reprojection2 = inverseSolution * p_hom;
//    reprojection1 = reprojection1 / reprojection1.norm();
//    reprojection2 = reprojection2 / reprojection2.norm();
//    bearingVector_t f1 = _adapter.getBearingVector1(indices[i]);
//    bearingVector_t f2 = _adapter.getBearingVector2(indices[i]);

//    //bearing-vector based outlier criterium (select threshold accordingly):
//    //1-(f1'*f2) = 1-cos(alpha) \in [0:2]
//    double reprojError1 = 1.0 - (f1.transpose() * reprojection1);
//    double reprojError2 = 1.0 - (f2.transpose() * reprojection2);
//    scores.push_back(reprojError1 + reprojError2);
//  }
//}

