#include <ollieRosTools/Map.hpp>

int OdoPoint::pIdCounter = 0;


cv::Mat getPointsProjectedImage(FramePtr& f, const opengv::points_t& worldPts, const Ints& idx){

    //cv::Mat img = f->getVisualImage();
    cv::Mat img = cv::Mat::zeros(f->getImage().size(), CV_8UC3);

    ROS_ASSERT_MSG(idx.size()==worldPts.size(), "WorldPts and Idx must be the same size. WP[i] <-> Feat[idx[i]]");
    //cv::Mat imgOverlay = cv::Mat::zeros(img.size(), CV_8UC3);
    const Points2f& fpts  = f->getPoints();

    /// TODO: would be better to back project instead of assuming the 2d point is correct
    // put the world points in frame of f
    opengv::points_t pts3d = worldPts;
    OVO::transformPoints(f->getPose().inverse(), pts3d); /// TODO: should be done twice, one for F and once for KF
    Eigen::VectorXd dists(idx.size());
    for (uint i=0; i<idx.size(); ++i){
        dists[i] = pts3d[i].norm();
    }

    float range_max = dists.maxCoeff();
    float range_min = dists.minCoeff();
    float range_mean = dists.mean();

    // draw colour coded depth points
    for (uint i=0; i<idx.size(); ++i){
        cv::circle(img, fpts[idx[i]], 3, OVO::getColor(range_min, range_max, dists[i]), CV_FILLED, CV_AA);
    }

    OVO::putInt(img, idx.size(), cv::Point(10,3*25), CV_RGB(0,96*2,0),  true, "VO:");
    OVO::putInt(img, range_min,  cv::Point(10,5*25), OVO::getColor(range_min, range_max, range_min),  false , "MinR:");
    OVO::putInt(img, range_mean, cv::Point(10,6*25), OVO::getColor(range_min, range_max, range_mean), false , "AvgR:");
    OVO::putInt(img, range_max,  cv::Point(10,7*25), OVO::getColor(range_min, range_max, range_max),  false , "MaxR:");

    //cv::addWeighted(img, 1.0, imgOverlay, 0.75, 0, img);

    return img;
}

bool noRef(const PointPtr& p){
    return !(*(p.refcount)>1);
}
