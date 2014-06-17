#include <ollieRosTools/Frame.hpp>
#include <ollieRosTools/Landmark.hpp>


/// initialise static members
cv::Ptr<CameraATAN> Frame::cameraModel = cv::Ptr<CameraATAN>();
cv::Ptr<Detector>   Frame::detector    =  cv::Ptr<Detector>();
cv::Ptr<PreProc>    Frame::preproc     =  cv::Ptr<PreProc>();
int Frame::idCounter   = -1;
int Frame::kfIdCounter   = -1;
float Frame::averageQuality = 0.8;
cv::Mat Frame::mask;
cv::Mat Frame::maskRect;

Frame::Frame(const cv::Mat& img, const tf::StampedTransform& imu) : initialised(true) {
    id = ++idCounter;
    kfId = -1;


    ROS_INFO("FRA > CREATING NEW FRAME [ID: %d]", id);
    time = imu.stamp_;

    timePreprocess = 0;
    timeExtract    = 0;
    timeDetect     = 0;

    /// SHOULD BE SET SOMEHOW
    gyroX = gyroY = GyroZ = 0.0;
    accX  = accY  = accZ  = 0.0;

    detectorId = -1; //-1 = none, -2 = KLT
    descriptorId = -1;

    ros::WallTime t0 = ros::WallTime::now();
    cv::Mat imgProc = preproc->process(img);

    image_orig = img.clone();

    image = cameraModel->rectify(imgProc);

    timePreprocess = (ros::WallTime::now()-t0).toSec();

    /// Deal with IMU and Pose
    // reset pose
    pose.setIdentity();    
    // get imu (convert from TF)
    tf::matrixTFToEigen(imu.getBasis(), imuAttitude);


    // just for printing

    OVO::tf2RPY(imu, roll, pitch, yaw);
    roll*=-1.;

    /// TODO: estiamte quality of imageu using acceleromter, gyro and blurriness estiamtion
    estimateImageQuality();

    hasPoseEstimate = false;

    ROS_INFO("FRA < NEW FRAME CREATED [ID: %d]", id);
}




void Frame::addLandMarkRef(int idx, const Landmark::Ptr& lm){
    ROS_ASSERT(static_cast<uint>(idx)<keypointsImg.size() && idx>=0);
    ROS_ASSERT(!lm.empty());
    //ROS_INFO("FRA = Insering LM[%3d] into position [%d] . KP:[%.1f;%.1f] ", lm->getId(), idx, keypointsImg[idx].pt.x,keypointsImg[idx].pt.y);
    if (landmarkRefs.size()==0 || landmarkRefs.find(idx  ) == landmarkRefs.end()){
        landmarkRefs[idx] = lm;
    } else {
        /// Descriptor already matches to a landmark
        ROS_ERROR("FRA = Landmark [%3d] insertion failed for frame [%d|%d], Landmark [%3d] already in position [%3d]!", lm->getId(), getId(), getKfId(), landmarkRefs[idx]->getId(), idx);
        landmarkRefs[idx]->removeObservation(getId(), getKfId());
        landmarkRefs[idx] = lm;
    }
}

void Frame::removeLandMarkRef(const int id){
    ROS_ASSERT_MSG(!landmarkRefs[id].empty(), "FRA = LandmarkRef [%d] Cannot be removed, no landmark there!", id);
    //p.release(); ?
    landmarkRefs.at(id)->removeObservation(getId(), getKfId());
    //landmarkRefs[id] = Landmark::Ptr();
}

// Prepares the keyframe for removal. Remove all references to landmarks and landmarks references to it
void Frame::prepareRemoval(){
    ROS_WARN("Frame [%d|%d] Preparing for removal", getId(), getKfId());
    // remove itself from observations
    const Ints inds = getIndLM();
    for (uint i=0; i<inds.size(); ++i){
        removeLandMarkRef(inds[i]);
    }
    landmarkRefs.clear();
}


void Frame::computeSBI(){
    /// Computes the sbi
    ROS_INFO("SBI > Computing SBI [ID: %d]", id);
    ros::WallTime t0 = ros::WallTime::now();

    double mindimr = std::min(image.cols, image.rows)*0.5;

    cv::Mat sbiroi = cv::Mat(image.clone(), cv::Rect(image.cols/2 - mindimr, image.rows/2 - mindimr, mindimr*2, mindimr*2));
    sbiroi.convertTo(sbi, CV_32FC1);
    cv::resize(sbi, sbi, cv::Size(), 0.5, 0.5, CV_INTER_AREA);
    sbi = OVO::rotateImage(sbi, -getRoll(), CV_INTER_LINEAR, 1.0, 1);
    cv::resize(sbi, sbi, cv::Size(), 0.5, 0.5, CV_INTER_AREA);
    //cv::boxFilter(sbi, sbi, -1, cv::Size(5,5));
    cv::GaussianBlur(sbi, sbi, cv::Size(), 2, 2);
    sbi = sbi(cv::Rect(sbi.cols/2 - sbi.cols/2*0.707, sbi.rows/2 - sbi.rows/2*0.707, sbi.cols/2*1.414, sbi.rows/2*1.414));


    sbi -= cv::mean(sbi);
    double time = (ros::WallTime::now()-t0).toSec();

    ROS_INFO("SBI < Computed in [%.1fms]" ,time*1000.);

}

const cv::Mat& Frame::getSBI(){
    /// Returns a reference to a small blurry image. Computes if if required.
    if (sbi.empty()){
        computeSBI();
    }
    return sbi;
}

float Frame::compareSBI(FramePtr f) {
    ROS_INFO("SBI > Comparing SBI F[%d] vs F[%d]", id, f->getId());
    ros::WallTime t0 = ros::WallTime::now();
    cv::Mat result;
    const cv::Mat s = getSBI();
    int match_method = CV_TM_CCORR_NORMED; // CV_TM_SQDIFF, CV_TM_SQDIFF_NORMED, CV_TM _CCORR, CV_TM_CCORR_NORMED, CV_TM_CCOEFF, CV_TM_CCOEFF_NORMED
    cv::matchTemplate(f->getSBI(), s(cv::Rect(s.cols*0.25, s.rows*0.25, s.cols*0.5, s.rows*0.5)), result, match_method);
    double minVal, maxVal, value;
    cv::Point minLoc, maxLoc, matchLoc;
    cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
    cv::normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

    if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED ){
        matchLoc = minLoc;
        value = minVal;
    } else {
        matchLoc = maxLoc;
        value = maxVal;
    }

    double time = (ros::WallTime::now()-t0).toSec();
    ROS_INFO("SBI < Similarity: %f [%.1fms]" ,value, time*1000.);

    cv::Mat debug_img = f->getSBI().clone();
    cv::rectangle(debug_img, matchLoc, cv::Point(matchLoc.x + s.cols/2 , matchLoc.y + s.rows/2), CV_RGB(255,0,0));
    cv::circle(debug_img, cv::Point(matchLoc.x + s.cols/4 , matchLoc.y + s.rows/4), 3, CV_RGB(255,0,0), 1, CV_AA);
    cv::hconcat(debug_img, s.clone(), debug_img);
    cv::rectangle(debug_img, cv::Point(s.cols*1.25, s.rows*0.25), cv::Point(s.cols*7./4. , s.rows*0.75), CV_RGB(255,0,0));
    cv::normalize(debug_img, debug_img, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
    cv::imshow("debug_img", debug_img);
    cv::Mat debug_img2;
    cv::hconcat(cv::Mat(f->getSBI(), cv::Rect(matchLoc, cv::Point(matchLoc.x + s.cols/2 , matchLoc.y + s.rows/2))) ,
                cv::Mat(s,           cv::Rect(cv::Point(s.cols*0.25, s.rows*0.25), cv::Point(s.cols*0.75 , s.rows*0.75))), debug_img2);
    cv::normalize(debug_img2, debug_img2, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
    cv::imshow("debug_img2", debug_img2);
    cv::waitKey(10);

    return value;
}
