#ifndef FRAME_HPP
#define FRAME_HPP


#include <iostream>
#include <iomanip>      // std::setprecision


#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

#include <ollieRosTools/aux.hpp>
#include <ollieRosTools/Detector.hpp>
#include <ollieRosTools/CameraATAN.hpp>
#include <ollieRosTools/PreProc.hpp>

class Frame{
private:
        cv::Mat image;
        cv::Mat mask;
        Mats pyramid;
        cv::Size pyramidWindowSize;
        opengv::rotation_t imuAttitude;
        Eigen::Affine3d pose; // transformation in the world frame
        double roll, pitch, yaw;

        /// TODO not used
        double gyroX, gyroY, GyroZ;
        double accX, accY, accZ;

        // Meta
        static long unsigned int idCounter;
        static long unsigned int kfIdCounter;
        ros::Time time;
        int id;
        int kfId; //keyframe id
        bool initialised;
        double timePreprocess, timeDetect, timeExtract;
        float quality; // <0 means not measured, 0 means bad, 1 means perfect
        static float averageQuality;

        // Type of descriptor / detector used
        int descId;
        int detId;


        // Features
        KeyPoints keypoints;
        KeyPoints keypointsRotated; // TODO: should be eigen
        Points2f points; // used by KLT, should be same as keypoints
        cv::Mat descriptors;
        Eigen::MatrixXd bearings;

        //
        static CameraATAN cameraModel;
        static Detector detector;
        static PreProc preproc;


        void drawRPY(cv::Mat &img) const{
            /// Draw an RPY indicator
            const int rad = 50;
            const int offset = 60;
            const float Rd = roll*toDeg;
            const float Pd = pitch*toDeg;
            const float Yd = yaw*toDeg;
            cv::circle(img, cv::Point(img.cols-offset,offset), rad, CV_RGB(30,120,120), 1, CV_AA);
            cv::circle(img, cv::Point(img.cols-offset,offset), 1, CV_RGB(30,150,150), 1, CV_AA);
            cv::circle(img, cv::Point(img.cols-offset+Rd,offset-Pd), 2, CV_RGB(50,255,255), 1, CV_AA);
            cv::circle(img, cv::Point(img.cols-offset+Rd,offset-Pd), 3, CV_RGB(5,25,25), 1, CV_AA);
            cv::circle(img, cv::Point(img.cols-offset + sin(yaw)*rad,offset-cos(yaw)*rad), 2, CV_RGB(50,255,255), 1, CV_AA);
            cv::circle(img, cv::Point(img.cols-offset + sin(yaw)*rad,offset-cos(yaw)*rad), 3, CV_RGB(5,25,25), 1, CV_AA);
            OVO::putInt(img, Rd, cv::Point2d(img.cols-75,img.rows-3*25), CV_RGB(0,200,200), false, "R:");
            OVO::putInt(img, Pd, cv::Point2d(img.cols-75,img.rows-2*25), CV_RGB(0,200,200), false, "P:");
            OVO::putInt(img, Yd, cv::Point2d(img.cols-75,img.rows-1*25), CV_RGB(0,200,200), false, "Y:");
        }

        void computeKeypoints(){
            keypointsRotated.clear();
            points.clear();
            descriptors = cv::Mat();
            ros::WallTime t0 = ros::WallTime::now();
            detector.detect(image, keypoints, detId, mask);
            timeDetect = (ros::WallTime::now()-t0).toSec();
        }

        /// TODO: estiamte image quality. -1 = not estaimted, 0 = bad, 1 = perfect
        void estimateImageQuality(){
            /// USE MASK
            /// estimate noise
            /// estiamte drop outs
            /// estimate blur

            quality =  -1.0;

            // compute average quality
            const float alpha = 0.9;
            if (quality>0){
                averageQuality = averageQuality*alpha + quality*(1.f-alpha);
            }

        }




public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Frame() : initialised(false){
    }

    Frame(const cv::Mat& img, const tf::StampedTransform& imu, const cv::Mat& mask=cv::Mat()) : initialised(true) {
        id = ++idCounter;
        kfId = -1;
        timePreprocess = 0;
        timeExtract    = 0;
        timeDetect     = 0;


        /// SHOULD BE SET SOMEHOW
        gyroX = gyroY = GyroZ = 0.0;
        accX  = accY  = accZ  = 0.0;

        detId = -1; //-1 = none, -2 = KLT
        descId = -1;

        this->mask = mask;

        ros::WallTime t0 = ros::WallTime::now();
        cv::Mat imgProc = preproc.process(img);
        image = cameraModel.rectify(imgProc);
        timePreprocess = (ros::WallTime::now()-t0).toSec();

        pose.setIdentity();

        tf::matrixTFToEigen(imu.getBasis(), imuAttitude);
        time = imu.stamp_;
        OVO::tf2RPY(imu, roll, pitch, yaw); /// TODO: RPY mighrt be negative

        /// TODO: estiamte quality of imageu using acceleromter, gyro and blurriness estiamtion
        estimateImageQuality();
    }


    float getQuality() const {
        return quality;
    }

    cv::Size getSize(){
        return image.size();
    }

    const tf::Pose getTFPose() const {
        //TODO
        tf::Pose pose;
        tf::poseEigenToTF(getPose(), pose);
        return pose;
    }

    const Eigen::Affine3d& getPose() const {
        return pose;
    }

    void setPose(const opengv::transformation_t& t){
        pose = t;
    }

    void setPoseRotationFromImu(){
        // sets the pose from the imu rotation. This is usually used on the very first keyframe
        pose.linear() = imuAttitude;
    }

    bool isInitialised() const{
        return initialised;
    }

    void setAsKF(bool first=false){
        if(first){
            // reset static variables
            kfIdCounter = 0;
            idCounter   = 0;
            // initial pose is zero translation with IMU rotation
            pose.setIdentity();
            setPoseRotationFromImu();
        }
        kfId = ++kfIdCounter;
    }

    const opengv::rotation_t& getImuRotation() const {
        return imuAttitude;
    }

    const sensor_msgs::CameraInfoPtr& getCamInfo() const {
        /// Fetches the cameraInfo used by ros for the current rectification output
        return cameraModel.getCamInfo();
    }

    const cv::Mat& getImage() const{
        return image;
    }

    cv::Mat getVisualImage() const{
        cv::Mat img;

        // Draw keypoints if they exist, also makes img colour
        cv::drawKeypoints(image, keypoints, img, CV_RGB(0,90,20));
        OVO::putInt(img, keypoints.size(), cv::Point(10,1*25), CV_RGB(0,96,0),  true,"");

        // Draw Frame ID
        OVO::putInt(img, id, cv::Point2f(img.cols-95,img.rows-5*25), CV_RGB(0,100,110), true,  "FID:");

        if (kfId>=0){
            OVO::putInt(img, kfId, cv::Point2f(img.cols-95,img.rows-4*25), CV_RGB(0,100,110), true,  "KID:");
        }

        // Draw RPY
        drawRPY(img);

        // Draw timing
        OVO::putInt(img, timePreprocess*1000., cv::Point(10,img.rows-6*25),CV_RGB(200,0,200), false, "P:");
        if (!keypoints.empty()){
            OVO::putInt(img, timeDetect*1000., cv::Point(10,img.rows-5*25), CV_RGB(200,0,200), false, "D:");
        }
        if (descriptors.rows>0){
            OVO::putInt(img, timeExtract*1000., cv::Point(10,img.rows-4*25), CV_RGB(200,0,200), false, "E:");
        }

        // Draw artificial horizon
        return img;
    }


    float getRoll() const{
        /// Gets roll
        return roll;
    }

    KeyPoints& getKeypoints(bool dontCompute=false, bool allowKLT=true){
        /// Gets keypoints. Computes them if required/outdated type unless dontCompute is true. Uses points as kps if possible.
        if (keypoints.empty()){
            // We dont have key points
            if (!points.empty() && allowKLT){
                // use klt points as keypoints
                cv::KeyPoint::convert(points, keypoints);
            } else {
                // we dont have points either or dont want to use them
                if (!dontCompute){
                    // compute  points
                   computeKeypoints();
                }
            }
        } else {
            // we have keypoints
            if (!dontCompute && getDetId() != detector.getDetectorId()){
                // we have outdated keypoints - recompute
                computeKeypoints();
            }
        }
        return keypoints;
    }

    bool hasPoints() const {
        return keypoints.size()>0 || points.size()>0;
    }


    /// TODO: dont shortcut here, rotate points directly
    const Points2f getRotatedPoints(bool aroundOpticalAxis = true){
        // simply returns rotates keypoints as points
        Points2f pts;
        cv::KeyPoint::convert(getRotatedKeypoints(aroundOpticalAxis), pts);
        return pts;
    }

    const Points2f& getPoints(bool dontCompute=true){
        /// Gets points. Use kps as pts if we dont have pts. IF we dont have pts or kps, only compute them if dontCompute = false.
        if (points.empty()){
            // dont have points
            if (!keypoints.empty()){
                // but have keypoints
                cv::KeyPoint::convert(keypoints, points);
            } else {
                // dont have keypoints
                if (!dontCompute){
                    // compute them
                    computeKeypoints();
                    cv::KeyPoint::convert(keypoints, points);
                }
            }
        }
        return points;
    }


    void swapKeypoints(KeyPoints& kps){
        /// Swap keypoints and remove everything that could have come from previous ones
        keypointsRotated.clear();
        points.clear();
        descriptors = cv::Mat();
        descId = -1;
        detId = -2; // unknown type
        std::swap(keypoints, kps);
    }
    void swapPoints(Points2f& pts){
        /// Swap points and remove everything that could have come from previous ones
        keypointsRotated.clear();
        keypoints.clear();
        descriptors = cv::Mat();
        descId = -1;
        detId = -2; // unknown type
        std::swap(points, pts);
    }
    void clearAllPoints(){
            keypointsRotated.clear();
            keypoints.clear();
            points.clear();
            descriptors = cv::Mat();
            descId = -1;
            detId = -2; // unknown type
    }

    const Mats& getPyramid(const cv::Size& winSize, const int maxLevel, const bool withDerivatives=true, int pyrBorder=cv::BORDER_REFLECT_101, int derivBorder=cv::BORDER_CONSTANT){
        /// return image pyramid. Compute if required
        if (pyramid.empty() || winSize != pyramidWindowSize){
            cv::buildOpticalFlowPyramid(image, pyramid, winSize, maxLevel, withDerivatives, pyrBorder, derivBorder, true); //not sure about last flag reuseInput
            pyramidWindowSize = winSize;
        }
        return pyramid;
    }




    const KeyPoints& getRotatedKeypoints(bool aroundOptical=false){
        /// Gets rotated keypoints. Computes them if required.
        if (keypointsRotated.empty()){
            if (!keypoints.empty()){
                keypointsRotated = cameraModel.rotatePoints(keypoints, -getRoll(), aroundOptical);
            } else {
                if (!points.empty()){
                    cv::KeyPoint::convert(points, keypoints);
                    keypointsRotated = cameraModel.rotatePoints(keypoints, -getRoll(), aroundOptical);
                } else {
                    // compute them?
                    ROS_WARN("Asked for rotated keypoints but dont have any points or keypoints to compute them from");
                }
            }
        }
        return keypointsRotated;
    }

    const cv::Mat& getDescriptors(){
        /// Gets descriptors. Computes them if they are empty or the wrong type (determined from the current set extraxtor)
        if (descriptors.empty() || keypoints.empty() || getDescId() != detector.getExtractorId()){
            getKeypoints();
            ros::WallTime t0 = ros::WallTime::now();
            detector.extract(image, keypoints, descriptors, descId, getRoll() );
            timeExtract = (ros::WallTime::now()-t0).toSec();
        }
        return descriptors;
    }

    const Eigen::MatrixXd& getBearings(){
        /// Computes unit bearing vectors projected from the optical center through the (key) points on the image plane
        if (points.size()>0){
            cameraModel.bearingVectors(points, bearings);
        } else if (keypoints.size()>0){
            cv::KeyPoint::convert(keypoints, points);
            cameraModel.bearingVectors(points, bearings);
        }
        return bearings;

    }

    int getId() const {
        return id;
    }



    static void setParameter(ollieRosTools::VoNode_paramsConfig &config, uint32_t level){
        preproc.setParam(config.doPreprocess,
                         config.doDeinterlace,
                         config.doEqualise,
                         config.doEqualiseColor,
                         config.kernelSize,
                         config.sigmaX,
                         config.sigmaY,
                         config. brightness,
                         config.contrast);
        cameraModel.setParams(config.zoomFactor, config.zoom,
                           config.PTAMRectify,
                           config.sameOutInSize,
                           config.width, config.height,
                           config.fx, config.fy,
                           config.cx, config.cy,
                           config.s);

        detector.setParameter(config, level);

    }

    int getDescType() const{
        return detector.getDescriptorType();
    }

    int getDescId() const {
        return descId;
    }
    int getDetId() const {
        return detId;
    }


    friend std::ostream& operator<< (std::ostream& stream, const Frame& frame) {
        stream << "[ID:" << std::setw(5) << std::setfill(' ') << frame.id << "]"
               << "[KF:"  << std::setw(3) << std::setfill(' ') << frame.kfId<< "]"
               << "[KP:"  << std::setw(4) << std::setfill(' ') << frame.keypoints.size() << "]"
               << "[ P:"  << std::setw(4) << std::setfill(' ') << frame.points.size() << "]"
               << "[RP:"  << std::setw(4) << std::setfill(' ') << frame.keypointsRotated.size() << "]"
               << "[ D:"  << std::setw(4) << std::setfill(' ') << frame.descriptors.rows << "]"
               << "[BV:"  << std::setw(4) << std::setfill(' ') << frame.bearings.rows() << "]"
               << "[TP:"  << std::setw(4) << std::setfill(' ') << std::setprecision(1) << frame.timePreprocess << "]"
               << "[TD:"  << std::setw(4) << std::setfill(' ') << std::setprecision(1) << frame.timeDetect << "]"
               << "[TE:"  << std::setw(4) << std::setfill(' ') << std::setprecision(1) << frame.timeExtract << "]";

        return stream;
    }
};

#endif // FRAME_HPP
