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
        Eigen::Matrix3d imuAttitude;
        float roll, pitch, yaw;

        // Meta
        static long unsigned int idCounter;
        static long unsigned int kfIdCounter;
        ros::Time time;
        int id;
        int kfId; //keyframe id
        bool initialised;
        double timePreprocess, timeDetect, timeExtract;


        // Features
        KeyPoints keypoints;
        KeyPoints keypointsRotated; // TODO: should be eigen
        Points2f points; // used by KLT, should be same as keypoints
        cv::Mat descriptors;
        Eigen::MatrixXf bearings;

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




public:
    Frame() : initialised(false){
    }

    Frame(const cv::Mat& img, const tf::StampedTransform& imu, const cv::Mat& mask=cv::Mat()) : initialised(true) {
        id = ++idCounter;
        kfId = -1;
        timePreprocess = 0;
        timeExtract    = 0;
        timeDetect     = 0;

        this->mask = mask;

        ros::WallTime t0 = ros::WallTime::now();
        cv::Mat imgProc = preproc.process(img);
        image = cameraModel.rectify(imgProc);
        timePreprocess = (ros::WallTime::now()-t0).toSec();


        tf::matrixTFToEigen(imu.getBasis(), imuAttitude);
        time = imu.stamp_;
        OVO::tf2RPY(imu, roll, pitch, yaw); /// TODO: RPY mighrt be negative
    }

    bool isInitialised() const{
        return initialised;
    }

    void setAsKF(){
        kfId = ++kfIdCounter;
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
        cv::drawKeypoints(image, keypoints, img, CV_RGB(0,128,20));
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

    KeyPoints& getKeypoints(bool dontCompute=false){
        /// Gets keypoints. Computes them if required unless dontCompute is true. Uses points as kps if possible.
        if (keypoints.empty() && !dontCompute){
            // If we have points (eg from klt), use as keypoints
            if (points.empty()){
                ros::WallTime t0 = ros::WallTime::now();
                detector.detect(image, keypoints, mask);
                timeDetect = (ros::WallTime::now()-t0).toSec();
            } else {
                cv::KeyPoint::convert(points, keypoints);
            }
        }
        return keypoints;
    }

    const Points2f& getPoints(){
        /// Gets points. Use kps as pts if we dont have pts
        if (points.empty()){
            // dont have points
            if (!keypoints.empty()){
                // but have keypoints
                cv::KeyPoint::convert(keypoints, points);
            }
        }
        return points;
    }


    void swapKeypoints(KeyPoints& kps){
        /// Swap keypoints and remove everything that could have come from previous ones
        keypointsRotated.clear();
        points.clear();
        descriptors = cv::Mat();
        std::swap(keypoints, kps);
    }
    void swapPoints(Points2f& pts){
        /// Swap points and remove everything that could have come from previous ones
        keypointsRotated.clear();
        keypoints.clear();
        descriptors = cv::Mat();
        std::swap(points, pts);
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
            keypointsRotated = cameraModel.rotatePoints(getKeypoints(), -getRoll(), aroundOptical);
        }
        return keypointsRotated;
    }

    const cv::Mat& getDescriptors(){
        /// Gets descriptors. Computes them if required
        if (descriptors.empty()){
            KeyPoints kps = getKeypoints();
            ros::WallTime t0 = ros::WallTime::now();
            detector.extract(image, kps, descriptors, getRoll() );
            timeExtract = (ros::WallTime::now()-t0).toSec();
        }
        return descriptors;
    }

    //const Eigen::MatrixXf& getBearings(){
        /// TODO
        // get bearings
        // if not existant, compute from keypoiints
        // if no keypoints, compute from points
        // else return none
    //}



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
