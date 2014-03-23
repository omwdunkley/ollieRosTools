#ifndef FRAME_HPP
#define FRAME_HPP

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <Eigen/Eigen>

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
        Eigen::Matrix3d imuAttitude;
        float roll, pitch, yaw;

        // Meta
        static long unsigned int idCounter;
        ros::Time time;
        int id;
        bool initialised;
        double timePreprocess, timeDetect, timeExtract;


        // Features
        KeyPoints keypoints;
        KeyPoints keypointsRotated;
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
        cv::drawKeypoints(image, keypoints, img, CV_RGB(0,255,0));
        OVO::putInt(img, keypoints.size(), cv::Point(10,1*25), CV_RGB(0,96,0),  true,"");

        // Draw Frame ID
        OVO::putInt(img, id, cv::Point2f(img.cols-95,img.rows-5*25), CV_RGB(0,100,110), true,  "FID:");

        // Draw RPY
        drawRPY(img);

        // Draw timing


        // DRaw timing
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

    KeyPoints& getKeypoints(){
        /// Gets keypoints. Computes them if required.
        if (keypoints.empty()){
            ros::WallTime t0 = ros::WallTime::now();
            detector.detect(image, keypoints, mask);
            timeDetect = (ros::WallTime::now()-t0).toSec();
        }
        return keypoints;
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
};

#endif // FRAME_HPP
