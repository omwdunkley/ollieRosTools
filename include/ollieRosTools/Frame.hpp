#ifndef FRAME_HPP
#define FRAME_HPP


#include <iostream>
#include <iomanip>      // std::setprecision
#include <vector>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

#include <ollieRosTools/Detector.hpp>
#include <ollieRosTools/CameraATAN.hpp>
#include <ollieRosTools/PreProc.hpp>
//#include <ollieRosTools/Map.hpp>
#include <ollieRosTools/aux.hpp>




class Frame{
    private:
        cv::Mat image;
        cv::Mat mask;
        cv::Mat sbi;
        Mats pyramid;
        cv::Size pyramidWindowSize;
        opengv::rotation_t imuAttitudeCam; // IMU rotation in the camera frame
        Eigen::Affine3d pose; // transformation of the camera in the world frame
        double roll, pitch, yaw;  //quad frame
        /// TODO not used yet
        double gyroX, gyroY, GyroZ; //quad frame
        double accX, accY, accZ; // quad frame

        // Meta
        static int idCounter;
        static int kfIdCounter;
        ros::Time time;
        int id;
        int kfId; //keyframe id
        bool initialised;
        double timePreprocess, timeDetect, timeExtract;
        float quality; // <0 means not measured, 0 means bad, 1 means perfect
        static float averageQuality;
        Eigen::Matrix3d imu2cam;
        bool hasPoseEstimate;

        // Type of descriptor / detector used
        int descriptorId;
        int detectorId;


        // Features - ALL OF THESE SHOULD ALWAYS BE ALIGNED!
        KeyPoints keypointsImg;
        KeyPoints keypointsRotated; // TODO: should be eigen
        cv::Mat descriptors;
        cv::Mat descriptorsCachedVo;
        Eigen::MatrixXd bearings;
        Eigen::MatrixXd pointsRect; // rectified points, align with all of the above

        //
        static CameraATAN cameraModel;
        static Detector detector;
        static PreProc preproc;

        /// If a keyframe, this contains references to map points
        std::map<uint, LandmarkPtr> landmarkRefs;

        /// cached?
//        KeyPoints             mapKeyPoints;
//        cv::Mat               mapDescriptors;
//        Eigen::MatrixXd       mapbearings;


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
            ROS_INFO("FRA > Computing Keypoints frame [id: %d]", getId());
            clearAllPoints();
            ros::WallTime t0 = ros::WallTime::now();
            detector.detect(image, keypointsImg, detectorId, mask);
            timeDetect = (ros::WallTime::now()-t0).toSec();
            ROS_INFO("FRA < Computed [%lu] Keypoints of [Type: %d] for frame [id: %d] in  [%.1fms] ", keypointsImg.size(), getDetId(), getId(),timeDetect*1000.);
        }

        /// TODO: estiamte image quality. -1 = not estaimted, 0 = bad, 1 = perfect
        void estimateImageQuality(){
            ROS_INFO("FRA > Computing Image Quality Frame [%d]", getId());
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
            ROS_WARN("FRA = NOT IMPLEMENTED IMAGE QUALITY");
            ROS_INFO("FRA < Computed Image Quality %f/%f", quality, averageQuality);

        }




    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Frame() : initialised(false){
            ROS_INFO("FRA = NEW UNINITIALISED FRAME");
        }

        Frame(const cv::Mat& img, const tf::StampedTransform& imu, const cv::Mat& mask=cv::Mat()) : initialised(true) {
            id = ++idCounter;
            kfId = -1;


            ROS_INFO("FRA > CREATING NEW FRAME [ID: %d]", id);
            time = imu.stamp_;

            imu2cam << 0, 0, 1, -1, 0 ,0, 0,-1, 0;

            timePreprocess = 0;
            timeExtract    = 0;
            timeDetect     = 0;

            /// SHOULD BE SET SOMEHOW
            gyroX = gyroY = GyroZ = 0.0;
            accX  = accY  = accZ  = 0.0;

            detectorId = -1; //-1 = none, -2 = KLT
            descriptorId = -1;

            this->mask = mask;

            ros::WallTime t0 = ros::WallTime::now();
            cv::Mat imgProc = preproc.process(img);
            image = cameraModel.rectify(imgProc);
            timePreprocess = (ros::WallTime::now()-t0).toSec();

            /// Deal with IMU and Pose
            // reset pose
            pose.setIdentity();
            // get imu (convert from TF)
            Eigen::Matrix3d imuAttitude;
            tf::matrixTFToEigen(imu.getBasis(), imuAttitude);

            // get imu in cam frame
            imuAttitudeCam = imu2cam*imuAttitude;

            // just for printing

            OVO::tf2RPY(imu, roll, pitch, yaw);
            roll*=-1.;

            /// TODO: estiamte quality of imageu using acceleromter, gyro and blurriness estiamtion
            estimateImageQuality();

            hasPoseEstimate = false;

            ROS_INFO("FRA < NEW FRAME CREATED [ID: %d]", id);


        }

        void addLandMarkRef(const uint id, LandmarkPtr lm){
            ROS_ASSERT(id<keypointsImg.size());
            std::pair<std::map<int,LandmarkPtr>::iterator,bool> ret;
            ret = landmarkRefs.insert(std::make_pair(id, lm));
            ROS_ASSERT_MSG(ret.second, "FRA = Landmark [%d] insertion failed, Landmark [%d] already under positiong [%d]!",lm->getId(), ret.first->second->getId(), id);
        }

        void removeLandMarkRef(const int id){
            size_t removed = landmarkRefs.erase(id);
            ROS_ASSERT(removed>0);
        }

        bool PoseEstimated() const{
            return hasPoseEstimate;
        }

        void swapKeypoints(KeyPoints& kps, bool updateOnly = false){
            /// Swap keypoints and remove everything that could have come from previous ones
            // IF updateOnly is set, we just "update" the keypoints.
            if (updateOnly){
                // update points only
                ROS_ASSERT(kps.size()==keypointsImg.size());
                std::swap(keypointsImg, kps);
                ROS_INFO("FRA = Updated [%lu] Keypoints for frame [%d]", keypointsImg.size(), getId());
            } else {
                // replace points
                clearAllPoints();
                std::swap(keypointsImg, kps);
                ROS_INFO("FRA = Replaced [%lu] old Keypoints with new [%lu] ones for frame [%d]", kps.size(), keypointsImg.size(), getId());
            }
        }

        const cv::Mat& getSBI(){
            /// Returns a reference to a small blurry image. Computes if if required.
            if (sbi.empty()){
                computeSBI();
            }
            return sbi;
        }

        void computeSBI(){
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

        float compareSBI(FramePtr& f) {
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



/*        visualization_msgs::Marker getWorldPointsMarker(int id=0, float scale=1.0, float brightness = 1.0){
            visualization_msgs::Marker ms;
            ms.header.stamp = ros::Time::now();
            ms.ns = "worldPoints";
            ms.id = id;
            ms.header.frame_id = "/world"; /// TODO: use variable name
            ms.type = visualization_msgs::Marker::POINTS;
            ms.action = visualization_msgs::Marker::ADD;
            ms.scale.x = 0.05*scale;
            ms.scale.y = 0.05*scale;

            if (worldPoints3d.size()>0){
                /// HEre we could use the distance from the camera, or the distance in z direction
                // put points in local frame
                opengv::points_t pts3d = worldPoints3d;
                OVO::transformPoints(getPose().inverse(), pts3d);
                Eigen::VectorXd dists(pts3d.size());

                /// distance from camera
                for (uint i=0; i<pts3d.size(); ++i){
                    dists[i] = pts3d[i].norm();
                }

//                /// depth from camera
//                for (uint i=0; i<pts3d.size(); ++i){
//                    dists[i] = pts3d[i][2];
//                }

                /// Set min, max depth and scale colours accordingly
                float range_max = 6; //dists.maxCoeff();
                float range_min = 1; //dists.minCoeff();

//                /// Compute min, max depth and scale colours accordingly
//                float range_max = dists.maxCoeff();
//                float range_min = dists.minCoeff();


                for (uint i=0; i< worldPoints3d.size(); ++i){
                    geometry_msgs::Point p;
                    cv::Scalar rgb = OVO::getColor(range_min, range_max, dists[i]);
                    rgb *= brightness/255.;
                    std_msgs::ColorRGBA col;
                    col.a = 0.7;
                    col.r = rgb[0];
                    col.g = rgb[1];
                    col.b = rgb[2];
                    ms.colors.push_back(col);
                    tf::pointEigenToMsg(worldPoints3d[i], p);
                    ms.points.push_back(p);
                }
            }
            return ms;

        }*/

        // Sets frame as keyframe, prepares it for bundle adjustment
        void setAsKF(bool first=false){
            ros::WallTime t0 = ros::WallTime::now();            
            kfId = ++kfIdCounter;
            if(first){
                ROS_INFO("FRA = Setting as first ever frame/keyframe, resetting counters");
                // reset static variables
                kfIdCounter = 0;
                idCounter   = 0;
                id          = 0;
                // initial pose is zero translation with IMU rotation
                pose.setIdentity();
                setPoseRotationFromImu(true);
            } else {
                ROS_INFO("FRA > Setting Frame [%d] as keyframe", id);
            }

            /// TODO: detect, extract (sift?)
            double time = (ros::WallTime::now()-t0).toSec();
            ROS_INFO("FRA < Frame [%d] set as keyframe [%d] in [%.1fms]", id, kfId,time*1000.);
        }



        /// All functions
        float getQuality() const {
            return quality;
        }

        // Returns image size in pixels
        cv::Size getSize() const{
            return image.size();
        }

        // Gets IMU roll
        float getRoll() const{
            return roll;
        }

        // Convers the internal eigen pose to a TF pose
        const tf::Pose getTFPose() const {
            //TODO
            tf::Pose pose;
            tf::poseEigenToTF(getPose(), pose);
            return pose;
        }

        // return stamped transform with current time and pose in world frame
        const tf::StampedTransform getStampedTransform() const {
            if (kfId<0){
                return tf::StampedTransform(getTFPose(), ros::Time::now(), "/world", "/F"); /// TODO: world frame!
            } else {
                return tf::StampedTransform(getTFPose(), ros::Time::now(), "/world", "/KF_"+boost::lexical_cast<std::string>(kfId)); /// TODO: world frame!
            }
        }

        // returns the eigen pose of this frame in the world frame
        const Pose& getPose() const {
            return pose;
        }

        // Prepares the keyframe for removal
        void prepareRemoval(){
            ROS_WARN("Frame [%d|%d] Preparing for removal", getId(), getKfId());
            clearAllPoints();

        }

        // gets the optical axis in the world frame
        const Eigen::Vector3d getOpticalAxisBearing() const {
            return pose.linear()*Eigen::Vector3d(0., 0., 1.);
        }

        // gets the optical center in the world frame
        const Eigen::Vector3d getOpticalCenter() const {
            return pose.translation();
        }

        // Sets the pose from a 3x4 mat, converting it to an eigen affine3d
        void setPose(Eigen::Affine3d& t){
            pose = t;
            hasPoseEstimate = true;
        }

        // sets the pose from the imu rotation. This is usually used on the very first keyframe
        void setPoseRotationFromImu(bool inverse = false){
            pose.linear() = imuAttitudeCam;
            if (inverse){
                pose.linear().transposeInPlace();
            }

            hasPoseEstimate = true;
        }

        // Returns true if this frame is not the result of the default constructor
        bool isInitialised() const{
            return initialised;
        }

        // Gets the imu rotation recorded at the time the frame was taken.
        const opengv::rotation_t& getImuRotationCam() const {
            return imuAttitudeCam;
        }

        // Fetches the cameraInfo used by ros for the current rectification output
        const sensor_msgs::CameraInfoPtr& getCamInfo() const {
            return cameraModel.getCamInfo();
        }

        // returns the original preprocessed image the frame was initialised with. Cannot be changed
        const cv::Mat& getImage() const{
            return image;
        }

        // Creates a new image with visual information displayed on it
        cv::Mat getVisualImage() const{
            cv::Mat img;

            // Draw keypoints if they exist, also makes img colour
            cv::drawKeypoints(image, keypointsImg, img, CV_RGB(0,90,20));
            OVO::putInt(img, keypointsImg.size(), cv::Point(10,1*25), CV_RGB(0,96,0),  true,"KP:");

            // Draw Frame ID
            OVO::putInt(img, id, cv::Point2f(img.cols-95,img.rows-4*25), CV_RGB(0,110,255), true,  "FID:");

            // if a kf, draw id
            if (kfId>=0){
                OVO::putInt(img, kfId, cv::Point2f(img.cols-95,img.rows-5*25), CV_RGB(0,110,255), true,  "KID:");
            }

            /// draw depth points
/*            if (worldPoints3d.size()>0){
                opengv::points_t pts3d = worldPoints3d;
                OVO::transformPoints(getPose().inverse(), pts3d);
                Eigen::VectorXd dists(pts3d.size());
                for (uint i=0; i<pts3d.size(); ++i){
                    dists[i] = pts3d[i].norm();
                }

                float range_max = dists.maxCoeff();
                float range_min = dists.minCoeff();
                float range_mean = dists.mean();

                // draw colour coded depth points
                cv::Mat imgOverlay = cv::Mat::zeros(img.size(), CV_8UC3);

                // aliged
                if (voMode){
                    for (uint i=0; i<pts3d.size(); ++i){
                        cv::circle(imgOverlay, pointsImg[i], 3, OVO::getColor(range_min, range_max, dists[i]), CV_FILLED, CV_AA);
                    }
                } else {
                    for (uint i=0; i<pts3d.size(); ++i){
                        cv::circle(imgOverlay, pointsImg[VOinliers[i]], 3, OVO::getColor(range_min, range_max, dists[i]), CV_FILLED, CV_AA);
                    }
                }
                OVO::putInt(img, pts3d.size(), cv::Point2f(img.cols-95,img.rows-6*25), CV_RGB(0,180,110), true,  "WPT:"); // numner of 3d points frames can triangulate from

                cv::addWeighted(img, 1.0, imgOverlay, 0.75, 0, img);


                //OVO::putInt(img, idx.size(), cv::Point(10,3*25), CV_RGB(0,96*2,0),  true, "VO:");
                OVO::putInt(img, range_min,  cv::Point(10,5*25), OVO::getColor(range_min, range_max, range_min),  false , "MinR:");
                OVO::putInt(img, range_mean, cv::Point(10,6*25), OVO::getColor(range_min, range_max, range_mean), false , "AvgR:");
                OVO::putInt(img, range_max,  cv::Point(10,7*25), OVO::getColor(range_min, range_max, range_max),  false , "MaxR:");



           }
*/

            // Draw RPY
            drawRPY(img);

            // Draw timing info
            OVO::putInt(img, timePreprocess*1000., cv::Point(10,img.rows-6*25),CV_RGB(200,0,180), false, "P:");
            if (!keypointsImg.empty()){
                OVO::putInt(img, timeDetect*1000., cv::Point(10,img.rows-5*25), CV_RGB(200,0,180), false, "D:");
            }
            if (descriptors.rows>0){
                OVO::putInt(img, timeExtract*1000., cv::Point(10,img.rows-4*25), CV_RGB(200,0,180), false, "E:");
            }

            // Draw artificial horizon using CameraModel (eg it might be a curve too)
            return img;
        }

        // Gets all keypoints. Computes them if required/outdated type unless dontCompute is true. Uses points as kps if possible.
        KeyPoints& getKeypoints(bool dontCompute=false){            
            if (!dontCompute){
                if (keypointsImg.empty()){
                    computeKeypoints();
                } else if (getDetId() != detector.getDetectorId()){
                    // we have outdated keypoints - recompute
                    ROS_INFO("FRA > Keypoints outdated [%d != %d], redetecting", getDetId(), detector.getDetectorId());
                    computeKeypoints();
                    ROS_INFO("FRA < Keypoints redetected");
                }
            }
            return keypointsImg;
        }

        // Clears all point types (rotated points, keypoints, bearings, desciptors)
        void clearAllPoints(){            
            ROS_INFO("FRA = Clearing all Points Frame [%d | %d]", getId(), getKfId());
            keypointsRotated.clear();
            keypointsImg.clear();
            bearings = Eigen::MatrixXd();
            descriptors = cv::Mat();
            landmarkRefs.clear();
            //descId = -1;
            //detId = -2; // unknown type
        }

        const Eigen::Matrix3d& getImu2Cam() const{
            /// Returns the imu2Cam transform
            return imu2cam;
        }

        // Converts keypoints to poitns and returns them. Used for KLT refinement TODO: cache
        Points2f getPoints() const {
            Points2f p;
            cv::KeyPoint::convert(keypointsImg, p);
            return p;
        }

        // Returns an image pyramid, computes it if required. Recomputes if the window size  changed. Used for KLT refineminement
        const Mats& getPyramid(const cv::Size& winSize, const int maxLevel, const bool withDerivatives=true, int pyrBorder=cv::BORDER_REFLECT_101, int derivBorder=cv::BORDER_CONSTANT){
            /// return image pyramid. Compute if required
            if (pyramid.empty() || winSize != pyramidWindowSize){
                cv::buildOpticalFlowPyramid(image, pyramid, winSize, maxLevel, withDerivatives, pyrBorder, derivBorder, true); //not sure about last flag reuseInput
                pyramidWindowSize = winSize;
            }
            return pyramid;
        }

        // Gets rotated keypoints. Computes them if required.
        const KeyPoints& getRotatedKeypoints(bool aroundOptical=false){            
            ROS_INFO("FRA = Getting rotated keypoints [Frame %d]", getId());
            if (!keypointsRotated.empty()){
                // we already have some
            } else if (!keypointsImg.empty()){
                // compute them from out keypoints if we have some
                keypointsRotated = cameraModel.rotatePoints(keypointsImg, -getRoll(), aroundOptical);
            } else {
                ROS_WARN("FRA = Asked for rotated keypoints but dont have any keypoints to compute them from");
            }
            return keypointsRotated;
        }

        // Gets descriptors. Computes them if they are empty or the wrong type (determined from the current set extraxtor)
        const cv::Mat& getDescriptors(bool withLMOnly=false){
            if (descriptors.empty()) {
                // we have no descriptors
                extractDescriptors();
            } else {
                // we have descriptrs, check if they are still okay
                if (getDescriptorId() != detector.getExtractorId() ){
                    // we switched the type
                    ROS_WARN("FRA = Descriptor type changed [%d vs %d], recomputing descriptors for frame [%d]", getDescriptorId(), detector.getExtractorId(), getId());
                    extractDescriptors();
                }
            }

            if (withLMOnly){
                ROS_ASSERT(landmarkRefs.size()>0);





            }

            return descriptors;
        }


        // Extracts descriptors, setting the frames desc type. Computes/recomputes Keypoints if required
        void extractDescriptors(){
            getKeypoints(); // updates them if required
            uint kpsize = keypointsImg.size();
            ROS_INFO("FRA > Computing [%lu] descriptors for frame [id: %d]", keypointsImg.size(), id);
            ros::WallTime t0 = ros::WallTime::now();
            detector.extract(image, keypointsImg, descriptors, descriptorId, getRoll() );
            // detector might actually remove/add keypoints. IN this case it is important to realign existing data
            if (kpsize!=0 && kpsize != keypointsImg.size()){
                ROS_INFO("FRA = Detecting changed kp size, resetting data aligned with these");
                cv::Mat d;
                KeyPoints kps;
                std::swap(keypointsImg, kps);
                std::swap(descriptors, d);
                clearAllPoints();
                std::swap(keypointsImg, kps);
                std::swap(descriptors, d);
            }
            timeExtract = (ros::WallTime::now()-t0).toSec();
            ROS_INFO("FRA < Computed [%d] descriptors for frame [id: %d] in [%.1fms]", descriptors.rows, id,timeExtract*1000.);
        }

        /// Computes unit bearing vectors projected from the optical center through the rectified (key) points on the image plane
        const Eigen::MatrixXd& getBearings(){            
            if (bearings.rows()==0){
                ROS_INFO("FRA > Computing bearing vectors and rectified points for frame [id: %d]", id);
                ros::WallTime t0 = ros::WallTime::now();
                if (keypointsImg.size()>0){
                    cameraModel.bearingVectors(keypointsImg, bearings, pointsRect);
                } else {
                    ROS_WARN("FRA = Asked for bearings but dont have any keypoints to compute them from");
                }
                ROS_INFO("FRA < Computed [%ld] bearing vectors and rectified points [%.1fms]", pointsRect.rows(), 1000.*(ros::WallTime::now()-t0).toSec());
            }
            return bearings;
        }


        const Eigen::MatrixXd& getRectifiedPoints(){
            /// gets rectified points. If we dont have any, compute them (and bearing vectors)
            if (pointsRect.rows()==0){
                getBearings();
            }
            return pointsRect;
        }


        // Gets single descriptor
        const cv::Mat getDescriptor(int id) const{
            ROS_ASSERT(id <= descriptors.rows);
            ROS_ASSERT(id >=0);
            return descriptors.row(id);
        }
        // get single bearing
        const Bearing getBearing(int id) const{
            ROS_ASSERT(id <= bearings.rows());
            ROS_ASSERT(id >=0);
            return bearings.row(id);
        }

        // get single KeyPoints
        const cv::KeyPoint& getKeypoint(int id) const{
            ROS_ASSERT(id <= static_cast<int>(keypointsImg.size()));
            ROS_ASSERT(id >=0);
            return keypointsImg[id];
        }


        int getId() const {
            return id;
        }

        int getKfId() const {
            return kfId;
        }

        int getDescType() const{
            return detector.getDescriptorType();
        }

        int getDescSize() const{
            return detector.getDescriptorSize();
        }

        int getDescriptorId() const {
            return descriptorId;
        }

        int getDetId() const {
            return detectorId;
        }


        static void setParameter(ollieRosTools::VoNode_paramsConfig &config, uint32_t level){
            ROS_INFO("FRA > SETTING PARAMS");
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

            ROS_INFO("FRA < PARAMS SET");

        }


        friend std::ostream& operator<< (std::ostream& stream, const Frame& frame) {
            stream << "[ID:" << std::setw(5) << std::setfill(' ') << frame.id << "]"
                   << "[KF:"  << std::setw(3) << std::setfill(' ') << frame.kfId<< "]"
                   << "[KP:"  << std::setw(4) << std::setfill(' ') << frame.keypointsImg.size() << "]"                   
                   << "[RP:"  << std::setw(4) << std::setfill(' ') << frame.keypointsRotated.size() << "]"
                   << "[ D:"  << std::setw(4) << std::setfill(' ') << frame.descriptors.rows << "]"
                   << "[BV:"  << std::setw(4) << std::setfill(' ') << frame.bearings.rows() << "]"
                   << "[RE:"  << std::setw(4) << std::setfill(' ') << frame.pointsRect.rows() << "]"
                   << "[LM:"  << std::setw(4) << std::setfill(' ') << frame.landmarkRefs.size() << "]"
                   << "[TP:"  << std::setw(4) << std::setfill(' ') << std::setprecision(1) << frame.timePreprocess << "]"
                   << "[TD:"  << std::setw(4) << std::setfill(' ') << std::setprecision(1) << frame.timeDetect << "]"
                   << "[TE:"  << std::setw(4) << std::setfill(' ') << std::setprecision(1) << frame.timeExtract << "]";

            return stream;
        }
};

typedef cv::Ptr<Frame> FramePtr;

#endif // FRAME_HPP
