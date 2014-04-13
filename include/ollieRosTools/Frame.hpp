#ifndef FRAME_HPP
#define FRAME_HPP


#include <iostream>
#include <iomanip>      // std::setprecision
#include <vector>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

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

        // Type of descriptor / detector used
        int descriptorId;
        int detectorId;


        // Features - ALL OF THESE SHOULD ALWAYS BE ALIGNED!
        KeyPoints keypointsImg;
        KeyPoints keypointsRotated; // TODO: should be eigen
        Points2f pointsImg; // points and keypoints.pt should match. These are the detected points in the potentially unrectified image
        cv::Mat descriptors;
        Eigen::MatrixXd bearings;
        Eigen::MatrixXd pointsRect; // rectified points, align with all of the above


        bool voMode;
        Ints VOinliers;
        KeyPoints VOkeypointsImg;
        KeyPoints VOkeypointsRotated; // TODO: should be eigen
        Points2f VOpointsImg; // points and keypoints.pt should match. These are the detected points in the potentially unrectified image
        cv::Mat VOdescriptors;
        Eigen::MatrixXd VObearings;
        Eigen::MatrixXd VOpointsRect; // rectified points, align with all of the above


        //
        static CameraATAN cameraModel;
        static Detector detector;
        static PreProc preproc;

        /// KF ONLY ///////////////////////////////////////////////////////////////////////////////////////////////////
        /// If a keyframe, this is used for other frames to track against. Aligned with descs, keypoints, points, rotated kps, bearing vectors
        /// also used for regular frames, just to visualise depth
        opengv::points_t worldPoints3d;

        /// If a keyframe, this contains references to map points
        std::vector<PointPtr> mapPointRefs;
        KeyPoints             mapKeyPoints;
        cv::Mat               mapDescriptors;
        Eigen::MatrixXd       mapbearings;


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
            ROS_ASSERT_MSG(!voMode, "FRA = Computing keypoints in non vo mode does not make sense");
            ROS_INFO("FRA > Computing Keypoints frame [id: %d]", getId());
            clearAllPoints();
            ros::WallTime t0 = ros::WallTime::now();
            detector.detect(image, keypointsImg, detectorId, mask);
            timeDetect = (ros::WallTime::now()-t0).toSec();
            ROS_INFO("FRA < Computed [%lu] Keypoints of [Type: %d] for frame [id: %d] in  [%.1fms] ", keypointsImg.size(), getDetId(), getId(),timeDetect*1000.);
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

            voMode = false;

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

            ROS_INFO("FRA < NEW FRAME CREATED [ID: %d]", id);


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


        /// KF ONLY FUNCTIONS
        // Gets Desc at specified row
        const cv::Mat getMapDescriptor(const int rowNr) const {
            return mapDescriptors.row(rowNr);
        }



        visualization_msgs::Marker getWorldPointsMarker(int id=0, float scale=1.0, float brightness = 1.0){
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

        }

        // Sets frame as keyframe, prepares it for bundle adjustment
        void setAsKF(bool first=false){
            ros::WallTime t0 = ros::WallTime::now();
            ROS_INFO("FRA > Setting Frame [%d] as keyframe", id);
            kfId = ++kfIdCounter;
            if(first){
                ROS_INFO("FRA = Setting as first ever frame/keyframe, resetting counters");
                // reset static variables
                kfIdCounter = 0;
                idCounter   = 0;
                id          = 0;
                // initial pose is zero translation with IMU rotation
                pose.setIdentity();
                setPoseRotationFromImu();
            }


            /// TODO: detect, extract (sift?)
            double time = (ros::WallTime::now()-t0).toSec();
            ROS_INFO("FRA < Frame [%d] set as keyframe [%d] in [%.1fms]", id, kfId,time*1000.);
        }



        /// All functions
        float getQuality() const {
            return quality;
        }

        cv::Size getSize(){
            return image.size();
        }

        // Convers the internal eigen pose to a TF pose
        const tf::Pose getTFPose() const {
            //TODO
            tf::Pose pose;
            tf::poseEigenToTF(getPose(), pose);
            return pose;
        }

        const tf::StampedTransform getStampedTransform() const {
            if (kfId<0){
                return tf::StampedTransform(getTFPose(), ros::Time::now(), "/cf_xyz", "/F");
            } else {
                return tf::StampedTransform(getTFPose(), ros::Time::now(), "/cf_xyz", "/KF_"+boost::lexical_cast<std::string>(kfId));
            }
        }

        // returns the eigen pose of this frame in the world frame
        const Pose& getPose() const {
            return pose;
        }

        // Sets the pose from a 3x4 mat, converting it to an eigen affine3d
        void setPose(Eigen::Affine3d& t){
            pose = t;
        }

        // sets the pose from the imu rotation. This is usually used on the very first keyframe
        void setPoseRotationFromImu(){
            pose.linear() = imuAttitudeCam;
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

            // if a kf
            if (kfId>=0){
                OVO::putInt(img, kfId, cv::Point2f(img.cols-95,img.rows-5*25), CV_RGB(0,110,255), true,  "KID:");                
                OVO::putInt(img, mapPointRefs.size(), cv::Point2f(img.cols-95,img.rows-7*25), CV_RGB(0,180,110), true,  "MPT:");
            }




            /// draw depth points
            if (worldPoints3d.size()>0){
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

            // Draw RPY
            drawRPY(img);

            // Draw timing
            OVO::putInt(img, timePreprocess*1000., cv::Point(10,img.rows-6*25),CV_RGB(200,0,200), false, "P:");
            if (!keypointsImg.empty()){
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

        bool hasPoints() const {
            return keypointsImg.size()>0 || pointsImg.size()>0;
        }


        KeyPoints& getKeypoints(bool dontCompute=false, bool allowKLT=true, bool forceCompute = false){
            /// Gets keypoints. Computes them if required/outdated type unless dontCompute is true. Uses points as kps if possible.
            ROS_ASSERT(!(forceCompute&&dontCompute));

            if (forceCompute){
                computeKeypoints();
            } else if (keypointsImg.empty()){
                // We dont have key points
                if (!pointsImg.empty() && allowKLT){
                    // use klt points as keypoints
                    cv::KeyPoint::convert(pointsImg, keypointsImg);
                } else {
                    // we dont have points either or dont want to use them
                    if (!dontCompute){
                        // compute  points
                        computeKeypoints();
                    }
                }
            } else if (!dontCompute && getDetId() != detector.getDetectorId()){
                // we have outdated keypoints - recompute
                ROS_INFO("FRA > Keypoints outdated [%d != %d], redetecting", getDetId(), detector.getDetectorId());
                computeKeypoints();
                ROS_INFO("FRA < Keypoints redetected");
            }

            return keypointsImg;
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
            if (!pointsImg.empty()){
                // have points, do nothing
            } else  if (!keypointsImg.empty()){
                // but have keypoints
                cv::KeyPoint::convert(keypointsImg, pointsImg);
            } else if (voMode){
                if (!VOpointsImg.empty()){
                    // use points from non vo
                    OVO::vecReduceInd(VOpointsImg, pointsImg, VOinliers);
                } else if (!keypointsImg.empty()){
                    // but have keypoints from vo
                    OVO::vecReduceInd(VOkeypointsImg, keypointsImg, VOinliers);
                    cv::KeyPoint::convert(keypointsImg, pointsImg);
                }
            } else if (!dontCompute){
                // compute them
                ROS_ASSERT_MSG(!voMode, "FRA = IT does not make sense to compute KPS in VoMode: %s", __SHORTFILE__);
                computeKeypoints();
                cv::KeyPoint::convert(keypointsImg, pointsImg);
            }

            return pointsImg;
        }


        void swapKeypoints(KeyPoints& kps, bool updateOnly = false){
            /// Swap keypoints and remove everything that could have come from previous ones
            // IF updateOnly is set, we just "update" the keypoints.
            if (updateOnly){
                // update points only
                ROS_ASSERT(kps.size()==keypointsImg.size());
                std::swap(keypointsImg, kps);
                // might still need to update the pts too
                if (pointsImg.size()>0){
                    cv::KeyPoint::convert(keypointsImg, pointsImg);
                }
            } else {
                // replace points
                clearAllPoints();
                std::swap(keypointsImg, kps);
            }
        }

        void swapPoints(Points2f& pts, bool updateOnly = false){
            /// Swap points and remove everything that could have come from previous ones
            if (updateOnly){
                // update points only
                ROS_ASSERT(pts.size()==pointsImg.size());
                std::swap(pointsImg, pts);
                // might still need to update the kpts too
                if (pointsImg.size()>0){
                    cv::KeyPoint::convert(pointsImg, keypointsImg);
                }
            } else {
                // replace points
                clearAllPoints();
                std::swap(pointsImg, pts);
            }

        }


        void clearAllPoints(){
            /// Clears all point types, keeping them in sync
            ROS_INFO("FRA = Clearing all Points [%d | %d]", getId(), getKfId());
            keypointsRotated.clear();
            keypointsImg.clear();
            pointsImg.clear();
            bearings = Eigen::MatrixXd();
            worldPoints3d.clear();
            VOinliers.clear();
            descriptors = cv::Mat();

            if(voMode){
                ROS_WARN("FRA = Reset frame while in VO MODE; returning out of VO Mode");
                setModeVoOnly(false);
                clearAllPoints(); // reset the rest too
            }
            //descId = -1;
            //detId = -2; // unknown type
        }

        const Eigen::Matrix3d& getImu2Cam(){
            return imu2cam;
        }






        // Sets worldPoints3d and aligns them to all other points by removing non inliers
        /// TODO : not supported ot call this function mutliple times.
        void setWorldPoints(const Ints& inliers, const opengv::points_t pts3d, bool setVoMode=true){
            ROS_ASSERT_MSG(VOinliers.size()==0, "Not supported to setWorldPoints multiple times: %s", __SHORTFILE__);
            ROS_INFO("FRA > Removing non inliers from KP/P/Desc/BV/RP");
            worldPoints3d = pts3d;
            setVoPoints(inliers, setVoMode);
            VOinliers = inliers;
            ROS_INFO("FRA < Added world points");

        }

        const opengv::points_t& getWorldPoints3d() const {
            return worldPoints3d;
        }

        void setModeVoOnly(bool voInliersOnly = true){
            /// if this is set, all the getter functions only return points, matches, etc aligned with vo inliers
            ROS_ASSERT_MSG(worldPoints3d.size()>0 || !voInliersOnly, "Cannot set to Vo Only Mode if there are no vo inliers: %s", __SHORTFILE__);

            if (voMode != voInliersOnly){
                std::swap(VObearings, bearings);
                std::swap(VOdescriptors, descriptors);
                std::swap(VOpointsRect, pointsRect);
                std::swap(VOkeypointsImg, keypointsImg);
                std::swap(VOkeypointsRotated, keypointsRotated);
                std::swap(VOpointsImg, pointsImg);
                voMode = voInliersOnly;
                ROS_INFO("FRA = FRAME [%d|%d]: VO MODE NOW %s", getId(), getKfId(), voMode ? "ON" : "OFF");
            }
        }

        bool getVoMode() const {
            return voMode;
        }



//        void reducePointsToWorldPoints(){
//            /// required world points and worldPoints3dInliers to be set already.
//            /// Alignts the rest
//            ROS_INFO("FRA > Alinging world points with all point types for frame [id: %d]", getId());
//            reduceAllPoints(worldPoints3dInliers);
//        }

        void setVoPoints(const Ints& inliers, bool setVoMode = true){
            /// Same as clear all points but instead of removing all, it keeps the ones at position ind
            ROS_INFO("FRA > Keeping %lu/%lu inliers from all point types for frame [id: %d]", inliers.size(), std::max(pointsImg.size(), keypointsImg.size()), getId());

            ROS_ASSERT_MSG(VOinliers.size()==0, "FRA = Should not be able to update VoPoints twice: %s", __SHORTFILE__);

            ROS_INFO_STREAM("FRA = FRAME STATE BEFORE:\n"<< *this);

            if (bearings.rows()>0){
                VObearings = Eigen::MatrixXd(inliers.size(), 3);
                for (uint i=0; i<inliers.size(); ++i){
                    VObearings.row(i) = bearings.row(inliers[i]);
                }
                ROS_INFO("FRA = Bearing Vectors updated");
            } else {
                ROS_INFO("FRA = No Bearing Vectors to update");
            }

            if (descriptors.rows>0){
                OVO::matReduceInd(descriptors, VOdescriptors, inliers);
                ROS_INFO("FRA = Descriptors Vectors updated");
            } else {
                ROS_INFO("FRA = No Descriptors to update");
            }

            if (keypointsImg.size()>0){
                OVO::vecReduceInd<KeyPoints>(keypointsImg, VOkeypointsImg, inliers);
                ROS_INFO("FRA = Keypoints updated");
            } else {
                ROS_INFO("FRA = No Keypoints to update");
            }

            if (pointsImg.size()>0){
                OVO::vecReduceInd<Points2f>(pointsImg, VOpointsImg, inliers);
                ROS_INFO("FRA = Points 2d updated");
            } else {
                ROS_INFO("FRA = No Points 2d to update");
            }

            if (keypointsRotated.size()>0){
                OVO::vecReduceInd<KeyPoints>(keypointsRotated, VOkeypointsRotated, inliers);
                ROS_INFO("FRA = Rotated keypoints updated");
            } else {
                ROS_INFO("FRA = No Rotated keypoints to update");
            }
            if (pointsRect.size()>0){
                VOpointsRect = Eigen::MatrixXd(inliers.size(), 2);
                for (uint i=0; i<inliers.size(); ++i){
                    VOpointsRect.row(i) = pointsRect.row(inliers[i]);
                }
                ROS_INFO("FRA = Rectified Points updated");
            } else {
                ROS_INFO("FRA = No Rectified Points to update");
            }


            ROS_INFO("FRA < Outliers removed");

            ROS_INFO_STREAM("FRA = FRAME STATE AFTER:\n"<< *this);

            if (setVoMode){
                setModeVoOnly(true);
                ROS_INFO_STREAM("FRA = FRAME STATE AFTER SWITCH:\n"<< *this);
            }



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
            ROS_INFO("FRA = Getting rotated keypoints [Frame %d]", getId());
            if (!keypointsRotated.empty()){
                // we already have some
            } else {
                if (voMode) {
                    if (!VOkeypointsRotated.empty()){
                        // we have some in non VO mode
                        OVO::vecReduceInd(VOkeypointsRotated, keypointsRotated, VOinliers);
                    } else if (!VOkeypointsImg.empty()){
                        // we didnt have rotated ones but we have some key points, compute the rotated ones from there
                        VOkeypointsRotated = cameraModel.rotatePoints(VOkeypointsImg, -getRoll(), aroundOptical);
                        OVO::vecReduceInd(VOkeypointsRotated, keypointsRotated, VOinliers);
                    } else if (!VOpointsImg.empty()){
                        // we didnt have keypoints, but we had points, compute rotated keypoints from there
                        cv::KeyPoint::convert(VOpointsImg, VOkeypointsImg);
                        VOkeypointsRotated = cameraModel.rotatePoints(VOkeypointsImg, -getRoll(), aroundOptical);
                        OVO::vecReduceInd(VOkeypointsImg, keypointsImg, VOinliers);
                        OVO::vecReduceInd(VOkeypointsRotated, keypointsRotated, VOinliers);
                    }
                } else {
                    if (!keypointsImg.empty()){
                        // we didnt have rotated ones but we have some key points, compute the rotated ones from there
                        keypointsRotated = cameraModel.rotatePoints(keypointsImg, -getRoll(), aroundOptical);
                    } else if (!pointsImg.empty()){
                        // we didnt have keypoints, but we had points, compute rotated keypoints from there
                        cv::KeyPoint::convert(pointsImg, keypointsImg);
                        keypointsRotated = cameraModel.rotatePoints(keypointsImg, -getRoll(), aroundOptical);
                    }
                }
            }

            ROS_WARN_COND(keypointsRotated.size()==0, "FRA = Asked for rotated keypoints but dont have any points or keypoints to compute them from");


            return keypointsRotated;
        }


//        const KeyPoints& getRotatedKeypoints(bool aroundOptical=false){
//            /// Gets rotated keypoints. Computes them if required.
//            ROS_INFO("FRA = Getting rotated keypoints [Frame %d]", getId());
//            if (!keypointsRotated.empty()){
//                // we already have some
//            } else if (!keypointsImg.empty()){
//                // we didnt have rotated ones but we have some key points, compute the rotated ones from there
//                keypointsRotated = cameraModel.rotatePoints(keypointsImg, -getRoll(), aroundOptical);
//            } else if (!pointsImg.empty()){
//                // we didnt have keypoints, but we had points, compute rotated keypoints from there
//                cv::KeyPoint::convert(pointsImg, keypointsImg);
//                keypointsRotated = cameraModel.rotatePoints(keypointsImg, -getRoll(), aroundOptical);
//            } else if (voMode){
//                // we didnt have any at all. Maybe we are in VO mode and have some in non VO mode taht we can use
//                // get the from the non vo mode ones if possible
//                if(!VOkeypointsRotated.empty()){
//                    // we have some rotated kps, get inliers
//                    OVO::vecReduceInd(VOkeypointsRotated, keypointsRotated, VOinliers);
//                } else if (!VOkeypointsImg.empty()) {
//                    // we didnt have rotated ones, but we have kps
//                    OVO::vecReduceInd(VOkeypointsImg, keypointsImg, VOinliers);
//                    keypointsRotated = cameraModel.rotatePoints(keypointsImg, -getRoll(), aroundOptical);
//                } else if (!VOpointsImg.empty()) {
//                    // we didnt have kps, but we have points))
//                    OVO::vecReduceInd(VOpointsImg, pointsImg, VOinliers);
//                    cv::KeyPoint::convert(pointsImg, keypointsImg);
//                    keypointsRotated = cameraModel.rotatePoints(keypointsImg, -getRoll(), aroundOptical);
//                } else {
//                    ROS_WARN("FRA = Asked for rotated keypoints but dont have any points or keypoints to compute them from");
//                }
//            }
//            return keypointsRotated;
//        }

        const cv::Mat& getDescriptors(){
            /// Gets descriptors. Computes them if they are empty or the wrong type (determined from the current set extraxtor)
            if (descriptors.empty()) {
                // we have no descriptors
                if (!VOdescriptors.empty() && voMode){
                    // We have some in non vo mode, select inliers
                    OVO::matReduceInd(VOdescriptors, descriptors, VOinliers);
                } else {//  if ( keypointsImg.empty() || getDescId() != detector.getExtractorId()){
                    if (voMode){
                        ROS_WARN("FRA = Computing descriptors in VO mode. First doing all, then selecting inliers");
                        setModeVoOnly(false);
                        extractDescriptors();
                        OVO::matReduceInd(descriptors, VOdescriptors, VOinliers);
                        setModeVoOnly(true);
                    } else {
                        extractDescriptors();
                    }
                }
            } else {
                // we have descriptrs
                if (getDescriptorId() != detector.getExtractorId() ){
                    // we switched the type
                    ROS_WARN("FRA = Descriptor type changed [%d vs %d], recomputing descriptors for frame [%d]", getDescriptorId(), detector.getExtractorId(), getId());
                    extractDescriptors();
                }
            }
            return descriptors;
        }

        void extractDescriptors(){
            getKeypoints(); // computes keypoints if required
            ros::WallTime t0 = ros::WallTime::now();
            ROS_INFO("FRA > Computing [%lu] descriptors for frame [id: %d]", keypointsImg.size(), id);
            uint kpsize = keypointsImg.size();
            detector.extract(image, keypointsImg, descriptors, descriptorId, getRoll() );
            // detector might actually remove/add keypoints. IN this case it is important to realign existing data
            if (kpsize!=0 && kpsize != keypointsImg.size()){
                ROS_INFO("FRA = Detecting changed kp size, resetting data aligned with these");
                ROS_ASSERT_MSG(!voMode && worldPoints3d.size()==0 && VOdescriptors.rows==0, "FRA = Data probably misaligned");
                cv::Mat d;
                KeyPoints kps;
                std::swap(keypointsImg, kps);
                std::swap(descriptors, d);
                clearAllPoints();
                std::swap(keypointsImg, kps);
                std::swap(descriptors, d);
                detectorId = detector.getDetectorId();
                descriptorId = detector.getExtractorId();
            }

            timeExtract = (ros::WallTime::now()-t0).toSec();
            ROS_INFO("FRA < Computed [%d] descriptors for frame [id: %d] in [%.1fms]", descriptors.rows, id,timeExtract*1000.);
        }

        const Ints& getVoInliers() const {
            return VOinliers;
        }

        const Eigen::MatrixXd& getBearings(){
            /// Computes unit bearing vectors projected from the optical center through the rectified (key) points on the image plane
            if (bearings.rows()==0){
                if (voMode && VObearings.rows()>0){
                    // we are in vo mode and have vectors in non vo mode, select inliers and be done
                    bearings = Eigen::MatrixXd(VOinliers.size(), 3);
                    for (uint i=0; i<VOinliers.size(); ++i){
                        bearings.row(i) = VObearings.row(VOinliers[i]);
                    }
                } else {
                    if (voMode){
                        ROS_WARN("FRA = Computing bearings vectos in VO mode. First doing all, then selecting inliers");
                        setModeVoOnly(false);
                        if (pointsImg.size()>0){
                            cameraModel.bearingVectors(pointsImg, bearings, pointsRect);
                        } else if (keypointsImg.size()>0){
                            cv::KeyPoint::convert(keypointsImg, pointsImg);
                            cameraModel.bearingVectors(pointsImg, bearings, pointsRect);
                        }
                        ROS_INFO("FRA < Computed [%lu] bearing vectors and rectified points", pointsImg.size());

                        VObearings = Eigen::MatrixXd(VOinliers.size(), 3);
                        for (uint i=0; i<VOinliers.size(); ++i){
                            VObearings.row(i) = bearings.row(VOinliers[i]);
                        }
                        setModeVoOnly(true);
                    } else {
                        ROS_INFO("FRA > Computing bearing vectors and rectified points for frame [id: %d]", id);
                        if (pointsImg.size()>0){
                            cameraModel.bearingVectors(pointsImg, bearings, pointsRect);
                        } else if (keypointsImg.size()>0){
                            cv::KeyPoint::convert(keypointsImg, pointsImg);
                            cameraModel.bearingVectors(pointsImg, bearings, pointsRect);
                        }
                        ROS_INFO("FRA < Computed [%lu] bearing vectors and rectified points", pointsImg.size());
                    }
                }
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


        int getId() const {
            return id;
        }
        int getKfId() const {
            return kfId;
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


        friend std::ostream& operator<< (std::ostream& stream, const Frame& frame) {
            stream << "[ID:" << std::setw(5) << std::setfill(' ') << frame.id << "]"
                   << "[VO:" << (frame.voMode ? " ON ]": " OFF]")
                   << "[KF:"  << std::setw(3) << std::setfill(' ') << frame.kfId<< "]"
                   << "[KP:"  << std::setw(4) << std::setfill(' ') << frame.keypointsImg.size() << "]"
                   << "[ P:"  << std::setw(4) << std::setfill(' ') << frame.pointsImg.size() << "]"
                   << "[RP:"  << std::setw(4) << std::setfill(' ') << frame.keypointsRotated.size() << "]"
                   << "[ D:"  << std::setw(4) << std::setfill(' ') << frame.descriptors.rows << "]"
                   << "[BV:"  << std::setw(4) << std::setfill(' ') << frame.bearings.rows() << "]"
                   << "[TP:"  << std::setw(4) << std::setfill(' ') << std::setprecision(1) << frame.timePreprocess << "]"
                   << "[TD:"  << std::setw(4) << std::setfill(' ') << std::setprecision(1) << frame.timeDetect << "]"
                   << "[TE:"  << std::setw(4) << std::setfill(' ') << std::setprecision(1) << frame.timeExtract << "]";

            stream << "\n[ID:" << std::setw(5) << std::setfill(' ') << frame.id << "]"
                   << "[ALTERNA]"
                   << "[KF:"  << std::setw(3) << std::setfill(' ') << frame.kfId<< "]"
                   << "[KP:"  << std::setw(4) << std::setfill(' ') << frame.VOkeypointsImg.size() << "]"
                   << "[ P:"  << std::setw(4) << std::setfill(' ') << frame.VOpointsImg.size() << "]"
                   << "[RP:"  << std::setw(4) << std::setfill(' ') << frame.VOkeypointsRotated.size() << "]"
                   << "[ D:"  << std::setw(4) << std::setfill(' ') << frame.VOdescriptors.rows << "]"
                   << "[BV:"  << std::setw(4) << std::setfill(' ') << frame.VObearings.rows() << "]"
                   << "[TP:"  << std::setw(4) << std::setfill(' ') << std::setprecision(1) << frame.timePreprocess << "]"
                   << "[TD:"  << std::setw(4) << std::setfill(' ') << std::setprecision(1) << frame.timeDetect << "]"
                   << "[TE:"  << std::setw(4) << std::setfill(' ') << std::setprecision(1) << frame.timeExtract << "]";

            return stream;
        }
};

typedef cv::Ptr<Frame> FramePtr;

#endif // FRAME_HPP
