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
        Mats pyramid;
        cv::Size pyramidWindowSize;
        opengv::rotation_t imuAttitude;
        Eigen::Affine3d pose; // transformation in the world frame
        double roll, pitch, yaw;        
        /// TODO not used yet
        double gyroX, gyroY, GyroZ;
        double accX, accY, accZ;

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

        /// KF ONLY ///////////////////////////////////////////////////////////////////////////////////////////////////
        /// If a keyframe, this is used for other frames to track against. Aligned with descs, keypoints, points, rotated kps, bearing vectors
        opengv::points_t worldPoints3d;
        Ints worldPoints3dInliers;

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
            ROS_INFO("FRA > Computing Keypoints frame [id: %d]", getId());
            keypointsRotated.clear();
            points.clear();
            descriptors = cv::Mat();
            ros::WallTime t0 = ros::WallTime::now();
            detector.detect(image, keypoints, detId, mask);
            timeDetect = (ros::WallTime::now()-t0).toSec();
            ROS_INFO("FRA < Computed [%lu] Keypoints [%.1fms] for frame [id: %d]", keypoints.size(), timeDetect*1000., getId());
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

            ROS_INFO("FRA < NEW FRAME CREATED [ID: %d]", id);
        }

        /// KF ONLY FUNCTIONS
        // Gets Desc at specified row
        const cv::Mat getMapDescriptor(const int rowNr) const {
            return mapDescriptors.row(rowNr);
        }

        // just for now
//        void setWorldPoints(const Points2f& pts2d, const opengv::points_t pts3d){
//            worldPoints3d = pts3d;
//            worldPoints2d = pts2d;
//        }


        // Sets worldPoints3d and aligns them to all other points by removing non inliers
        void setWorldPoints(const Ints& inliers, const opengv::points_t pts3d){
            ROS_INFO("FRA > Removing non inliers from KP/P/Desc/BV/RP");
            reduceAllPoints(inliers);
            worldPoints3d = pts3d;
            ROS_INFO("FRA > Aligned world points with all other things");
        }

        // same as above but doesnt remove points. Just used for vis of non-kf frames
        void setWorldPointsInliers(const Ints& inliers, const opengv::points_t pts3d){
            ROS_INFO("FRA > adding world points for vis");
            worldPoints3dInliers = inliers;
            worldPoints3d = pts3d;
            ROS_INFO("FRA < [%lu] world points and [%lu] inliers added for vis", worldPoints3d.size(), inliers.size());
        }

        const opengv::points_t& getWorldPoints3d() const {
            return worldPoints3d;
        }


        visualization_msgs::Marker getWorldPointsMarker(int id=0, float scale=1.0, float brightness = 1.0){
            visualization_msgs::Marker ms;
            ms.header.stamp = ros::Time::now();
            ms.ns = "worldPoints";
            ms.id = id;
            ms.header.frame_id = "/cf_xyz";
            ms.type = visualization_msgs::Marker::POINTS;
            ms.action = visualization_msgs::Marker::ADD;
            ms.scale.x = 0.05*scale;
            ms.scale.y = 0.05*scale;

            if (worldPoints3d.size()>0){
                opengv::points_t pts3d = worldPoints3d;
                OVO::transformPoints(getPose().inverse(), pts3d);
                Eigen::VectorXd dists(pts3d.size());
                for (uint i=0; i<pts3d.size(); ++i){
                    dists[i] = pts3d[i].norm();
                }

                float range_max = dists.maxCoeff();
                float range_min = dists.minCoeff();

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
        const Eigen::Affine3d& getPose() const {
            return pose;
        }

        // Sets the pose from a 3x4 mat, converting it to an eigen affine3d
        void setPose(const opengv::transformation_t& t){
            pose = t;
        }

        // sets the pose from the imu rotation. This is usually used on the very first keyframe
        void setPoseRotationFromImu(){
            pose.linear() = imuAttitude;
        }

        // Returns true if this frame is not the result of the default constructor
        bool isInitialised() const{
            return initialised;
        }

        // Gets the imu rotation recorded at the time the frame was taken.
        const opengv::rotation_t& getImuRotation() const {
            return imuAttitude;
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
            cv::drawKeypoints(image, keypoints, img, CV_RGB(0,90,20));
            OVO::putInt(img, keypoints.size(), cv::Point(10,1*25), CV_RGB(0,96,0),  true,"KP:");

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

                if (worldPoints3dInliers.size()>0){
                    // not aligned
                    for (uint i=0; i<worldPoints3dInliers.size(); ++i){
                        cv::circle(imgOverlay, points[worldPoints3dInliers[i]], 3, OVO::getColor(range_min, range_max, dists[i]), CV_FILLED, CV_AA);
                    }
                    OVO::putInt(img, worldPoints3dInliers.size(), cv::Point2f(img.cols-95,img.rows-6*25), CV_RGB(0,180,110), true,  "WPT:"); // numner of 3d points frames can triangulate from
                } else {
                    // aligned
                    ROS_ASSERT_MSG(pts3d.size()==points.size(), "2d and 3d points not aligned");
                    for (uint i=0; i<pts3d.size(); ++i){
                        cv::circle(imgOverlay, points[i], 3, OVO::getColor(range_min, range_max, dists[i]), CV_FILLED, CV_AA);
                    }
                    OVO::putInt(img, worldPoints3d.size(), cv::Point2f(img.cols-95,img.rows-6*25), CV_RGB(0,180,110), true,  "WPT:"); // numner of 3d points frames can triangulate from
                }
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
                    ROS_INFO("FRA > Keypoints outdated [%d != %d], redetecting", getDetId(), detector.getDetectorId());
                    computeKeypoints();
                    ROS_INFO("FRA < Keypoints redetected");
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
            clearAllPoints();
            std::swap(keypoints, kps);
        }
        void swapPoints(Points2f& pts){
            /// Swap points and remove everything that could have come from previous ones
            clearAllPoints();
            std::swap(points, pts);
        }


        void clearAllPoints(){
            /// Clears all point types, keeping them in sync
            keypointsRotated.clear();
            keypoints.clear();
            points.clear();
            bearings = Eigen::MatrixXd();
            worldPoints3d.clear();
            descriptors = cv::Mat();
            descId = -1;
            detId = -2; // unknown type
        }


        void reduceAllPoints(const Ints& inliers){
            /// Same as clear all points but instead of removing all, it keeps the ones at position ind
            ROS_INFO("FRA > Keeping %lu/%lu inliers from all point types for frame [id: %d]", inliers.size(), std::max(points.size(), keypoints.size()), getId());

            if (bearings.rows()>0){
                Eigen::MatrixXd bearingsTemp = Eigen::MatrixXd(inliers.size(), 3);
                for (uint i=0; i<inliers.size(); ++i){
                    bearingsTemp.row(i) = bearings.row(inliers[i]);
                }
                std::swap(bearingsTemp, bearings);
                ROS_INFO("FRA = Bearing Vectors updated");
            } else {
                ROS_INFO("FRA = No Bearing Vectors to update");
            }

            if (descriptors.rows>0){
                cv::Mat descriptorsTemp;
                for (uint i=0; i<inliers.size(); ++i){
                    descriptorsTemp.push_back(descriptors.row(inliers[i]));
                }
                std::swap(descriptorsTemp, descriptors);
                ROS_INFO("FRA = Descriptors Vectors updated");
            } else {
                ROS_INFO("FRA = No Descriptors to update");
            }

            if (keypoints.size()>0){
                KeyPoints keypointsTemp;
                for (uint i=0; i<inliers.size(); ++i){
                    keypointsTemp.push_back(keypoints[inliers[i]]);
                }
                std::swap(keypointsTemp, keypoints);
                ROS_INFO("FRA = Keypoints updated");
            } else {
                ROS_INFO("FRA = No Keypoints to update");
            }

            if (points.size()>0){
                 Points2f pointsTemp;
                for (uint i=0; i<inliers.size(); ++i){
                    pointsTemp.push_back(points[inliers[i]]);
                }
                std::swap(pointsTemp, points);
                ROS_INFO("FRA = Points 2d updated");
            } else {
                ROS_INFO("FRA = No Points 2d to update");
            }

            if (keypointsRotated.size()>0){
                KeyPoints keypointsRotatedTemp;
                for (uint i=0; i<inliers.size(); ++i){
                    keypointsRotatedTemp.push_back(keypointsRotated[inliers[i]]);
                }
                std::swap(keypointsRotatedTemp, keypointsRotated);
                ROS_INFO("FRA = Rotated keypoints updated");
            } else {
                ROS_INFO("FRA = No Rotated keypoints to update");
            }

            ROS_INFO("FRA < Outliers removed");
            ROS_INFO_STREAM("FRA = "<< *this);

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
                        ROS_WARN("FRA = Asked for rotated keypoints but dont have any points or keypoints to compute them from");
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
                ROS_INFO("FRA > Computing [%lu] descriptors for frame [id: %d]", keypoints.size(), id);
                detector.extract(image, keypoints, descriptors, descId, getRoll() );                
                timeExtract = (ros::WallTime::now()-t0).toSec();
                ROS_INFO("FRA < Computed [%d] descriptors for frame [id: %d] in [%.1fms]", descriptors.rows, id,timeExtract*1000.);
            }
            return descriptors;
        }

        const Eigen::MatrixXd& getBearings(){
            /// Computes unit bearing vectors projected from the optical center through the rectified (key) points on the image plane
            if (bearings.rows()==0){
                ROS_INFO("FRA > Computing bearing vectors for frame [id: %d]", id);

                ROS_INFO_STREAM(*this);

                if (points.size()>0){
                    cameraModel.bearingVectors(points, bearings);
                } else if (keypoints.size()>0){
                    cv::KeyPoint::convert(keypoints, points);
                    cameraModel.bearingVectors(points, bearings);
                }
                ROS_INFO("FRA < Computed [%lu] bearing vectors", points.size());
            }
            return bearings;

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

typedef cv::Ptr<Frame> FramePtr;

#endif // FRAME_HPP
