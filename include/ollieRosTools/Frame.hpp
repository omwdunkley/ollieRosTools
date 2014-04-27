#ifndef FRAME_HPP
#define FRAME_HPP


#include <iostream>
#include <iomanip>      // std::setprecision
#include <vector>
#include <deque>
#include <tr1/unordered_map>

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
#include <ollieRosTools/Landmark.hpp> //circular dep



class Frame{

    protected:
        cv::Mat image;
        cv::Mat mask;
        cv::Mat sbi;
        Mats pyramid;
        cv::Size pyramidWindowSize;
        opengv::rotation_t imuAttitude; // IMU rotation in the world frame
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
        bool hasPoseEstimate;

        // Type of descriptor / detector used
        int descriptorId;
        int detectorId;


        // Features - ALL OF THESE SHOULD ALWAYS BE ALIGNED!
        KeyPoints keypointsImg;
        KeyPoints keypointsRotated; // TODO: should be eigen
        cv::Mat descriptors;
        //cv::Mat descriptorsCachedVo;
        Eigen::MatrixXd bearings;
        Eigen::MatrixXd pointsRect; // rectified points, align with all of the above

        //
        static cv::Ptr<CameraATAN> cameraModel;
        static cv::Ptr<Detector> detector;
        static cv::Ptr<PreProc> preproc;

        /// If a keyframe, this contains references to map points
        Landmark::IntMap landmarkRefs;

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

        virtual void computeKeypoints(){
            ROS_INFO("FRA > Computing Keypoints frame [id: %d]", getId());
            clearAllPoints();
            ros::WallTime t0 = ros::WallTime::now();
            detector->detect(image, keypointsImg, detectorId, mask);
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
        typedef cv::Ptr<Frame> Ptr;
        typedef std::deque<Frame::Ptr> Ptrs;


        Frame() : initialised(false){
            ROS_INFO("FRA = NEW UNINITIALISED FRAME");
        }
        virtual ~Frame(){            
            ROS_INFO(OVO::colorise("FRA = Destroying frame [%d|%d]",OVO::FG_DGRAY).c_str(),getId(), getKfId());
            ROS_ASSERT(landmarkRefs.size()==0); // must have deallocated all references!
            landmarkRefs.clear();
        }

        Frame(const cv::Mat& img, const tf::StampedTransform& imu, const cv::Mat& mask=cv::Mat());

        void static setCamera  (cv::Ptr<CameraATAN> cm){cameraModel=cm;}
        void static setDetector(cv::Ptr<Detector>    d){detector=d;    }
        void static setPreProc (cv::Ptr<PreProc>    pp){preproc=pp;    }


        void addLandMarkRef(const int id, const Landmark::Ptr& lm);
        void removeLandMarkRef(const int id);
        void prepareRemoval();

        void computeSBI();
        float compareSBI(Frame::Ptr f);
        const cv::Mat& getSBI();

        bool poseEstimated() const{
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









        visualization_msgs::Marker getBearingsMarker(int id=0, const std::string& name="Bearings", const std::string frame ="/world", double length=1.0, double width=1.0, const CvScalar RGB = CV_RGB(0,200,0), const Eigen::Matrix3d& rot=Eigen::Matrix3d::Identity()) const{
            visualization_msgs::Marker ms;
            ms.header.stamp = ros::Time::now();
            ms.ns = name;
            ms.id = id;
            ms.header.frame_id = frame;
            ms.type = visualization_msgs::Marker::LINE_LIST;
            ms.action = visualization_msgs::Marker::ADD;
            ms.scale.x = 0.005*width;
            ms.frame_locked = true;

            std_msgs::ColorRGBA col;
            col.a = 0.7;
            col.r = RGB.val[2]/255;
            col.g = RGB.val[1]/255;
            col.b = RGB.val[0]/255;
            ms.color = col;
            Eigen::MatrixXd bv(bearings);
            if (bv.rows()==0){
                return ms;
            }
            // Rotate. Note it should be rot * bearing' but as bearing has wrong shape we do bearing * rot'
            bv*=rot.transpose();
            for (int i=0; i< bv.rows(); ++i){
                geometry_msgs::Point p;
                tf::pointEigenToMsg(bv.row(i)*length, p);
                ms.points.push_back(p);
                ms.points.push_back(geometry_msgs::Point());
            }
            return ms;

        }

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
                setPoseRotationFromImu(/*true*/);
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
        const tf::Pose getTFPose(bool imu=false) const {
            //TODO            
            tf::Pose pose;
            tf::poseEigenToTF(getPose(imu), pose);
            return pose;
        }

        // return stamped transform with current time and pose in world frame
         tf::StampedTransform getStampedTransform(bool imu=false) const {
            if (kfId<0){
                return tf::StampedTransform(getTFPose(imu), ros::Time::now(), WORLD_FRAME, imu?"/Fimu":"/F");
            } else {
                return tf::StampedTransform(getTFPose(imu), ros::Time::now(), WORLD_FRAME, (imu?"/KFimu":"/KF")+boost::lexical_cast<std::string>(kfId));
            }
        }

        // returns the eigen pose of this frame in the world frame
        Pose getPose(bool imu=false) const {
            ROS_ASSERT(hasPoseEstimate);
            if (imu){
                return pose * IMU2CAM.inverse();
            } else {
                return pose;
            }
        }

        // gets the optical axis in the world frame
        const Eigen::Vector3d getOpticalAxisBearing() const {
            ROS_ASSERT(hasPoseEstimate);
            return pose.linear()*Eigen::Vector3d(0., 0., 1.);
        }

        // gets the optical center in the world frame
        const Eigen::Vector3d getOpticalCenter() const {
            ROS_ASSERT(hasPoseEstimate);
            return pose.translation();
        }

        // Sets the pose from a 3x4 mat, converting it to an eigen affine3d
        void setPose(Eigen::Affine3d& t){
            pose = t;
            hasPoseEstimate = true;
        }

        // sets the pose from the imu rotation. This is usually used on the very first keyframe
        virtual void setPoseRotationFromImu(/*bool inverse = false*/){
            ROS_INFO("FRA = Setting pose from IMU rotation");
            pose.setIdentity();
            pose.linear() = imuAttitude*IMU2CAM.linear();
            /*if (inverse){
                pose.linear().transposeInPlace();
            }*/
            hasPoseEstimate = true;
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
            return cameraModel->getCamInfo();
        }

        // returns the original preprocessed image the frame was initialised with. Cannot be changed
        const cv::Mat& getImage() const{
            return image;
        }

        // Creates a new image with visual information displayed on it
        cv::Mat getVisualImage() const{
            ROS_INFO("FRA > GETTING VISUAL IMAGE OF FRAME [%d|%d]", getId(), getKfId());
            cv::Mat img;

            // Draw keypoints if they exist, also makes img colour
            cv::drawKeypoints(image, keypointsImg, img, CV_RGB(0,90,20), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            OVO::putInt(img, keypointsImg.size(), cv::Point(10,1*25), CV_RGB(0,96,0),  true,"KP:");

            // Draw Frame ID
            OVO::putInt(img, id, cv::Point2f(img.cols-95,img.rows-4*25), CV_RGB(0,110,255), true,  "FID:");

            // if a kf, draw id
            if (kfId>=0){
                OVO::putInt(img, kfId, cv::Point2f(img.cols-95,img.rows-5*25), CV_RGB(0,110,255), true,  "KID:");                
                OVO::putInt(img, landmarkRefs.size(), cv::Point2f(img.cols-95,img.rows-6*25), CV_RGB(0,110,255), true,  "LMs:");
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
            ROS_INFO("FRA < Returning VISUAL IMAGE OF FRAME [%d|%d]", getId(), getKfId());
            return img;
        }

        // Gets all keypoints. Computes them if required/outdated type unless dontCompute is true. Uses points as kps if possible.
        KeyPoints& getKeypoints(bool dontCompute=false){            
            if (!dontCompute){
                if (keypointsImg.empty()){
                    computeKeypoints();
                } else if (getDetId() != detector->getDetectorId()){
                    // we have outdated keypoints - recompute
                    ROS_WARN("FRA > Keypoints outdated [%d != %d], redetecting", getDetId(), detector->getDetectorId());
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
                keypointsRotated = cameraModel->rotatePoints(keypointsImg, -getRoll(), aroundOptical);
            } else {
                ROS_WARN("FRA = Asked for rotated keypoints but dont have any keypoints to compute them from");
            }
            return keypointsRotated;
        }

        Ints getIndLM() const{
            Ints vo;
            vo.reserve(landmarkRefs.size());
            for(Landmark::IntMap::const_iterator it = landmarkRefs.begin(); it != landmarkRefs.end(); ++it) {
                vo.push_back(it->first);
            }
            return vo;
        }

        const Landmark::IntMap& getLandmarkRefs() const {
            return landmarkRefs;
        }

        uint getLandmarkRefNr() const {
            return landmarkRefs.size();
        }


        // Gets descriptors. Computes them if they are empty or the wrong type (determined from the current set extraxtor)
        const cv::Mat& getDescriptors(){
            if (descriptors.empty()) {
                // we have no descriptors
                extractDescriptors();
            } else {
                // we have descriptrs, check if they are still okay
                if (getDescriptorId() != detector->getExtractorId() ){
                    // we switched the type
                    ROS_WARN("FRA = Descriptor type changed [%d vs %d], recomputing descriptors for frame [%d]", getDescriptorId(), detector->getExtractorId(), getId());
                    extractDescriptors();
                }
            }
            return descriptors;
        }


        // Extracts descriptors, setting the frames desc type. Computes/recomputes Keypoints if required
        virtual void extractDescriptors(){
            getKeypoints(); // updates them if required
            uint kpsize = keypointsImg.size();
            ROS_INFO("FRA > Computing [%lu] descriptors for frame [id: %d]", keypointsImg.size(), id);
            ros::WallTime t0 = ros::WallTime::now();
            detector->extract(image, keypointsImg, descriptors, descriptorId, getRoll() );
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
                    cameraModel->bearingVectors(keypointsImg, bearings, pointsRect);
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

        // gets the pose of the camera ( orIMU)
        const geometry_msgs::Pose getPoseMarker(bool imu=false, bool swapxz = false) const {
            geometry_msgs::Pose pm;
            tf::Pose p = getTFPose(imu);
            if (swapxz){
                p = p*tf::Transform(tf::Quaternion(0.5, -0.5, 0.5, 0.5));
            }
            tf::poseTFToMsg(p, pm);
            return pm;
        }


        int getKfId() const {
            return kfId;
        }

        int getDescType() const{
            return detector->getDescriptorType();
        }

        int getDescSize() const{
            return detector->getDescriptorSize();
        }

        int getDescriptorId() const {
            return descriptorId;
        }

        int getDetId() const {
            return detectorId;
        }


        static void setParameter(ollieRosTools::VoNode_paramsConfig &config, uint32_t level){
            ROS_INFO("FRA > SETTING PARAMS");


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

class FrameSynthetic : public Frame {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        FrameSynthetic(){
            ROS_ASSERT(USE_SYNTHETIC);
        }

        FrameSynthetic(const cv::Mat& img,
                       const tf::StampedTransform& imu,
                       const std::vector<geometry_msgs::Point>& KPDesc,
                       const tf::Transform& poseCamGroundTruth,
                       const tf::Transform& poseImuGroundTruth,
                       const sensor_msgs::CameraInfo cam,
                       const cv::Mat& mask = cv::Mat())
            : Frame(),
              synKPDesc(KPDesc){

            tf::transformTFToEigen(poseCamGroundTruth,groundTruthCamTransform);
            tf::transformTFToEigen(poseImuGroundTruth,groundTruthImuTransform);

            initialised = true;
            id = ++idCounter;
            kfId = -1;


            ROS_INFO("FRA [SYN] = CREATING NEW FRAME [ID: %d]", id);
            time = imu.stamp_;

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
            cv::Mat imgProc = preproc->process(img);


            image = cameraModel->rectify(imgProc, cam);

            timePreprocess = (ros::WallTime::now()-t0).toSec();

            /// Deal with IMU and Pose
            // reset pose
            pose.setIdentity();
            // get imu (convert from TF)
            tf::matrixTFToEigen(imu.getBasis(), imuAttitude);


            // just for printing

            /// TODO: WHY IS THIS LIKE THIS????
            OVO::tf2RPY(imuAttitude, pitch, roll, yaw);
//            OVO::tf2RPY(imuAttitude, roll, pitch, yaw);

            /// TODO: estiamte quality of imageu using acceleromter, gyro and blurriness estiamtion
            estimateImageQuality();
            hasPoseEstimate = false;
            /*
            IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
            ROS_INFO_STREAM("---------------------\nFRA [SYN] IMU2CAM:\n" << IMU2CAM.matrix().format(CleanFmt));
            //ROS_INFO_STREAM("\nFRA [SYN] IMU:\n " << imuAttitude.format(CleanFmt));
            ROS_INFO_STREAM("\nFRA [SYN] POSE IMU:\n" << groundTruthImuTransform.matrix().format(CleanFmt));
            ROS_INFO_STREAM("\nFRA [SYN] POSE CAM:\n" << groundTruthCamTransform.matrix().format(CleanFmt) <<"\n---------------------\n");
            */
            ROS_INFO("FRA [SYN] < GroundTruth Injected into Frame [ID: %d]", id);
        }

        const Pose& getGroundTruthCamPose() const {
            return groundTruthCamTransform;
        }
        const Pose& getGroundTruthIMUPose() const {
            return groundTruthImuTransform;
        }

        void extractDescriptors(){
            getKeypoints(); // updates them if required
            uint kpsize = keypointsImg.size();
            ROS_INFO("FRA [SYN] > Computing [%lu] descriptors for frame [id: %d]", keypointsImg.size(), id);
            ros::WallTime t0 = ros::WallTime::now();
            detector->extract(keypointsImg, descriptors, descriptorId, getRoll() );
            // detector might actually remove/add keypoints. IN this case it is important to realign existing data
            if (kpsize!=0 && kpsize != keypointsImg.size()){
                ROS_INFO("FRA [SYN] = Detecting changed kp size, resetting data aligned with these");
                cv::Mat d;
                KeyPoints kps;
                std::swap(keypointsImg, kps);
                std::swap(descriptors, d);
                clearAllPoints();
                std::swap(keypointsImg, kps);
                std::swap(descriptors, d);
            }
            timeExtract = (ros::WallTime::now()-t0).toSec();
            ROS_INFO("FRA [SYN] < Computed [%d] descriptors for frame [id: %d] in [%.1fms]", descriptors.rows, id,timeExtract*1000.);
        }

        void computeKeypoints(){
            ROS_INFO("FRA [SYN] > Computing Synthetic Keypoints frame [id: %d]", getId());
            clearAllPoints();
            ros::WallTime t0 = ros::WallTime::now();
            detector->detect(synKPDesc, image, keypointsImg, detectorId, mask);
            timeDetect = (ros::WallTime::now()-t0).toSec();
            ROS_INFO("FRA [SYN] < Computed [%lu] Keypoints of [Type: %d] for frame [id: %d] in  [%.1fms] ", keypointsImg.size(), getDetId(), getId(),timeDetect*1000.);
        }

        // sets the pose from the imu rotation. This is usually used on the very first keyframe
        void setPoseRotationFromImu(/*bool inverse = false*/){
            ROS_WARN("FRA [SYN] = Setting pose from Ground Truth instead of IMU");
            pose.setIdentity();
            //pose.linear() = imuAttitude*IMU2CAM.linear();
            pose = groundTruthImuTransform*IMU2CAM;

            hasPoseEstimate = true;
        }

        virtual ~FrameSynthetic(){
            ROS_INFO(OVO::colorise("FRA = Destroying Synthetic frame [%d|%d]",OVO::FG_DGRAY).c_str(),getId(), getKfId());
        }


    private:
        Pose groundTruthCamTransform;
        Pose groundTruthImuTransform;
        const std::vector<geometry_msgs::Point> synKPDesc; //x,y = kp, z = desc


};


// useful for std::map, find, etc
inline bool operator==(const Frame::Ptr& lhs, const int& fid){return lhs->getId() == fid;}
inline bool operator==(const int& fid, const Frame::Ptr& rhs){return rhs->getId() == fid;}
inline bool operator==(const Frame::Ptr& lhs, const Frame::Ptr& rhs){return lhs->getId() == rhs->getId();}
inline bool operator!=(const Frame::Ptr& lhs, const Frame::Ptr& rhs){return !operator==(lhs,rhs);}
inline bool operator< (const Frame::Ptr& lhs, const Frame::Ptr& rhs){return lhs->getId() < rhs->getId();}
inline bool operator> (const Frame::Ptr& lhs, const Frame::Ptr& rhs){return  operator< (rhs,lhs);}
inline bool operator<=(const Frame::Ptr& lhs, const Frame::Ptr& rhs){return !operator> (lhs,rhs);}
inline bool operator>=(const Frame::Ptr& lhs, const Frame::Ptr& rhs){return !operator< (lhs,rhs);}


#endif // FRAME_HPP
