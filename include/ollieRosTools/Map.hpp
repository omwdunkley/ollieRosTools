#ifndef MAP_HPP
#define MAP_HPP

#include <deque>

//#include <ollieRosTools/Frame.hpp>
#include <ollieRosTools/Tracker.hpp>
#include <ollieRosTools/aux.hpp>

// worldPoint[i] corresponds to feature[idx[i]]
cv::Mat getPointsProjectedImage(FramePtr& f, const opengv::points_t& worldPts, const Ints& idx);

bool noRef(const PointPtr& p);

class OdoPoint {
private:
    // XYZ position in world frame
    Eigen::Vector3d xyz;
    // Frame that observed this landmark
    FramePtr seenFrom;
    // Descriptor / Keypoint / image point Id. id = descriptor row of seenFrom
    int pointId;

    static int pIdCounter;
    int pId;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    OdoPoint(const FramePtr& frame, const int descId, Eigen::Vector3d& point){
        pId = ++pIdCounter;
        pointId = descId;
        seenFrom = frame;
        xyz = point;
    }

    const cv::Mat getDesc(){
        return seenFrom->getMapDescriptor(pointId);
    }


};



class OdoMap {
    private:
        // list of keyframes. Latest KF at end, oldest first
        std::deque<FramePtr> keyframes;
        std::deque<PointPtr> landmarks;

        FramePtr currentFrame;
        Tracker tracker;
        uint maxKFNr;

        // will need to be changed later
        // 3d land marks
        opengv::points_t points;
        DMatches kfMatches;



    public:
        OdoMap(){
            maxKFNr = 10;
        }


        /// Goes through all map points and remove them if they only have one reference (ie this container holding it)
        void removeNonVisiblePoints(){
            ROS_INFO("MAP > Removing non visible points");
            size_t s = landmarks.size();
            landmarks.erase( std::remove_if( landmarks.begin(), landmarks.end(), noRef), landmarks.end() );
            ROS_INFO("MAP < Removed [%lu/%lu] points. [%lu] points left", s-landmarks.size(),s, landmarks.size());
        }


        cv::Mat getVisualImage(){
            cv::Mat img = tracker.getVisualImage();
            /// THIS IS ONLY VALID RIGHT AFTER INIT
            if (points.size()>0){
                Ints qIdx,tIdx;
                OVO::match2ind(kfMatches, qIdx, tIdx);
                cv::Mat imgt = getPointsProjectedImage(getLatestKF(), points, tIdx);
                cv::Mat imgq = getPointsProjectedImage(getCurrentFrame(), points, qIdx);
                cv::Mat imgPt;
                cv::hconcat(imgq, imgt, imgPt);
                cv::addWeighted(img, 0.5, imgPt, 0.9, 0.0, img);
                cv::imshow("Map", img);
                cv::waitKey(800);
                points.clear();

            }




            return img;
        }

        void reset(){
            ROS_INFO("MAP > RESETING MAP");
            keyframes.clear();
            ROS_INFO("MAP < MAP RESET");
        }

        long unsigned int getKeyframeNr() const{
            return keyframes.size();
        }

        FramePtr& getLatestKF(){
            return keyframes.back();
        }


        /// Show the frame to the map, track against latest KF, return disparity
        float showFrame(FramePtr& frame){
            currentFrame = frame;
            const float disparity = tracker.track(currentFrame);
            return disparity;
        }


        FramePtr& getCurrentFrame(){
            return currentFrame;
        }

        const DMatches& getF2KFMatches(){
            return tracker.getF2KFMatches();
        }

        void addKeyFrame(FramePtr& frame, bool first=false){
            ROS_INFO("MAP > ADDING KF TO MAP");
            frame->setAsKF();
            if (keyframes.size()==0 || first){
                reset();
                tracker.initialise(frame);
                keyframes.push_back(frame);
                ROS_INFO("MAP > INITIAL KF ADDED");
            } else {
                /// TODO:
                // bundle adjustment
                // add more points to the frame/keyframe
                // triangulate points



                tracker.initialise(frame);
                keyframes.push_back(frame);
                ROS_INFO("MAP > KF ADDED [KFS = %lu]", getKeyframeNr());

                // Check we dont have too many keyframes
                shirnkKFs();

            }
        }

        // Assumes worldPoiints are triangulated from current frame and frame
        /// TODO
        void initialise(const opengv::points_t worldPoints, const DMatches& voMatches){
            ROS_INFO("MAP < INITIALISING MAP ");
            points = worldPoints;
            kfMatches = voMatches;
            ROS_INFO("MAP < MAP INITIALISED ");
        }

        void shirnkKFs(){
            if (keyframes.size()>maxKFNr) {
                ROS_INFO("MAP > TOO MANY KEYFRAMES, REMOVING OLDEST");
                while(keyframes.size()>maxKFNr){
                    removeKF();
                }
                ROS_INFO("MAP < CAPPED KFS");
            }
        }

        void removeKF(){
             ROS_INFO("MAP > REMOVING OLDEST KF");
             keyframes.pop_front();
             ROS_INFO("MAP < REMOVED OLDEST KF");
        }

        void setParameter(ollieRosTools::VoNode_paramsConfig &config, uint32_t level){
            ROS_INFO("MAP > SETTING PARAMS");

            maxKFNr = config.map_maxKF;
            shirnkKFs();

            tracker.setParameter(config,level);

            ROS_INFO("MAP < PARAMS SET");
        }


};

#endif // MAP_HPP
