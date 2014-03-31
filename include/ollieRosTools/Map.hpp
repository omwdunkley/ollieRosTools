#ifndef MAP_HPP
#define MAP_HPP

#include <deque>
#include <ollieRosTools/aux.hpp>
#include <ollieRosTools/Frame.hpp>
#include <ollieRosTools/Tracker.hpp>

class OdoMap {
    private:
        // list of keyframes. Latest KF at end, oldest first
        std::deque<FramePtr> keyframes;
        FramePtr currentFrame;

        Tracker tracker;

        uint maxKFNr;

    public:
        OdoMap(){
            maxKFNr = 10;
        }

        cv::Mat getVisualImage(){
            cv::Mat img = tracker.getVisualImage();
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
