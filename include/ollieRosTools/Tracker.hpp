#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <ollieRosTools/aux.hpp>
#include <ollieRosTools/Frame.hpp>
#include <opencv2/opencv.hpp>
#include <deque>


class Tracker{
    private:
    std::deque<cv::Ptr<Frame> > keyframes;

    public:
        Tracker(){        }

    void update(cv::Ptr<Frame> frame){
        /// First frame is a key frame
        if (keyframes.size()==0){
            keyframes.push_back(frame);
            frame->getKeypoints();
            return;
        }

        /// Nth Frame






    }



};

#endif // TRACKER_HPP
