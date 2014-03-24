#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <ollieRosTools/aux.hpp>
#include <ollieRosTools/Frame.hpp>
#include <opencv2/opencv.hpp>
#include <deque>
#include <opencv2/video/tracking.hpp>


class Tracker{
    private:
    std::deque<FramePtr> keyframes;
    FramePtr prevFrame;


    int       m_maxNumberOfPoints;
    cv::Mat   m_prevImg;
    //cv::Mat   m_nextImg;
    cv::Mat   m_mask;
    Points2f  m_prevPts;
    Points2f  m_nextPts;
    KeyPoints m_prevKeypoints;
    KeyPoints m_nextKeypoints;
    cv::Mat   m_prevDescriptors;
    cv::Mat   m_nextDescriptors;
    std::vector<cv::Mat> prevPyr, nextPyr;

    std::vector<unsigned char> m_status;
    std::vector<float>         m_error;



    public:
        Tracker(){
            m_maxNumberOfPoints = 400;
        }

        void setMax(const int max){
            m_maxNumberOfPoints = max;
        }

        void setKeyframe(FramePtr frame){

        }

        cv::Mat update(FramePtr frame){
            /// First frame is a key frame
            //        if (keyframes.size()==0){
            //            keyframes.push_back(frame);
            //            return;
            //        }

            const cv::Mat& m_nextImg = frame->getImage();
            //cv::buildOpticalFlowPyramid(m_nextImg, nextPyr, cv::Size(21,21), 3, true);
            cv::Mat outputFrame = frame->getVisualImage();



            if (m_mask.rows != frame->getImage().rows || m_mask.cols != frame->getImage().cols)
                m_mask.create(frame->getImage().rows, frame->getImage().cols, CV_8UC1);



            if (m_prevPts.size() > 0) {

                //cv::calcOpticalFlowPyrLK(prevPyr, nextPyr, m_prevPts, m_nextPts, m_status, m_error, cv::Size(21,21), 3);

                cv::calcOpticalFlowPyrLK(m_prevImg, m_nextImg, m_prevPts, m_nextPts, m_status, m_error);


            }

            m_mask = cv::Scalar(255);

            Points2f trackedPts;
            for (size_t i=0; i<m_status.size(); i++){
                if (m_status[i]){
                    if (m_nextPts[i].x>19 && m_nextPts[i].y>11 && (m_nextPts[i].x<m_nextImg.cols-21) && (m_nextPts[i].y<m_nextImg.rows-11)){

                        if (cv::norm(m_prevPts[i]-m_nextPts[i]) < (30)) {
                        trackedPts.push_back(m_nextPts[i]);
                        cv::circle(m_mask, m_prevPts[i], 15, cv::Scalar(0), CV_FILLED);
                        cv::line(outputFrame, m_prevPts[i], m_nextPts[i], CV_RGB(0,250,0));
                        cv::circle(outputFrame, m_nextPts[i], 3, CV_RGB(0,250-m_error[i],0));
                        m_mask.at<uchar>(m_nextPts[i].x, m_nextPts[i].y) = 0;
                        } else {
                            cv::line(outputFrame, m_prevPts[i], m_nextPts[i], CV_RGB(255,0,0));
                        }
                    }
                }
            }

            cv::rectangle(outputFrame, cv::Rect(cv::Point(11+8,11), cv::Point(m_nextImg.cols-11-10, m_nextImg.rows-11)), CV_RGB(0,0,200));
            cv::dilate(m_mask, m_mask,cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)) );

            bool needDetectAdditionalPoints = static_cast<int>(trackedPts.size()) < m_maxNumberOfPoints;
            if (needDetectAdditionalPoints){
                m_nextKeypoints = frame->getKeypoints();
                cv::KeyPointsFilter::runByPixelsMask(m_nextKeypoints, m_mask);
                cv::drawKeypoints(outputFrame, m_nextKeypoints, outputFrame, CV_RGB(0,0,200), cv::DrawMatchesFlags::DEFAULT);

                int pointsToDetect = 100 + m_maxNumberOfPoints -  static_cast<int>(trackedPts.size());

                if (static_cast<int>(m_nextKeypoints.size()) > pointsToDetect) {
                    cv::KeyPointsFilter::retainBest(m_nextKeypoints, pointsToDetect);
                }

                ROS_INFO_STREAM("Detected additional " << m_nextKeypoints.size() << " points");

                for (size_t i=0; i<m_nextKeypoints.size(); i++) {
                    trackedPts.push_back(m_nextKeypoints[i].pt);
                    cv::circle(outputFrame, m_nextKeypoints[i].pt, 5, cv::Scalar(255,0,255), 1);
                }
            }

            m_prevPts = trackedPts;
            m_nextImg.copyTo(m_prevImg);
            //std::swap(prevPyr, nextPyr);

            return outputFrame;

        }



};

#endif // TRACKER_HPP
