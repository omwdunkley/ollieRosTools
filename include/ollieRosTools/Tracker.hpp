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
    int       m_minNumberOfPoints;
    cv::Mat   m_prevImg;
    //cv::Mat   m_nextImg;
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
            m_maxNumberOfPoints = 800;
            m_minNumberOfPoints = 400;
        }

        void setMax(const int max){
            m_maxNumberOfPoints = max;
        }
        void setMin(const int min){
            m_minNumberOfPoints = min;
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



            cv::Mat m_mask(frame->getImage().size(), CV_8U);
            m_mask = cv::Scalar(255);



            if (m_prevPts.size() > 0) {

                //cv::calcOpticalFlowPyrLK(prevPyr, nextPyr, m_prevPts, m_nextPts, m_status, m_error, cv::Size(21,21), 3);

                cv::calcOpticalFlowPyrLK(m_prevImg, m_nextImg, m_prevPts, m_nextPts, m_status, m_error);


            }

            Points2f trackedPts;
            for (size_t i=0; i<m_status.size(); i++){
                if (m_status[i]){
                    if (m_nextPts[i].x>19 && m_nextPts[i].y>11 && (m_nextPts[i].x<m_nextImg.cols-21) && (m_nextPts[i].y<m_nextImg.rows-11)){
                        if (cv::norm(m_prevPts[i]-m_nextPts[i]) < 50) {
                            trackedPts.push_back(m_nextPts[i]);
                            // Mask area to avoid further detection there. Dialation faster? Prob not.
                            cv::circle(m_mask, m_nextPts[i], 5, cv::Scalar(0), CV_FILLED);

                            cv::line(outputFrame, m_prevPts[i], m_nextPts[i], CV_RGB(0,250,0));
                            cv::circle(outputFrame, m_nextPts[i], 3, CV_RGB(0,250-m_error[i]*10,0));
                        } else {
                            cv::line(outputFrame, m_prevPts[i], m_nextPts[i], CV_RGB(255,0,0));
                            cv::circle(outputFrame, m_nextPts[i], 3, CV_RGB(255,0,0), 2);
                        }
                    } else {
                        cv::line(outputFrame, m_prevPts[i], m_nextPts[i], CV_RGB(255,0,0));
                        cv::circle(outputFrame, m_nextPts[i], 3, CV_RGB(255,0,0), 2);
                    }
                }
            }

            cv::rectangle(outputFrame, cv::Rect(cv::Point(11+8,11), cv::Point(m_nextImg.cols-11-10, m_nextImg.rows-11)), CV_RGB(0,0,200));




            bool needDetectAdditionalPoints = static_cast<int>(trackedPts.size()) < m_minNumberOfPoints;
            if (needDetectAdditionalPoints){
                m_nextKeypoints = frame->getKeypoints();

                // If we need to detect new points, dont allow any close to existing ones
                //cv::imshow("mask", m_mask); cv::waitKey(30);
                cv::KeyPointsFilter::runByPixelsMask(m_nextKeypoints, m_mask);
                //cv::KeyPointsFilter::removeDuplicated(m_nextKeypoints); //already done inside getKeypoints()

                // Draw potential new ones
                cv::drawKeypoints(outputFrame, m_nextKeypoints, outputFrame, CV_RGB(0,0,200), cv::DrawMatchesFlags::DEFAULT);

                // Compute how many points we want to add
                int pointsToDetect = m_maxNumberOfPoints - static_cast<int>(trackedPts.size());

                // Keep the ones with strongest responses
                if (static_cast<int>(m_nextKeypoints.size()) > pointsToDetect) {
                    cv::KeyPointsFilter::retainBest(m_nextKeypoints, pointsToDetect);
                }

                ROS_INFO_STREAM("Detected additional " << m_nextKeypoints.size() << " points");

                for (size_t i=0; i<m_nextKeypoints.size(); i++) {
                    trackedPts.push_back(m_nextKeypoints[i].pt);
                    cv::circle(outputFrame, m_nextKeypoints[i].pt, 5, cv::Scalar(255,0,255), 1);
                }


                // make flash around when border when redetecting
//                cv::Mat mask(frame->getImage().size(), CV_8U);
//                mask = cv::Scalar::all(255);
//                mask(cv::Rect(cv::Point(11+8,11), cv::Point(m_nextImg.cols-11-10, m_nextImg.rows-11))) = cv::Scalar::all(0);
//                cv::subtract(cv::Scalar::all(255),outputFrame, outputFrame, mask);
                cv::subtract(cv::Scalar::all(255),outputFrame, outputFrame);
                OVO::putInt(outputFrame, trackedPts.size(), cv::Point(20,50), CV_RGB(255,0,0), true, "F: ");
            } else {
                OVO::putInt(outputFrame, trackedPts.size(), cv::Point(20,50), CV_RGB(0,255,0), true, "F: ");
            }

            m_prevPts = trackedPts;
            m_nextImg.copyTo(m_prevImg);
            //std::swap(prevPyr, nextPyr);

            float ratio = (static_cast<float>(trackedPts.size())-m_minNumberOfPoints)/(m_maxNumberOfPoints-m_minNumberOfPoints);
            cv::line(outputFrame,
                     cv::Point(85,45),
                     cv::Point(85+(m_maxNumberOfPoints-m_minNumberOfPoints)* ratio, 45),
                     CV_RGB(255*(1-ratio),255*ratio,0),5);
            cv::line(outputFrame,cv::Point(85,45),cv::Point(85+m_maxNumberOfPoints-m_minNumberOfPoints,45), CV_RGB(200,0,200),1);

            return outputFrame;

        }



};

#endif // TRACKER_HPP
