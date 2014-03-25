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
        FramePtr keyframe;
        FramePtr prevFrame;

        // Matches from current frame to keyframe;
        DMatches prevMatches;

        // Max nr of keypoints we want at any time
        int       maxPts;

        // Minimum number of keypoints desired. If nr falls below this threshold, redetect
        int       minPts;


        // Used by KLT
        cv::Point klt_window;
        int klt_levels;
        cv::TermCriteria klt_criteria;
        int klt_flags;
        double klt_eigenThresh;

    public:
        Tracker(){
            maxPts = 1200;
            minPts = 600;
            klt_window = cv::Size(31,31);
            klt_criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
            klt_levels = 3;
            klt_flags = 0; // cv::OPTFLOW_LK_GET_MIN_EIGENVALS
            klt_eigenThresh=0.0001;
        }


        void setKeyframe(FramePtr frame){
            ROS_INFO("ADDING KEYFRAME");
            frame->setAsKF();
            keyframes.push_back(frame);
            keyframe = frame;
        }




        /// TODO: Detect bad frames, dont update previous values for next step
        ///       Keyframe update should check matches from KLT. If flow is too big KLT returns mostly false matches
        cv::Mat update(FramePtr frame, bool makeKeyframe=false){
            /// First frame is a key frame

            const cv::Rect border = cv::Rect(cv::Point(8,0)+(klt_window*0.5), cv::Point(frame->getImage().cols-10, frame->getImage().rows)-(klt_window*0.5));

            if (makeKeyframe || keyframes.size()==0){
                setKeyframe(frame);

                // Compute initial keypoints
                const KeyPoints& kf_keypoints = keyframe->getKeypoints();

                /// build initial 1:1 matching
                prevMatches.clear();
                prevMatches.reserve(kf_keypoints.size());
                for (size_t i=0; i<kf_keypoints.size(); i++) {
                    prevMatches.push_back(cv::DMatch(i,i,0.f));
                }

                /// buffer frame for next update
                prevFrame = keyframe;


                ROS_INFO_STREAM("Detected initial " << kf_keypoints.size() << " key points");


                /// Draw stuff
                cv::Mat outputImage = keyframe->getVisualImage();
                cv::Mat mask(keyframe->getImage().size(), CV_8U);

                mask = cv::Scalar::all(255);
                mask(border) = cv::Scalar::all(0);
                cv::subtract(cv::Scalar::all(255),outputImage, outputImage, mask);
                cv::rectangle(outputImage, border, CV_RGB(0,0,200));
                cv::drawKeypoints(outputImage, kf_keypoints, outputImage, CV_RGB(0,0,200), cv::DrawMatchesFlags::DEFAULT);
                OVO::putInt(outputImage, kf_keypoints.size(), cv::Point(20,50), CV_RGB(255,0,0), true, "F: ");

                cv::Mat matchImage;
                cv::drawMatches(outputImage, kf_keypoints, keyframe->getVisualImage(), kf_keypoints, prevMatches, matchImage, CV_RGB(0,0,140), CV_RGB(100,0,100));
                for (size_t i=0; i<kf_keypoints.size(); i++) {
                    cv::circle(matchImage, kf_keypoints[i].pt, 5, cv::Scalar(255,0,255), 1, CV_AA);
                }

                return matchImage;
            }


            /// Matching against keyframe via previous frames
            cv::Mat outputImage = frame->getVisualImage();


            /// Mask to filter out existing point locations
            //cv::Mat klt_mask(frame->getImage().size(), CV_8U);
            //klt_mask = cv::Scalar(255);


            DMatches newMatches;
            Points2f trackedPts;


            /// TODO: detect bad frames (eg nr of detected points halfed). if frame is bad, skip and dont update prev_kpts, prev_image
            ///       frames should store their image pyramids for reuse
            ///

            if (prevFrame->getPoints().size() > 0) {
                Points2f  klt_predictedPts;
                std::vector<unsigned char> klt_status;
                std::vector<float>         klt_error;
                /// Compute matches and new positions
                cv::calcOpticalFlowPyrLK(prevFrame->getPyramid(klt_window, klt_levels),  //todo: false or true??
                                         frame->getPyramid(klt_window, klt_levels),
                                         prevFrame->getPoints(),
                                         klt_predictedPts,
                                         klt_status, klt_error, klt_window, klt_levels,
                                         klt_criteria, klt_flags, klt_eigenThresh);

                newMatches.reserve(klt_status.size());
                trackedPts.reserve(klt_status.size());
                const Points2f& prevPts = prevFrame->getPoints();
                int counter = 0;
                for (size_t i=0; i<klt_status.size(); i++){
                    if (klt_status[i]){
                        // Kill points at image border
                        if (klt_predictedPts[i].x>border.tl().x && klt_predictedPts[i].y>border.tl().y && (klt_predictedPts[i].x<border.br().x) && (klt_predictedPts[i].y<border.br().y)){
                            // Kill points that are too far away from previous one
                            float norm = cv::norm(prevPts[i]-klt_predictedPts[i]);
                            if (norm< 100) {
                                trackedPts.push_back(klt_predictedPts[i]);
                                newMatches.push_back(cv::DMatch(counter++, prevMatches[i].trainIdx, klt_error[i] /*norm*/));

                                /// Mask area to avoid further detection there. Dialation faster? Prob not.
                                //cv::circle(klt_mask, klt_nextPts[i], 5, cv::Scalar(0), CV_FILLED, CV_AA);

                                cv::line(outputImage, prevPts[i], klt_predictedPts[i], CV_RGB(0,250,0),1,CV_AA);
                                cv::circle(outputImage, klt_predictedPts[i], 3, CV_RGB(0,250-klt_error[i]*10,0), 1, CV_AA);
                            } else {
                                cv::line(outputImage, prevPts[i], klt_predictedPts[i], CV_RGB(255,0,50),1, CV_AA);
                                cv::circle(outputImage, klt_predictedPts[i], 3, CV_RGB(255,0,50), 2, CV_AA);
                            }
                        } else {
                            cv::line(outputImage, prevPts[i], klt_predictedPts[i], CV_RGB(255,0,0),1 , CV_AA);
                            cv::circle(outputImage, klt_predictedPts[i], 3, CV_RGB(255,0,0), 2, 1, CV_AA);
                        }
                    } else {
                        cv::line(outputImage, prevPts[i], klt_predictedPts[i], CV_RGB(255,50,0),1 , CV_AA);
                        cv::circle(outputImage, klt_predictedPts[i], 3, CV_RGB(200,0,100), 2, 1, CV_AA);
                    }
                }
            }

            cv::rectangle(outputImage, border, CV_RGB(0,0,200));




//            bool needDetectAdditionalPoints = static_cast<int>(trackedPts.size()) < m_minNumberOfPoints;
//            if (needDetectAdditionalPoints){
//                KeyPoints m_nextKeypoints = frame->getKeypoints();

//                // If we need to detect new points, dont allow any close to existing ones
//                //cv::imshow("mask", m_mask); cv::waitKey(30);
//                cv::KeyPointsFilter::runByPixelsMask(m_nextKeypoints, klt_mask);
//                //cv::KeyPointsFilter::removeDuplicated(m_nextKeypoints); //already done inside getKeypoints()

//                // Draw potential new ones
//                cv::drawKeypoints(outputImage, m_nextKeypoints, outputImage, CV_RGB(0,0,200), cv::DrawMatchesFlags::DEFAULT);

//                // Compute how many points we want to add
//                int pointsToDetect = m_maxNumberOfPoints - static_cast<int>(trackedPts.size());

//                // Keep the ones with strongest responses
//                if (static_cast<int>(m_nextKeypoints.size()) > pointsToDetect) {
//                    cv::KeyPointsFilter::retainBest(m_nextKeypoints, pointsToDetect);
//                }

//                ROS_INFO_STREAM("Detected additional " << m_nextKeypoints.size() << " points");

//                for (size_t i=0; i<m_nextKeypoints.size(); i++) {
//                    trackedPts.push_back(m_nextKeypoints[i].pt);
//                    cv::circle(outputImage, m_nextKeypoints[i].pt, 5, cv::Scalar(255,0,255), 1, CV_AA);
//                }


//                // make flash around when border when redetecting
//                cv::Mat mask(frame->getImage().size(), CV_8U);
//                mask = cv::Scalar::all(255);
//                mask(cv::Rect(cv::Point(11+8,11), cv::Point(klt_nextImg.cols-11-10, klt_nextImg.rows-11))) = cv::Scalar::all(0);
//                cv::subtract(cv::Scalar::all(255),outputImage, outputImage, mask);
//                //cv::subtract(cv::Scalar::all(255),outputFrame, outputFrame);
//                OVO::putInt(outputImage, trackedPts.size(), cv::Point(20,50), CV_RGB(255,0,0), true, "F: ");
//            } else {
            OVO::putInt(outputImage, trackedPts.size(), cv::Point(20,50), CV_RGB(0,255,0), true, "F: ");
            //}




            /// set points in current frame
            frame->swapPoints(trackedPts);

            /// Drawing
            cv::Mat matchImage;
            if(frame->getPoints().size()>0){
                // we have matches to draw
                cv::drawMatches(outputImage, frame->getKeypoints(), keyframe->getVisualImage(), keyframe->getKeypoints(), newMatches, matchImage, CV_RGB(0,0,100), CV_RGB(150,0,100),std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
            } else {
                // we dont have matches to draw and dont want to invoke getKeypoints if we have none (otherwise it will detect)
                cv::drawMatches(outputImage, KeyPoints(), keyframe->getVisualImage(), KeyPoints(), DMatches(), matchImage);

            }

            cv::Point l(85,45);
            cv::Point r(outputImage.cols-85,45);
            int length = r.x-l.x;
            float ratio = (static_cast<float>(frame->getPoints().size())-minPts)/(maxPts-minPts);
            cv::line(matchImage, l, l+cv::Point(length*frame->getPoints().size()/maxPts, 0), CV_RGB(255*(1-ratio),255*ratio,0), 3);
            cv::line(matchImage, l+cv::Point(length*minPts/maxPts, -5), l+cv::Point(length*minPts/maxPts, 5), CV_RGB(255,0,0), 1);
            cv::line(matchImage, l, r, CV_RGB(0,0,200), 1);
            /// buffer values for next iteration


            std::swap(prevMatches, newMatches);
            prevFrame = frame;
            return matchImage;

        }


        void setParameter(ollieRosTools::VoNode_paramsConfig &config, uint32_t level){
            maxPts          = config.kp_max;
            minPts          = config.kp_min;
            klt_window      = cv::Size(config.klt_window*2+1, config.klt_window*2+1);
            klt_criteria    = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, config.klt_iter, config.klt_eps);
            klt_levels      = config.klt_levels;
            klt_flags       = (config.tracker==1||config.tracker==3) ? (cv::OPTFLOW_LK_GET_MIN_EIGENVALS) : 0; //
            klt_eigenThresh = config.klt_eigenThresh;
            ROS_INFO("EVF: %d EVT: %f", klt_flags, klt_eigenThresh);
        }



};








/* // OPTICAL FLOW
//                cv::Mat flow;
//                cv::calcOpticalFlowFarneback(klt_prevImg, klt_nextImg, flow, 0.5, 3, 31, 3, 5, 1.1, 0);
//                std::cout << flow.size() << std::endl;
//                cv::Mat xy[2]; //X,Y
//                cv::split(flow, xy);
//                //calculate angle and magnitude
//                cv::Mat magnitude, angle;
//                cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);
//                //translate magnitude to range [0;1]
//                double mag_max=60;
//                //cv::minMaxLoc(magnitude, 0, &mag_max);
//                magnitude.convertTo(magnitude, -1, 1.0/mag_max);
//                //build hsv image
//                cv::Mat _hls[3], hsv;
//                _hls[0] = angle;
//                _hls[1] = cv::Mat::ones(angle.size(), CV_32F);
//                _hls[2] = magnitude;
//                cv::merge(_hls, 3, hsv);
//                //convert to BGR and show
//                cv::Mat bgr;//CV_32FC3 matrix
//                cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
//                cv::imshow("optical flow", bgr); cv::waitKey(15);
*/

#endif // TRACKER_HPP
