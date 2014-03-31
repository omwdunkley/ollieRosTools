#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <ollieRosTools/aux.hpp>
#include <ollieRosTools/Frame.hpp>
#include <opencv2/opencv.hpp>
#include <deque>
#include <opencv2/video/tracking.hpp>
#include <ollieRosTools/Matcher.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>


class Tracker{
    private:
        enum MethodType { F2KF, // match against KF every time [F -> KF]
                          KLT,  // use KLT to match frame to frame [F -> ... -> KF]
                          F2F };// match frame to frame [F -> ... -> KF]

        /// MEMBERS
        Matcher matcher;
        FramePtr keyFrame;
        FramePtr currFrame;
        FramePtr previousFrame;

        // Matches currFrame vs keyframe
        DMatches KFMatches;
        // Matches currFrame vs prevframe
        DMatches  FMatches;


        // time used for matching / KLTing
        double timeTrack;

        // rotated disparity between keyframe and current frame
        double disparity;


        // For drawing
        Points2f failedCF; // failed current frame
        Points2f failedKF; // failed keyframe
        Points2f failedPF; // failed previous frame
        // Counters for statistics
        uint counter; // successful KLT points
        Ints cBorder; // failed at border
        Ints cStepDist; // failed for step distance
        Ints cDist; // failed for distance
        Ints cKLT; // klt failed to track
        cv::Rect border; //area that we KLT on
        cv::Point borderF2KF; // tl/br point we dont detect on




        /// SETTINGS

        // maximum disparity allowed between matching points between current frame and keyframe
        int m_pxdist;
        // maximum disparity allowed between matching points for stepwise matching (between current frame and previous frame)
        int m_pxdistStep;
        // Max nr of keypoints we want at any time
        int maxPts;
        // Minimum number of keypoints desired. If nr falls below this threshold, redetect
        int minPts;
        // Tracking Method
        MethodType method;
        // when using KLT or F2F, allow intermediate F2KF lookups to add points
        bool f2kfRecovery;
        // when using KLT or F2F, assume that if the ratio of lost matches > failRatio means the frame is corrupt
        float failRatio;
        // Previous config just in case
        ollieRosTools::VoNode_paramsConfig preConfig;

        // Used by KLT specifically
        cv::Point klt_window;
        int klt_levels;
        cv::TermCriteria klt_criteria;
        int klt_flags;
        double klt_eigenThresh;
        bool klt_init; //flag if the klt tracker is intialised wrt the keyframe

        // vo stuff
        //bool voDoIntitialise;

    public:
        Tracker(){
            maxPts = 1400;
            minPts = 400;
            klt_window = cv::Size(15*2+1,15*2+1);
            klt_criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
            klt_levels = 3;
            klt_flags = 0; // cv::OPTFLOW_LK_GET_MIN_EIGENVALS
            klt_eigenThresh=0.0001;
            method = F2KF;
            f2kfRecovery = false;
            failRatio = 0.0;
            //voDoIntitialise = false;
            m_pxdist = 300;
            m_pxdistStep = 50;
            timeTrack = 0;
            disparity = 0;
            //pubMarkers = n.advertise<visualization_msgs::Marker>("/vo/markers", 1);
        }

        float rotatedDisparity(FramePtr& f1, FramePtr& f2, const DMatches& ms){
            const uint size = ms.size();
            if (size==0){
                return 0.f;
            }
            const KeyPoints& p1 = f1->getRotatedKeypoints();
            const KeyPoints& p2 = f2->getRotatedKeypoints();
            float s = 0;
            for (uint i=0; i<ms.size(); ++i){
                s+=cv::norm( p1[ms[i].queryIdx].pt - p2[ms[i].trainIdx].pt );
            }
            return s/size;
        }

        void initialise(FramePtr& frame){
            ROS_INFO("FTR > INITIALISING NEW KEYFRAME");
            keyFrame=frame;
            KFMatches.clear();
            FMatches.clear();
            failedCF.clear();
            failedKF.clear();
            failedPF.clear();
            previousFrame = frame;
            currFrame = frame;

            klt_init = false;
            ROS_INFO("FTR < INITIALISED");
        }


        DMatches& getF2KFMatches(){
            return KFMatches;
        }

        DMatches& getF2FMatches(){
            return FMatches;
        }



        /// TODO: Detect bad frames, dont update previous values for next step
        ///       Keyframe update should check matches from KLT. If flow is too big KLT returns mostly false matches
        float track(FramePtr& frame){
            ROS_INFO("FTR > TRACKING");

            ROS_ASSERT_MSG(!keyFrame.empty(), "FTR = Keyframe should not be empty" );


            /// DO TRACKING
            switch(method){
                case KLT: // track against previous frame using klt
                    doKLT(frame);
                    break;
                case F2F: // track against previous frame
                    //doF2F(FramePtr& frame, DMatches& matches, float& disparity, double& time, bool isKeyFrame=false)                    
                    ROS_ERROR("FTR = Not implemented F2F tracking");
                    doF2F(frame);
                    break;
                case F2KF: // always tracking against keyframe
                    doF2KF(frame);
                    break;
                default: // Dont do anything                    
                    currFrame = frame;
                    ROS_ERROR("FTR = Unknown Tracker Type <%d>", method);
                    break;
            }



/*
            //OVO::putInt(outputImage, 1000.*time, cv::Point(20,outputImage.rows-3*25), CV_RGB(200,0,200), false, "M:");


            /// DO INITIALISATION
//            DMatches matches;
//            Ints inliers;
//            bool init = false;
//            if (!makeKeyframe && voDoIntitialise){

//                voDoIntitialise = false;
//                if (kfMatches.size()>10){
//                    matches = kfMatches; //just a keep a copy to draw
//                    init = odometry.initialise(frame, keyframe, kfMatches, worldPoints, inliers);
//                    if (init){
//                        publishMarkers(frame, keyframe, kfMatches, worldPoints);
//                    }

//                } else {
//                    ROS_WARN("Not enough matches to initialise with");
//                }

//            }

//            /// DONE INITIALISATION
//            if (init){
//                outputImage = getFlowImage(outputImage, frame, keyframe, matches, worldMatches, worldPoints );
//                // cleaning up after init
////                Points2f points;
////                switch(method){
////                    case KLT: // track against previous frame using klt
////                        /// if doing KLT we need to remove the unused keypoints from the frame that is being cached
////                        OVO::vecReduceInd<Points2f>(frame->getPoints(), points, inliers);
////                        frame->swapPoints(points);
////                        break;
////                    case F2F:
////                        break;
////                    case F2KF:
////                        break;
////                    default:
////                        //ROS_ERROR("Unknown Tracker Type <%d>", method);
////                        break;
////                }

//            }  else {

//                if (odometry.initialised()){
//                    /// we are initialised
//                    outputImage = getFlowImage(outputImage, frame, keyframe, kfMatches, worldMatches, worldPoints);

//                } else {
//                    /// NOT DONE ANYTHING
//                    outputImage = getFlowImage(outputImage, frame, keyframe, kfMatches, DMatches(), opengv::points_t());
//                }
//            }

//            return outputImage;
            */

            disparity = rotatedDisparity(currFrame, keyFrame, KFMatches);
            ROS_INFO("FTR < TRACKED [Disparity: %.1f]", disparity);
            return disparity;
        }



/*      //Flow image
        cv::Mat getFlowImage(cv::Mat& flowImg, FramePtr& f, FramePtr& kf, const DMatches& msTrack, const DMatches& msVo, const opengv::points_t& worldPts) const{
            if (flowImg.empty()) {
                cv::drawMatches(f->getVisualImage(), KeyPoints(), kf->getVisualImage(), KeyPoints(), DMatches(), flowImg);
            }
            const cv::Point2f widthPt = cv::Point2f(f->getImage().cols,0);

            cv::Mat flowImgCopy = cv::Mat::zeros(flowImg.size(), CV_8UC3);



            const Points2f& fpts  = f->getPoints();
            const Points2f& kfpts = kf->getPoints();

            // draw feature matches
            for (uint i=0; i<msTrack.size(); ++i){
                const cv::Point2f& fpt = fpts[msTrack[i].queryIdx];
                const cv::Point2f& kfpt = kfpts[msTrack[i].trainIdx];
                cv::line(flowImgCopy, fpt, kfpt, CV_RGB(0,0,80),1,CV_AA);
            }

            if (worldPts.size()>0){
                // put the world points in frame of f


                opengv::points_t pts3d = worldPts; ///why copy??
                OVO::transformPoints(kf->getPose().inverse(), pts3d); /// TODO: should be done twice, one for F and once for KF
                Eigen::VectorXd dists(msVo.size());
                for (uint i=0; i<msVo.size(); ++i){
                    dists[i] = pts3d[i].norm();
                }

                float range_max = dists.maxCoeff();
                float range_min = dists.minCoeff();
                //float range_mean = dists.mean();
                float diff_intensity = range_max - range_min;
                if( diff_intensity == 0 ){
                    diff_intensity = 1e20;
                }

                // draw VO matches
                for (uint i=0; i<msVo.size(); ++i){
                    const cv::Point2f& fpt = fpts[msVo[i].queryIdx];
                    const cv::Point2f& kfpt = kfpts[msVo[i].trainIdx];
                    cv::line(flowImgCopy, fpt, kfpt, CV_RGB(0,0,255),1,CV_AA);
                }

                // draw colour coded depth points
                for (uint i=0; i<msVo.size(); ++i){
                    const cv::Point2f& fpt = fpts[msVo[i].queryIdx];
                    const cv::Point2f& kfpt = kfpts[msVo[i].trainIdx];
                    float depth = dists[i];
                    if (depth > 0.01){
                        float value = 1.0 - (depth - range_min)/diff_intensity;

                        value = std::min(value, 1.0f);
                        value = std::max(value, 0.0f);
                        float h = value * 5.0f + 1.0f;
                        int i = floor(h);
                        float f = h - i;
                        if ( !(i&1) ) f = 1 - f; // if i is even
                        float n = 1 - f;
                        cv::Vec3f color;
                        if      (i <= 1) color[0] = n, color[1] = 0, color[2] = 1;
                        else if (i == 2) color[0] = 0, color[1] = n, color[2] = 1;
                        else if (i == 3) color[0] = 0, color[1] = 1, color[2] = n;
                        else if (i == 4) color[0] = n, color[1] = 1, color[2] = 0;
                        else if (i >= 5) color[0] = 1, color[1] = n, color[2] = 0;
                        color *= 255;
                        cv::circle(flowImgCopy, fpt, 4, CV_RGB(color[0], color[1], color[2]), CV_FILLED, CV_AA);
                        cv::circle(flowImgCopy, kfpt+widthPt, 4, CV_RGB(color[0], color[1], color[2]), CV_FILLED, CV_AA);
                    } else {
                        cv::circle(flowImgCopy, fpt, 1, CV_RGB(0,0,0), CV_FILLED, CV_AA);
                        cv::circle(flowImgCopy, kfpt+widthPt, 1, CV_RGB(0,0,0), CV_FILLED, CV_AA);
                    }

                }
            }
            cv::addWeighted(flowImg, 0.6, flowImgCopy, 0.6, 0, flowImg);


            return flowImg;
        }

*/



/*
        void getFlowImage(cv::Mat& flowImg, FramePtr& f, FramePtr& kf, DMatches& ms, float disparity, bool rot=false, bool vo=false) const{
            // Draw an image showing the flow only
            if (flowImg.empty()){
                flowImg = cv::Mat::zeros(f->getImage().size(), CV_8UC3);
            }

            const Points2f& kpsF  = !rot ? f  ->getPoints() : f  ->getRotatedPoints();
            const Points2f& kpsKF = !rot ? kf ->getPoints() : kf ->getRotatedPoints();

            const float maxDisp = matcher.getMaxDisp();
            const float matchMaxDisp = maxDisp; /// TODO: should be may allowed vo_disparity (eg trigger new KF)


            const cv::Size& size= flowImg.size();
            if (vo){
                for (uint m = 0; m < ms.size(); ++m){
                    const cv::Point2f& p1 = kpsF [ms[m].queryIdx];
                    const cv::Point2f& p2 = kpsKF[ms[m].trainIdx];
                    const int colD =  250 - (ms[m].distance/maxDisp* 200);
                    cv::line(flowImg, p1, p2, CV_RGB(0,0,255),1,CV_AA);
                    cv::circle(flowImg, p1,2,CV_RGB(255-colD,colD,0),1,CV_AA);
                    cv::circle(flowImg, p1,3,CV_RGB(255-colD,colD,0),1,CV_AA);
                    cv::circle(flowImg, p2,2,CV_RGB(0,255,255),1,CV_AA);
                    cv::circle(flowImg, p1,1,CV_RGB(0,255,255),1,CV_AA);
                    cv::circle(flowImg, p2,1,CV_RGB(0,255,255),1,CV_AA);
                }
            } else {
                for (uint m = 0; m < ms.size(); ++m){
                    const cv::Point2f& p1 = kpsF [ms[m].queryIdx];
                    const cv::Point2f& p2 = kpsKF[ms[m].trainIdx];
                    const int colD =  250 - (ms[m].distance/maxDisp* 200);
                    const int colM =  250 - (ms[m].distance/matchMaxDisp* 200); //should be max vo_disparity
                    cv::line(flowImg, p1, p2, CV_RGB(255-colM,colM,0),1,CV_AA);
                    cv::circle(flowImg, p1,2,CV_RGB(255-colD,colD,0),1,CV_AA);
                    cv::circle(flowImg, p1,3,CV_RGB(255-colD,colD,0),1,CV_AA);
                    cv::circle(flowImg, p2,2,CV_RGB(0,255,255),1,CV_AA);
                }
            }
            cv::circle(flowImg, cv::Point2f(size.width/2, size.height/2), std::max(1.f,maxDisp), CV_RGB(0,0,255),1,CV_AA);
            cv::circle(flowImg, cv::Point2f(size.width/2, size.height/2), std::max(1.f,matchMaxDisp), CV_RGB(255,0,255),1,CV_AA);


            const int col =  250 - (disparity/maxDisp* 250);
            cv::circle(flowImg, cv::Point2f(size.width/2, size.height/2), std::max(1.f, disparity), CV_RGB(255-col,col,0),2,CV_AA);

            OVO::putInt(flowImg, disparity/maxDisp*100, cv::Point(10,1*25), CV_RGB(255,255,255), true, "Goal Perc :","%");
            OVO::putInt(flowImg, disparity, cv::Point(10,2*25), CV_RGB(255,255,255), true,       "Disparity : ","px");
            OVO::putInt(flowImg, maxDisp, cv::Point(10,3*25), CV_RGB(255,255,255), true,    "Max Disp  :","px");
            OVO::putInt(flowImg, matchMaxDisp, cv::Point(10,4*25), CV_RGB(255,255,255), true,    "Max Match :","px");



        }
*/


        void initKLT(){
            ROS_INFO("KLT > INITIALISING");

            // Compute initial keypoints
            const KeyPoints& kf_keypoints = keyFrame->getKeypoints();

            // start counting time after detection
            ros::WallTime t0 = ros::WallTime::now();

            currFrame = keyFrame;
            previousFrame = currFrame;

            /// build initial 1:1 matching
            KFMatches.clear();
            KFMatches.reserve(kf_keypoints.size());
            for (size_t i=0; i<kf_keypoints.size(); i++) {
                KFMatches.push_back(cv::DMatch(i,i,0.f));
            }

            /// buffer frame for next update
            timeTrack = (ros::WallTime::now()-t0).toSec();

            klt_init = true;
            ROS_INFO("KLT < INITIALISED WITH %lu POINTS", kf_keypoints.size());
            return;
        }

        // puts matches into class member prevMatches
        /// TODO: right now this returns the wrong disparity! We need the f->kf disparity, not the f->f disparity
        void doKLT(FramePtr& newFrame){
            ROS_INFO("KLT > TRACKING");
            ros::WallTime t0 = ros::WallTime::now();

            border = cv::Rect(cv::Point(8,0)+(klt_window*0.5), cv::Point(newFrame->getImage().cols-10, newFrame->getImage().rows)-(klt_window*0.5));

            // Counters for statistics
            cBorder.clear();
            cStepDist.clear();
            cDist.clear();
            cKLT.clear();
            // Points for visualision
            failedCF.clear();
            failedPF.clear();
            failedKF.clear();



            if (!klt_init){
                initKLT(); // keyframe must be set here
                // return
            }


            /// Matching against keyframe via previous frames
            //cv::Mat outputImage;
            //cv::drawMatches(frame->getVisualImage(), KeyPoints(), keyframe->getVisualImage(), KeyPoints(), DMatches(), outputImage);
            //const int width = frame->getVisualImage().cols;
            //const cv::Point2f widthPt = cv::Point2f(width,0);


            /// Mask to filter out existing point locations
            //cv::Mat klt_mask(frame->getImage().size(), CV_8U);
            //klt_mask = cv::Scalar(255);



            Points2f trackedPts; // new points that will be estiamted by KLT and put into the newFrame
            DMatches newKFMatches; // current frame to key frame
            DMatches newFMatches;  // current frame to previous frame


            /// TODO: detect bad frames (eg nr of detected points halfed). if frame is bad, skip and dont update prev_kpts, prev_image
            ///       frames should store their image pyramids for reuse
            ///

            if (currFrame->getPoints(true).size() > 0) {
                Points2f  klt_predictedPts;
                std::vector<unsigned char> klt_status;
                std::vector<float>         klt_error;

                /// Compute matches and new positions
                cv::calcOpticalFlowPyrLK(currFrame->getPyramid(klt_window, klt_levels),                      // input
                                         newFrame->getPyramid(klt_window, klt_levels),                       // input
                                         currFrame->getPoints(),                                             // input
                                         klt_predictedPts, klt_status, klt_error,                            // output row
                                         klt_window, klt_levels,  klt_criteria, klt_flags, klt_eigenThresh); // settings

                newKFMatches.reserve(klt_status.size());
                trackedPts.reserve(klt_status.size());
                newFMatches.reserve(klt_status.size());

                const Points2f& prevPts = currFrame->getPoints();
                const Points2f& kfPts   = keyFrame->getPoints();

                counter = 0;
                uint failCounter = -1; // so we can ++failCounter at start at 0
                for (size_t i=0; i<klt_status.size(); i++){
                    if (klt_status[i]){
                        // Kill points at image border
                        if (klt_predictedPts[i].x>border.tl().x && klt_predictedPts[i].y>border.tl().y && (klt_predictedPts[i].x<border.br().x) && (klt_predictedPts[i].y<border.br().y)){
                            // Kill points that are too far away from previous one
                            float normStep = cv::norm(prevPts[i]-klt_predictedPts[i]);
                            if (normStep < m_pxdistStep || m_pxdistStep<=0){

                                /// TODO should be rotated norm
                                float norm = cv::norm(kfPts[KFMatches[i].trainIdx]-klt_predictedPts[i]);

                                if (norm< m_pxdist || m_pxdist<=0) {
                                    // Points okay
                                    trackedPts.push_back(klt_predictedPts[i]);
                                    newKFMatches.push_back(cv::DMatch(counter, KFMatches[i].trainIdx, norm /*klt_error[i]*/));
                                    newFMatches.push_back(cv::DMatch(counter,  i, normStep /*klt_error[i]*/));
                                    ++counter;
                                    //newMatches.push_back(cv::DMatch(kfMatches[i].queryIdx, kfMatches[i].trainIdx, norm /*klt_error[i]*/));
                                    /// Mask area to avoid further detection there. Dialation faster? Prob not.
                                    //cv::circle(klt_mask, klt_nextPts[i], 5, cv::Scalar(0), CV_FILLED, CV_AA);

                                    //                                cv::line(outputImage, prevPts[i], klt_predictedPts[i], CV_RGB(0,250,0),1,CV_AA); //flow
                                    //                                cv::line(outputImage, klt_predictedPts[i], kfPts[kfMatches[i].trainIdx], CV_RGB(0,100,0),1,CV_AA);
                                    //                                cv::circle(outputImage, klt_predictedPts[i], 2, CV_RGB(0,200,0), 1, CV_AA);
                                    //                                cv::circle(outputImage, kfPts[kfMatches[i].trainIdx]+widthPt, 2, CV_RGB(0,200,0), 1, CV_AA);

                                } else {
                                    // pointsfailed norm test
                                    cDist.push_back(++failCounter);
                                    failedCF.push_back(klt_predictedPts[i]);
                                    failedKF.push_back(kfPts[KFMatches[i].trainIdx]);
                                    failedPF.push_back(prevPts[i]);
                                    //                                cv::line(outputImage, prevPts[i], klt_predictedPts[i], CV_RGB(255,0,50),1, CV_AA);
                                    //                                cv::circle(outputImage, klt_predictedPts[i], 2, CV_RGB(255,0,50), 1, CV_AA);
                                    //                                cv::circle(outputImage, kfPts[kfMatches[i].trainIdx]+widthPt, 2, CV_RGB(255,0,50), 1, CV_AA);
                                }
                            } else {
                                // pointsfailed step norm test
                                cStepDist.push_back(++failCounter);
                                failedCF.push_back(klt_predictedPts[i]);
                                failedKF.push_back(kfPts[KFMatches[i].trainIdx]);
                                failedPF.push_back(prevPts[i]);
                                //                                cv::line(outputImage, prevPts[i], klt_predictedPts[i], CV_RGB(255,0,50),1, CV_AA);
                                //                                cv::circle(outputImage, klt_predictedPts[i], 2, CV_RGB(255,0,50), 1, CV_AA);
                                //                                cv::circle(outputImage, kfPts[kfMatches[i].trainIdx]+widthPt, 2, CV_RGB(255,0,50), 1, CV_AA);
                            }
                        } else {
                            // point failed border test
                            cBorder.push_back(++failCounter);
                            failedCF.push_back(klt_predictedPts[i]);
                            failedKF.push_back(kfPts[KFMatches[i].trainIdx]);
                            failedPF.push_back(prevPts[i]);
                            //                            cv::line(outputImage, prevPts[i], klt_predictedPts[i], CV_RGB(200,0,0),1 , CV_AA);
                            //                            cv::circle(outputImage, klt_predictedPts[i], 2, CV_RGB(200,0,0), 1, CV_AA);
                            //                            cv::circle(outputImage, kfPts[kfMatches[i].trainIdx]+widthPt, 2, CV_RGB(200,0,0), 1, CV_AA);
                        }
                    } else {
                        // point failed KLT step
                        cKLT.push_back(++failCounter);
                        failedCF.push_back(klt_predictedPts[i]);
                        failedKF.push_back(kfPts[KFMatches[i].trainIdx]);
                        failedPF.push_back(prevPts[i]);
                        //                        cv::line(outputImage, prevPts[i], klt_predictedPts[i], CV_RGB(255,50,0),1 , CV_AA);
                        //                        cv::circle(outputImage, klt_predictedPts[i], 2, CV_RGB(255,50,0), 2, CV_AA);
                        //                        cv::circle(outputImage, kfPts[kfMatches[i].trainIdx]+widthPt, 2, CV_RGB(255,50,0), 2, CV_AA);
                    }
                }
            } else {
                ROS_WARN_THROTTLE(0.2, "KLT = NO POINTS TO TRACK AGAINST");
            }

//            cv::rectangle(outputImage, border, CV_RGB(0,0,200));



/*
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
*/

//            OVO::putInt(outputImage, trackedPts.size(), cv::Point(20,50), CV_RGB(0,255,0), true, "F: ");
            //}




            /// set points in current frame
            newFrame->swapPoints(trackedPts);

//            /// Drawing
//            cv::Mat matchImage;
//            if(frame->getPoints().size()>0){
//                // we have matches to draw
//                //cv::drawMatches(outputImage, frame->getKeypoints(), keyframe->getVisualImage(), keyframe->getKeypoints(), newMatches, matchImage, CV_RGB(0,0,100), CV_RGB(150,0,100),std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
//                cv::drawMatches(outputImage, KeyPoints(), keyframe->getVisualImage(), KeyPoints(), DMatches(), matchImage);
//            } else {
//                // we dont have matches to draw and dont want to invoke getKeypoints if we have none (otherwise it will detect)
//                cv::drawMatches(outputImage, KeyPoints(), keyframe->getVisualImage(), KeyPoints(), DMatches(), matchImage);

//            }

//            cv::Point l(85,45);
//            cv::Point r(width-85,45);
//            int length = r.x-l.x;
//            int maxPtsSafe = maxPts==0? 3000 : maxPts;
//            float ratio = (static_cast<float>(frame->getPoints().size())-minPts)/(maxPtsSafe-minPts);
//            cv::line(outputImage, l, l+cv::Point(length*frame->getPoints().size()/maxPtsSafe, 0), CV_RGB(255*(1-ratio),255*ratio,0), 3);
//            cv::line(outputImage, l+cv::Point(length*minPts/maxPtsSafe, -5), l+cv::Point(length*minPts/maxPtsSafe, 5), CV_RGB(255,0,0), 1);
//            cv::line(outputImage, l, r, CV_RGB(0,0,200), 1);
//            /// buffer values for next iteration


            std::swap(KFMatches, newKFMatches);
            std::swap(FMatches, newFMatches);
            previousFrame = currFrame;
            currFrame = newFrame;

            timeTrack = (ros::WallTime::now()-t0).toSec();
            ROS_INFO("KLT < TRACKED [OK: %4d]  [FAILED: KLT: %lu | STEP: %4lu | DIST: %4lu | BORDER: %4lu]", counter, cKLT.size(), cStepDist.size(), cDist.size(), cBorder.size());
            return;

        }

        // puts matches into class member prevMatches
        void doF2KF(FramePtr& newFrame){
            std::swap(FMatches, KFMatches); // store previous matches here
            matcher.match(newFrame, keyFrame, KFMatches, timeTrack, m_pxdist);
            border = cv::Rect(borderF2KF, cv::Point(newFrame->getSize().width, newFrame->getSize().height)-borderF2KF);

            // the first time this function is called we do not have a new frame
//            if (currFrame.empty()){
//                currFrame = newFrame;
//            }
            previousFrame = currFrame;
            currFrame = newFrame;
        }


        void doF2F(FramePtr& newFrame){
            border = cv::Rect(borderF2KF, cv::Point(newFrame->getSize().width, newFrame->getSize().height)-borderF2KF);
            if (!klt_init){
                klt_init = true;
                currFrame = keyFrame;
                matcher.match(newFrame, keyFrame, FMatches, timeTrack);
//                KFMatches = disparityFilter(KFMatches, newFrame, currFrame, m_pxdist, m_pxdist>0 && (m_pxdist<m_pxdistStep || m_pxdistStep==0) );
                //KFMatches = FMatches;
            } else {

//                // only match against points that have previous matches

//                DMatches newFMatches;
//                DMatches newKFMatches;
//                DMatches ms;
//                const KeyPoints& cfPts = currFrame->getKeypoints();
//                const KeyPoints& kfPts = keyFrame->getKeypoints();
//                const KeyPoints& nfPts = newFrame->getKeypoints();


                Ints queryIdx, trainIdx;
                OVO::match2ind(FMatches, queryIdx, trainIdx);
//                matcher.match(newFrame, currFrame, ms, time, 0.f, cv::Mat(), queryIdx );
                  matcher.match(newFrame, currFrame, FMatches, timeTrack, 0.f, cv::Mat(), queryIdx );

//                for (uint i=0; i<ms.size(); ++i){
//                    float normStep = cv::norm(nfPts[ms[i].queryIdx].pt-cfPts[ms[i].queryIdx].pt);
//                    if (normStep < m_pxdistStep || m_pxdistStep<=0){

//                        /// TODO kf-nf norm check

//                        newFMatches.push_back(cv::DMatch(ms[i].queryIdx, ms[i].trainIdx, normStep));
//                        //newKFMatches.push_back(cv::DMatch(ms[i].queryIdx, KFMatches[ms[i].trainIdx].trainIdx, 0)); /// TODO norm


//                    } else {
//                        //normStep failed
//                    }
//                }


//                //std::swap(newKFMatches, KFMatches);
//                std::swap(newFMatches, FMatches);
            }


            previousFrame = currFrame;
            currFrame = newFrame;



        }

      // Draws flow between the current frame and keyframe
        cv::Mat getVisualImage(){


            cv::Mat img;
            CvScalar col;
            cv::drawMatches(currFrame->getVisualImage(), KeyPoints(), keyFrame->getVisualImage(), KeyPoints(), DMatches(), img);
            cv::Mat imgOverlay = cv::Mat::zeros(img.size(), img.type());
            const cv::Point2f width = cv::Point2f(img.cols/2, 0);


            // draw borders on both images
            cv::rectangle(imgOverlay, border, CV_RGB(0,0,128));
            cv::rectangle(imgOverlay, cv::Rect(cv::Point(width.x, width.y)+border.tl(), cv::Point(width.x, width.y)+border.br()), CV_RGB(0,0,128));

            if (method==KLT){

                // draw all failed border points
                col = CV_RGB(200,0,0);
                for (uint i=0; i<cBorder.size(); ++i){
                    cv::circle(imgOverlay, failedCF[i], 3, col, 1, CV_AA);
                    cv::circle(imgOverlay, failedKF[i]+width, 3, col, 1, CV_AA);
                    cv::line(imgOverlay, failedCF[i], failedPF[i], col, 1, CV_AA);
                    cv::line(imgOverlay, failedKF[i]+width, failedCF[i]+width, col, 1, CV_AA);
                }
                // draw all failed klt points
                col = CV_RGB(255,0,0);
                for (uint i=0; i<cKLT.size(); ++i){
                    cv::circle(imgOverlay, failedCF[i], 3, col, 1, CV_AA);
                    cv::circle(imgOverlay, failedKF[i]+width, 3, col, 1, CV_AA);
                    cv::line(imgOverlay, failedCF[i], failedPF[i], col, 1, CV_AA);
                    cv::line(imgOverlay, failedKF[i]+width, failedCF[i]+width, col, 1, CV_AA);
                }
                // draw all failed step points
                col = CV_RGB(150,0,50);
                for (uint i=0; i<cStepDist.size(); ++i){
                    cv::circle(imgOverlay, failedCF[i], 3, col, 1, CV_AA);
                    cv::circle(imgOverlay, failedKF[i]+width, 3, col, 1, CV_AA);
                    cv::line(imgOverlay, failedCF[i], failedPF[i], col, 1, CV_AA);
                    cv::line(imgOverlay, failedKF[i]+width, failedCF[i]+width, col, 1, CV_AA);
                }
                // draw all failed dist points
                col = CV_RGB(150,50,0);
                for (uint i=0; i<cDist.size(); ++i){
                    cv::circle(imgOverlay, failedCF[i], 3, col, 1, CV_AA);
                    cv::circle(imgOverlay, failedKF[i]+width, 3, col, 1, CV_AA);
                    cv::line(imgOverlay, failedCF[i], failedPF[i], col, 1, CV_AA);
                    cv::line(imgOverlay, failedKF[i]+width, failedCF[i]+width, col, 1, CV_AA);
                }

                // Draw good keypoints
                const Points2f& kfpts = keyFrame->getPoints(true);
                const Points2f& cfpts = currFrame->getPoints(true);
                const Points2f& pfpts = previousFrame->getPoints(true);

                // current frame to key frame (right side)
                col = CV_RGB(0,200,0);
                for (uint i=0; i<KFMatches.size(); ++i){
                    cv::line(imgOverlay, width+cfpts[KFMatches[i].queryIdx], width+kfpts[KFMatches[i].trainIdx], OVO::getColor(0.f, m_pxdist, KFMatches[i].distance*1.1), 1, CV_AA);
                }
                // current frame to prev frame (left side)
                col = CV_RGB(0,0,200);
                for (uint i=0; i<FMatches.size(); ++i){
                    cv::line(imgOverlay, cfpts[FMatches[i].queryIdx], pfpts[FMatches[i].trainIdx], OVO::getColor(0.f, m_pxdistStep, FMatches[i].distance*1.1), 1, CV_AA);
                }




            } else if (method==F2KF){

                // Draw good keypoints
                const KeyPoints& kfpts = keyFrame->getKeypoints(true);
                const KeyPoints& cfpts = currFrame->getKeypoints(true);
                //const KeyPoints& pfpts = previousFrame->getKeypoints(true);

                // current frame to key frame (right side)
                CV_RGB(0,200,0);
                for (uint i=0; i<KFMatches.size(); ++i){                    
                    cv::circle(imgOverlay, cfpts[KFMatches[i].queryIdx].pt, 3, CV_RGB(0, 96*2, 0), 1, CV_AA);
                    cv::circle(imgOverlay, cfpts[KFMatches[i].queryIdx].pt, 2, OVO::getColor(0.f, m_pxdist, KFMatches[i].distance*1.1), 1, CV_AA);
                    cv::line(imgOverlay, width+cfpts[KFMatches[i].queryIdx].pt, width+kfpts[KFMatches[i].trainIdx].pt, OVO::getColor(0.f, m_pxdist, KFMatches[i].distance*1.1), 1, CV_AA);
                }

            } else if (method==F2F){
                // Draw good keypoints
                const Points2f& kfpts = keyFrame->getPoints(true);
                const Points2f& cfpts = currFrame->getPoints(true);
                const Points2f& pfpts = previousFrame->getPoints(true);

                // current frame to key frame (right side)
                col = CV_RGB(0,200,0);
                for (uint i=0; i<KFMatches.size(); ++i){
                    cv::line(imgOverlay, width+cfpts[KFMatches[i].queryIdx], width+kfpts[KFMatches[i].trainIdx], OVO::getColor(0.f, m_pxdist, KFMatches[i].distance*1.1), 1, CV_AA);
                }
                // current frame to prev frame (left side)
                col = CV_RGB(0,0,200);
                for (uint i=0; i<FMatches.size(); ++i){
                    cv::line(imgOverlay, cfpts[FMatches[i].queryIdx], pfpts[FMatches[i].trainIdx], OVO::getColor(0.f, m_pxdistStep, FMatches[i].distance*1.1), 1, CV_AA);
                }
                OVO::drawTextCenter(imgOverlay, "NOT IMPLEMENTED VIS", CV_RGB(255,0,0), 3, 1);

            } else {
                cv::drawMatches(currFrame->getVisualImage(), KeyPoints(), keyFrame->getVisualImage(), KeyPoints(), DMatches(), imgOverlay);
                OVO::drawTextCenter(imgOverlay, "NOT IMPLEMENTED VIS", CV_RGB(255,0,0), 3, 1);
            }



            ///  print disparity info

            /// Matches Info
            OVO::putInt(img, KFMatches.size(), cv::Point(10,2*25), CV_RGB(0,96*2,0),  true, "MA:");
            OVO::putInt(img, disparity, cv::Point(10,3*25), CV_RGB(0,96*2,0), false , "DI:");


            ///  print timing info
            OVO::putInt(img, timeTrack*1000., cv::Point(10,img.rows-3*25), CV_RGB(200,0,200), false, "T:");
            cv::addWeighted(img, 1.0, imgOverlay, 0.8, 0.0, img);

            return img;
        }





        void setParameter(ollieRosTools::VoNode_paramsConfig &config, uint32_t level){
            ROS_INFO("FTR > SETTING PARAM");
            matcher.setParameter(config, level);


            if (config.tracker!=preConfig.tracker){
                // We changed the tracker type - to be safe we clear some temporaty vars
                KFMatches.clear();
                FMatches.clear();
                klt_init = false;

                // this function gets called before we have any frames, so be check the pointers are not empty
                if (!currFrame.empty()){
                    currFrame->clearAllPoints();
                }
                if (!keyFrame.empty()) {
                    keyFrame->clearAllPoints();
                }
                if (!previousFrame.empty()){
                    previousFrame->clearAllPoints();
                }

                if (config.tracker<4){
                    method = KLT;
                } else if (config.tracker < 6){
                    method = F2F;
                } else {
                    method = F2KF;
                }

                f2kfRecovery = false;
                if (method!=F2KF) {
                    if (config.tracker ==2 || config.tracker ==3 || config.tracker ==5){
                        f2kfRecovery = true;
                    }
                }

            }
            failRatio = config.f2f_failRatio;

            maxPts          = config.kp_max;
            minPts          = config.kp_min;
            m_pxdist        = config.match_px;
            m_pxdistStep    = config.match_stepPx;
            klt_window      = cv::Size(config.klt_window*2+1, config.klt_window*2+1);
            klt_criteria    = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, config.klt_iter, config.klt_eps);
            klt_levels      = config.klt_levels;
            klt_flags       = (config.tracker==1||config.tracker==3) ? (cv::OPTFLOW_LK_GET_MIN_EIGENVALS) : 0; //
            klt_eigenThresh = config.klt_eigenThresh;
            borderF2KF = cv::Point(config.kp_border, config.kp_border);
            //ROS_INFO("EVF: %d EVT: %f", klt_flags, klt_eigenThresh);

            config.vo_doInitialisation = false;


            /*
            if (voDoIntitialise){
                // just for debugging
                // EXPERIMENTAL DRAWING
                cv::Mat flowImg;
                cv::Mat flowImgRot;
                DMatches ms = kfMatches;

                getFlowImage(flowImg, prevFrame, keyframe, ms, 1, false);
                getFlowImage(flowImgRot, prevFrame, keyframe, ms, 1, true);



                opengv::points_t worldPoints;
                odometry.initialise(prevFrame, keyframe, ms, worldPoints);
                getFlowImage(flowImg, prevFrame, keyframe, ms, 1, false, true);
                getFlowImage(flowImgRot, prevFrame, keyframe, ms, 1, true, true);
                cv::imshow("flow", flowImg);
                cv::imshow("flowRot", flowImgRot);
                publishMarkers(prevFrame, keyframe, ms, worldPoints);

                cv::waitKey(25);

                voDoIntitialise = false;
            }
            */
            preConfig = config;
            ROS_INFO("FTR < PARAM SET");

        }

//        void publishMarkers(FramePtr& f1, FramePtr& f2, const DMatches& matches, const opengv::points_t& worldPoints){
//            visualization_msgs::Marker ms;
//            ms.header.stamp = ros::Time::now();
//            ms.ns = "triangulation";
//            ms.id = 0;
//            ms.header.frame_id = "/cf_xyz";
//            ms.type = visualization_msgs::Marker::POINTS;
//            ms.action = visualization_msgs::Marker::ADD;
//            ms.scale.x = 0.05;
//            ms.scale.y = 0.05;

//            ms.color.a = 1.0;
//            ms.color.r = 1.0;
//            ms.color.g = 1.0;
//            ms.color.b = 1.0;

//            tf::poseTFToMsg(f2->getTFPose(), ms.pose);
//            uint i;
//            for (i=0; i< worldPoints.size();++i){
//                geometry_msgs::Point p;
//                tf::pointEigenToMsg(worldPoints[i],p);
//                ms.points.push_back(p);
//            }

//            pubMarkers.publish(ms);
//            pubTF.sendTransform(tf::StampedTransform(f1->getTFPose(),ros::Time::now(),"/cf_xyz","/F"));
//            pubTF.sendTransform(tf::StampedTransform(f2->getTFPose(),ros::Time::now(),"/cf_xyz", "/KF"));
//        }

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
