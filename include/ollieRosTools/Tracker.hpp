#ifndef TRACKER_HPP
#define TRACKER_HPP









/*

      // Draws flow between the current frame and keyframe
        cv::Mat getVisualImage(){


            cv::Mat img;
            CvScalar col;
            cv::drawMatches(currFrame->getVisualImage(), KeyPoints(), keyFrame->getVisualImage(), KeyPoints(), DMatches(), img);
            cv::Mat imgOverlay = cv::Mat::zeros(img.size(), img.type());
            const cv::Point2f width = cv::Point2f(img.cols/2, 0);


            // draw borders on both images
            cv::rectangle(imgOverlay, border, CV_RGB(0,0,128));

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

            }

            ///  print disparity info

            /// Matches Info
            OVO::putInt(img, KFMatches.size(), cv::Point(10,2*25), CV_RGB(0,96*2,0),  true, "MA:");
            OVO::putInt(img, disparity, cv::Point(10,8*25), CV_RGB(0,96*2,0), false , "DI:");


            ///  print timing info
            OVO::putInt(img, timeTrack*1000., cv::Point(10,img.rows-3*25), CV_RGB(200,0,200), false, "T:");
            cv::addWeighted(img, 1.0, imgOverlay, 0.8, 0.0, img);



            /// TODO: remove
//            float val = currFrame->compareSBI(keyFrame);
//            OVO::putInt(img, val, cv::Point(10,img.rows-7*25),  OVO::getColor(0.f, 1.f,val, true), false, "SBI:");

            return img;
        }
*/



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
