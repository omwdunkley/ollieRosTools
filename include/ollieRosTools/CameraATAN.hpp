#ifndef CAMERAATAN_HPP
#define CAMERAATAN_HPP
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <ros/ros.h>
#include <ollieRosTools/aux.hpp>
//#include <ollieRosTools/PreProcNode_paramsConfig.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <opencv2/highgui/highgui.hpp>

/**
  * Implements the ATAN camera model. For similar notation and more information see:
  * 'Straight Lines Have to Be Straight' (2001) by Frédéric Devernay and Olivier Faugeras
  */

using namespace Eigen;



class CameraATAN {

    public:
        CameraATAN():
            fx(0.425198),
            fy(0.579108),
            cx(0.477752),
            cy(0.581669),
            fov(0.970746),
            inWidth(-1),
            inHeight(-1),
            outWidth(720),
            outHeight(576),
            outZoom(1),
            zoomType(NOZOOM),
            interpolation(1),
            outSizeIsInSize(true)
        {
            //initialise(); // Now we initialise on first input image instead of here
        }


        cv::Mat rectify(const cv::Mat& imgIn){
            /// Rectifiy image using precomputed matricies and camInfoMessage
            cv::Mat imgOut;

            // Check input image size vs previous size. Initial previous size is -1,
            // so this forces an update the first time this function is called
            if (imgIn.cols != inWidth || imgIn.rows != inHeight){
                inWidth  = imgIn.cols;
                inHeight = imgIn.rows;

                if (outSizeIsInSize || interpolation<0){
                    outWidth = inWidth;
                    outHeight = inHeight;
                }
                // First imaged receiver -> we have a size -> we can init
                initialise();
            }


            if (interpolation>=0){
                // Interpolate
                cv::remap(imgIn, imgOut, rect_mapx, rect_mapy, interpolation);

//                cv::Mat testP;
//                cv::remap(imgOut, testP, rect_mapInvX, rect_mapInvY, interpolation);
//                cv::imshow("out",testP);
//                cv::imshow("in",imgIn);
//                cv::waitKey(30);

            } else {
                // Skip, just copy header
                imgOut = imgIn;
                //imgIn.copyTo(imgOut);
            }
            return imgOut;

        }

        const sensor_msgs::CameraInfoPtr& getCamInfo() const {
            return infoMsgPtr;
        }





        // Rotate a point in place
        void rotatePoint(cv::Point2f& inout, const cv::Point2f& center, const double angleRad) const {
            const cv::Point2f shifted = inout - center;
            inout = cv::Point2f(cos(angleRad)*shifted.x - sin(angleRad)*shifted.y, sin(angleRad)*shifted.x + cos(angleRad)*shifted.y)+ center;
        }
        void rotatePoint(cv::KeyPoint& inout, const cv::Point2f& center, const double angleRad) const {
            rotatePoint(inout.pt, center, angleRad);
        }

        KeyPoints rotatePoints(const KeyPoints& keypoints, const float angleRad, bool aroundOptical=true) const{
            // Copy key points
            KeyPoints keypointsOut = keypoints;

            // Get center
            cv::Point2f center;
            if (aroundOptical){
                center = cv::Point2f(infoMsgPtr->K[2], infoMsgPtr->K[5]); //cx, cy
            } else {
                center = cv::Point2f(infoMsgPtr->width/2., infoMsgPtr->height/2.);
            }

            // Rotate around center
            for (uint i=0; i<keypointsOut.size(); ++i){
                rotatePoint(keypointsOut[i].pt, center, angleRad);
            }


            /// DRAW
            cv::Mat img = cv::Mat::zeros(infoMsgPtr->height, infoMsgPtr->width, CV_8UC3);

            // Draw Chosen center
            cv::circle(img, center, 10, CV_RGB(255,0,0),3, CV_AA);

            // Draw line parallel to x
            cv::line(img,
                     cv::Point(0,                 center.y),
                     cv::Point(infoMsgPtr->width, center.y),
                     CV_RGB(0,255,0), 1, CV_AA);
            // Draw line parallel to y
            cv::line(img,
                     cv::Point(center.x, infoMsgPtr->height),
                     cv::Point(center.x, 0),
                     CV_RGB(0,255,0), 1, CV_AA);
            // Draw line parallel from optical center to image center
            cv::line(img,
                     cv::Point2f(infoMsgPtr->K[2], infoMsgPtr->K[5]),
                     cv::Point2f(infoMsgPtr->width/2., infoMsgPtr->height/2.),
                     CV_RGB(0,0,255), 1, CV_AA);
            for (uint i=0; i<keypointsOut.size(); ++i){
                cv::circle(img, keypointsOut[i].pt, 1, CV_RGB(255,0,0), 1, CV_AA);
                cv::circle(img, keypoints[i].pt,    1, CV_RGB(0,255,0), 1, CV_AA);
            }

            cv::imshow("center", img);
            cv::waitKey(10);

            return keypointsOut;
        }


        Points2 rotatePoints(const Points2& points, const float angleRad, bool aroundOptical=false) const{
            Points2 pointsOut = points;

            // Get center
            cv::Point2f center;
            if (aroundOptical){
                center = cv::Point2f(infoMsgPtr->K[2], infoMsgPtr->K[5]); //cx, cy
            } else {
                center = cv::Point2f(infoMsgPtr->width/2., infoMsgPtr->height/2.);
            }

            // Rotate around center
            for (uint i=0; i<pointsOut.size(); ++i){
                rotatePoint(pointsOut[i], center, angleRad);
            }



            /// DRAW
            cv::Mat img = cv::Mat::zeros(infoMsgPtr->height, infoMsgPtr->width, CV_8UC3);
            // Draw Chosen center
            cv::circle(img, center, 10, CV_RGB(255,0,0),3, CV_AA);
            // Draw line parallel to x
            cv::line(img,
                     cv::Point(0,                 center.y),
                     cv::Point(infoMsgPtr->width, center.y),
                     CV_RGB(0,255,0), 1, CV_AA);
            // Draw line parallel to y
            cv::line(img,
                     cv::Point(center.x, infoMsgPtr->height),
                     cv::Point(center.x, 0),
                     CV_RGB(0,255,0), 1, CV_AA);
            // Draw line parallel from optical center to image center
            cv::line(img,
                     cv::Point2f(infoMsgPtr->K[2], infoMsgPtr->K[5]),
                     cv::Point2f(infoMsgPtr->width/2., infoMsgPtr->height/2.),
                     CV_RGB(0,0,255), 1, CV_AA);
            // Draw points
            for (uint i=0; i<pointsOut.size(); ++i){
                cv::circle(img, pointsOut[i], 1, CV_RGB(255,0,0), 1, CV_AA);
                cv::circle(img, points[i],    1, CV_RGB(0,255,0), 1, CV_AA);
            }
            cv::imshow("center", img);
            cv::waitKey(10);



            return pointsOut;
        }


        KeyPoints rectifyPoints(const KeyPoints& keypoints, bool pointsFromCamera=true){
            KeyPoints kps =keypoints;
            if (pointsFromCamera && interpolation>=0 ){
                /// Incoming points are rectified
                // do nothing
            } else {
                Points2 points;
                cv::KeyPoint::convert(keypoints, points);
                Matrix<float, Dynamic, 2, RowMajor> f2d = Map<Matrix<float, Dynamic, 2, RowMajor> >(cv::Mat(points).ptr<float>(),points.size(), 2); //O(1)
                f2d.col(0).array() -= icx;
                f2d.col(1).array() -= icy;
                f2d.col(0) /= ifx;
                f2d.col(1) /= ify;
                const MatrixXf r = f2d.rowwise().norm();
                const ArrayXf fac = (r* fov).array().tan() / (r*d2t).array();
                f2d.array().colwise() *= fac;
                f2d.col(0) *= ifx; //should it not sometimes be ofx depending on what we want?
                f2d.col(1) *= ify;
                f2d.col(0).array() += icx;
                f2d.col(1).array() += icy;

                // Put point locations back into keypoints
                for (uint i=0; i<points.size(); ++i){
                    kps[i].pt.x = f2d(i,0);
                    kps[i].pt.y = f2d(i,1);
                }
            }

            /// DRAW
            cv::Point2f center = center = cv::Point2f(infoMsgPtr->width/2., infoMsgPtr->height/2.);
            cv::Mat img = cv::Mat::zeros(infoMsgPtr->height, infoMsgPtr->width, CV_8UC3);

            // Draw Chosen center
            cv::circle(img, center, 10, CV_RGB(255,0,0),3, CV_AA);

            // Draw line parallel to x
            cv::line(img,
                     cv::Point(0,                 center.y),
                     cv::Point(infoMsgPtr->width, center.y),
                     CV_RGB(0,255,0), 1, CV_AA);
            // Draw line parallel to y
            cv::line(img,
                     cv::Point(center.x, infoMsgPtr->height),
                     cv::Point(center.x, 0),
                     CV_RGB(0,255,0), 1, CV_AA);
            // Draw line parallel from optical center to image center
            cv::line(img,
                     cv::Point2f(infoMsgPtr->K[2], infoMsgPtr->K[5]),
                     cv::Point2f(infoMsgPtr->width/2., infoMsgPtr->height/2.),
                     CV_RGB(0,0,255), 1, CV_AA);
            // Draw circles before and after
            for (uint i=0; i<kps.size(); ++i){
                cv::circle(img, kps[i].pt, 2, CV_RGB(255,0,0), 1, CV_AA);
                cv::circle(img, keypoints[i].pt,    1, CV_RGB(0,255,0), 1, CV_AA);
            }

            cv::imshow("rect", img);
            cv::waitKey(10);

            return kps;
        }


        KeyPoints unrectifyPoints(const KeyPoints& keypoints){
            KeyPoints kps;

            return kps;
        }

        void bearingVectors(const KeyPoints& keypoints, MatrixXf& bearings) const{
            Points2 points;
            cv::KeyPoint::convert(keypoints, points);
            bearingVectors(points, bearings);
        }

        void bearingVectors(const Points2& points, MatrixXf& bearings) const{
            /// Given 2d points, compute the bearing vecotrs that point from the
            /// optical center in the direction of the features.
            // Note that this class might be rectifying images (in which case we just use the pinhole model)
            // or the images are not yet rectified in which case we must first rectify the 2d points


            // Use the precomputed look up table to rectify the points.
            const bool useLUT = false;

            Matrix<float, Dynamic, 2, RowMajor> f2d;

            //std::cout << std::endl<< std::endl<< "P" << std::endl << P << std::endl;

            if (interpolation>=0){
                /// Incoming points are rectified
               f2d = Map<Matrix<float, Dynamic, 2, RowMajor> >(cv::Mat(points).ptr<float>(),points.size(), 2); //O(1)



                //std::cout << "IN EIGEN:" << std::endl << f2d << std::endl;


            } else {                
                /// Incoming points are not rectified

                if (useLUT) {
                    /// Use lookup table to rectify points
                    //TODO
                    //cv::remap(points, pointsr,rect_mapx, rect_mapy, CV_INTER_LINEAR);

                } else {
                    /// Compute each point individually

//                    MatrixXf f2dCV(points.size(),2);
//                    for (uint i=0; i<points.size(); ++i){
//                        const float ox = (points[i].x - icx) / ifx;
//                        const float oy = (points[i].y - icy) / ify;
//                        const float r = sqrt(ox*ox + oy*oy);
//                        const float fac = tan(r * fov) / (r*d2t);
//                        f2dCV(i,0) = fac*ox;
//                        f2dCV(i,1) = fac*oy; // ofy*fac*oy+ocy;
//                    }
//                    std::cout << "RECTIFIED CV:" << std::endl << f2dCV << std::endl;

                    f2d = Map<Matrix<float, Dynamic, 2, RowMajor> >(cv::Mat(points).ptr<float>(),points.size(), 2); //O(1)
                    //std::cout << "IN EIGEN:" << std::endl << f2d << std::endl;
                    f2d.col(0).array() -= icx;
                    f2d.col(1).array() -= icy;
                    f2d.col(0) /= ifx;
                    f2d.col(1) /= ify;

                    const MatrixXf r = f2d.rowwise().norm();
                    const ArrayXf fac = (r* fov).array().tan() / (r*d2t).array();
                    f2d.array().colwise() *= fac;

                    f2d.col(0) *= fx;
                    f2d.col(1) *= fy;
                    f2d.col(0).array() += cx;
                    f2d.col(1).array() += cy;
                    //std::cout << "RECTIFIED:" << std::endl << f2d << std::endl;

                }

            }

            // Make homogenious, transpose 3xN
            const MatrixXf f2dh = f2d.transpose().colwise().homogeneous();
            //std::cout << "TCH:" << std::endl << f2dh << std::endl;

            // Project 2d -> 3d rays
            bearings = Pinv.solve(f2dh);
            //std::cout << "SOLVE:" << std::endl << bearings << std::endl;

            bearings.colwise().normalize();
            //std::cout << "CN" << std::endl << bearings << std::endl;

            bearings.transposeInPlace(); // Nx3
            //std::cout << "T:" << std::endl<< bearings << std::endl;

        }

        //ollieRosTools::PreProcNode_paramsConfig& setParameter(ollieRosTools::PreProcNode_paramsConfig &config, uint32_t level){
        void setParams(const double zoomFactor, const int zoom,
                       const int PTAMRectify,
                       const bool sameOutInSize,
                       const int width, const int height,
                       const double fx, const double fy,
                       const double cx, const double cy,
                       const double fov){
            /// Update the settings use roy dynamic reconfigure

            this->outZoom = zoomFactor;
            interpolation = PTAMRectify;
            outSizeIsInSize = sameOutInSize;

            // Use image size as outsize if we do not allow manual size setting or if we are not rectifying
            if (outSizeIsInSize || interpolation<0){
                outWidth = inWidth;
                outHeight = inHeight;
            } else {
                outWidth = width;
                outHeight = height;
            }

            zoomType = static_cast<ZoomType>(zoom);

            // PTAM params
            this->fx = fx;
            this->fy = fy;
            this->cx = cx;
            this->cy = cy;
            this->fov = fov;

            if (inWidth>0 && inHeight>0){
                // Only call this after we have an image size
                initialise();
            }
            //return config;
        }



    private:
        // Keep track of zoom types. This has to match the order of zoom_enum defined in PreProcNode_params.cfg
        enum ZoomType { NOZOOM, FULL_MAX, FULL_MIN, CROP, MANUAL };

        // Warping matricies, precomputed using PTAM paramters after every settings change
        cv::Mat rect_mapx, rect_mapy;
        //cv::Mat rect_mapInvX, rect_mapInvY;

        // Camera_info that gets sent with the rectified image.
        sensor_msgs::CameraInfoPtr infoMsgPtr;

        // Eigen stuff
        Eigen::Matrix3f P;
        Eigen::PartialPivLU<Matrix3f> Pinv;

        // PTAM parameters
        float fx,fy,cx,cy,fov; // normalised, ie [0 1]
        float d2t, ifx, ify, icx, icy;

        // Set according to imcoming images
        int inWidth, inHeight;

        /// User changeable variables
        // Rectified image size
        int outWidth, outHeight;

        // Rectified image scale and scale method
        float outZoom;
        ZoomType zoomType;
        // interpolation method (<0 means off)
        int interpolation;
        // Allow resize or not
        bool outSizeIsInSize;


        void initialise(){
            /// Depending on the incoming image size, outgoing image size/zoom, update the camera_info message and then precompute the warping matrices

            // Input parameters
            d2t = 2.0 * tan(fov / 2.0); // equation (13) inner part [see reference at top]
            ifx = fx * inWidth ;
            ify = fy * inHeight;
            icx = cx * inWidth  - 0.5;
            icy = cy * inHeight - 0.5;



            sensor_msgs::CameraInfo infoMsg;

            if (interpolation<0){
                /// NOT RECTIFYING IMAGE
                P = Matrix3f::Identity();
                P(0, 0) = fx;
                P(1, 1) = fy;
                P(0, 2) = cx;
                P(1, 2) = cy;
                // Invert for later use
                Pinv = P.partialPivLu();

                /// CREATE PINHILE CAMERA_INFO MESSAGE

                infoMsg.width = inWidth;
                infoMsg.height = inHeight;
                infoMsg.distortion_model = "plumb_bob";

//                // Rectification rotation matrix (only needed stereo cameras really...)
//                infoMsg.R.at(0) = 1;
//                infoMsg.R.at(4) = 1;
//                infoMsg.R.at(8) = 1;
//                // Intrinsic camera matrix for the raw (distorted) image
//                infoMsg.K.at(0) = inHeight; /// WTF are height and wwidth switched?
//                infoMsg.K.at(4) = inWidth;
//                infoMsg.K.at(8) = 1;
//                infoMsg.K.at(2) = inHeight/2;
//                infoMsg.K.at(5) = inWidth/2;

//                // Projection/camera matrix that specifies the intrinsic (camera) matrix of the processed (rectified) image
//                infoMsg.P.at(0) = inHeight;
//                infoMsg.P.at(5) = inWidth;
//                infoMsg.P.at(10) = 1;
//                infoMsg.P.at(2) = inHeight/2;
//                infoMsg.P.at(6) = inWidth/2;
                // Intrinsic camera matrix for the raw (distorted) image
                infoMsg.K.at(0) = ifx;
                infoMsg.K.at(4) = ify;
                infoMsg.K.at(8) = 1;
                infoMsg.K.at(2) = icx;
                infoMsg.K.at(5) = icy;

                // Projection/camera matrix that specifies the intrinsic (camera) matrix of the processed (rectified) image
                infoMsg.P.at(0) = ifx;
                infoMsg.P.at(5) = ify;
                infoMsg.P.at(10) = 1;
                infoMsg.P.at(2) = icx;
                infoMsg.P.at(6) = icy;

                ROS_INFO("   Default camera matrix without rectification [%dpx*%dpx (%.3f)]",
                         inWidth, inHeight, static_cast<float>(inWidth)/inHeight);


            } else {
                /// RECIFYING IMAGE

                // Output paramters
                float ofx ,ofy, ocx, ocy;


                if (zoomType == NOZOOM){
                    /// Dont apply any zoom, just use the output image size. This does not pan or change the aspect ratio either.
                    ROS_INFO("Initialising calibration: No Zoom");
                    ofx = fx * outWidth;
                    ofy = fy * outHeight;
                    ocx = cx * outWidth  - 0.5;
                    ocy = cy * outHeight - 0.5;

                } else if (zoomType == FULL_MAX || zoomType == FULL_MIN){
                    /// Zoom and pan so the furthest vertical/horizontal valid part of the image is also present.
                    /// If zoomType == FULL_MIN, This might still crop the extremly distorted corners


                    // Find radii to each side
                    const float radL = icx/ifx;
                    const float radR = (inWidth-1 - icx)/ifx;
                    const float radT = icy/ify;
                    const float radB = (inHeight-1 - icy)/ify;

                    // Find radii to each corner distored
                    const float radTL = sqrt(pow(radT, 2) + pow(radL, 2));
                    const float radTR = sqrt(pow(radT, 2) + pow(radR, 2));
                    const float radBL = sqrt(pow(radB, 2) + pow(radL, 2));
                    const float radBR = sqrt(pow(radB, 2) + pow(radR, 2));

                    // Radii to each undistored corner: equation (14) inverse distortion function [see reference at top]
                    float invTL = tan(radTL * fov)/d2t * outZoom;
                    float invTR = tan(radTR * fov)/d2t * outZoom;
                    float invBL = tan(radBL * fov)/d2t * outZoom;
                    float invBR = tan(radBR * fov)/d2t * outZoom;

                    float hor, vert, invHor, invVert;
                    if (zoomType == FULL_MIN){
                        ROS_INFO("Initialising calibration: Zoom Full Min");
                        // std::maximum distorted
                        hor  =  std::min(std::min(radBR, radTR), std::min(radBL, radTL));
                        vert =  std::min(std::min(radTR, radTL), std::min(radBL, radBR));
                        // std::maximum undistroted
                        invHor  = std::min(std::min(invBR, invTR), std::min(invBL, invTL));
                        invVert = std::min(std::min(invTR, invTL), std::min(invBL, invBR));

                        // Ratios define output
                        ofx = ifx * ((hor ) / (invHor )) * ((float)outWidth /(float)inWidth );
                        ofy = ify * ((vert) / (invVert)) * ((float)outHeight/(float)inHeight);
                        ocx = invHor/hor*ofx/ifx*icx;
                        ocy = invVert/vert*ofy/ify*icy;
                    } else {
                        ROS_INFO("Initialising calibration: Zoom Full Max");
                        // std::maximum distorted
                        hor  = std::max(radBR, radTR) + std::max(radBL, radTL);
                        vert = std::max(radTR, radTL) + std::max(radBL, radBR);
                        // std::maximum undistroted
                        invHor  = std::max(invBR, invTR) + std::max(invBL, invTL);
                        invVert = std::max(invTR, invTL) + std::max(invBL, invBR);

                        // Ratios define output
                        ofx = ifx * ((hor ) / (invHor )) * ((float)outWidth /(float)inWidth );
                        ofy = ify * ((vert) / (invVert)) * ((float)outHeight/(float)inHeight);
                        ocx = std::max(invBL/radBL, invTL/radTL)*ofx/ifx*icx;
                        ocy = std::max(invTL/radTL, invTR/radTR)*ofy/ify*icy;
                    }



                } else if (zoomType == CROP){
                    ROS_INFO("Initialising calibration: Zoom Crop");
                    /// Zoom, pan and stretch so the whole output image is valid. Might crop sides of image depending on distortion
                    // find left-most and right-most radius
                    float radL = (icx)/ifx;
                    float radR = (inWidth-1 - icx)/ifx;
                    float radT = (icy)/ify;
                    float radB = (inHeight-1 - icy)/ify;

                    // Undisorted radii to each side
                    float invL = tan(radL * fov)/d2t* outZoom;
                    float invR = tan(radR * fov)/d2t* outZoom;
                    float invT = tan(radT * fov)/d2t* outZoom;
                    float invB = tan(radB * fov)/d2t* outZoom;

                    // Ratios define output
                    ofx = ifx * ((radL + radR) / (invL + invR)) * ((float)outWidth /(float)inWidth);
                    ofy = ify * ((radT + radB) / (invT + invB)) * ((float)outHeight/(float)inHeight);
                    ocx = (invL/radL)*ofx/ifx*icx;
                    ocy = (invT/radT)*ofy/ify*icy;


                } else { //if (zoomType == MANUAL){
                    /// Just manual scale, no panning
                    ROS_INFO("Initialising calibration: Manual Zoom: %.2f", outZoom);
                    ofx = fx * outWidth  * outZoom;
                    ofy = fy * outHeight * outZoom;
                    ocx = cx * outWidth  - 0.5;
                    ocy = cy * outHeight - 0.5;
                }


                /// PRECOMPUTE WARP MATRIX
                rect_mapx = cv::Mat(outHeight, outWidth, CV_32FC1, cv::Scalar(0));
                rect_mapy = cv::Mat(outHeight, outWidth, CV_32FC1, cv::Scalar(0));

                for (int x=0; x<outWidth; ++x){
                    for (int y=0; y<outHeight; ++y){
                        const float ix = (x - ocx) / ofx;
                        const float iy = (y - ocy) / ofy;
                        const float r = sqrt(ix*ix + iy*iy);
                        const float fac = r<0.01 ? 1:atan(r * d2t)/(fov*r);
                        rect_mapx.at<float>(y,x) = ifx*fac*ix+icx;
                        rect_mapy.at<float>(y,x) = ify*fac*iy+icy;
                    }
                }


                //            rect_mapInvX = cv::Mat(inHeight, inWidth, CV_32FC1, cv::Scalar(0));
                //            rect_mapInvY = cv::Mat(inHeight, inWidth, CV_32FC1, cv::Scalar(0));
                //            for (int x=0; x<inWidth; ++x){
                //                for (int y=0; y<inHeight; ++y){
                //                    const float ox = (x - icx) / ifx;
                //                    const float oy = (y - icy) / ify;
                //                    const float r = sqrt(ox*ox + oy*oy);
                //                    const float fac = tan(r * fov)/(2*r*tan(fov/2));
                //                    rect_mapInvX.at<float>(y,x) = ofx*fac*ox+ocx;
                //                    rect_mapInvY.at<float>(y,x) = ofy*fac*oy+ocy;
                //                }
                //            }

                P = Matrix3f::Identity();
                P(0, 0) = ofx;
                P(1, 1) = ofy;
                P(0, 2) = ocx;
                P(1, 2) = ocy;
                // Invert for later use
                Pinv = P.partialPivLu();

                /// CREATE PINHILE CAMERA_INFO MESSAGE

                infoMsg.width = outWidth;
                infoMsg.height = outHeight;
                infoMsg.distortion_model = "plumb_bob";

                // Rectification rotation matrix (only needed stereo cameras really...)
                infoMsg.R.at(0) = 1;
                infoMsg.R.at(4) = 1;
                infoMsg.R.at(8) = 1;

                // Intrinsic camera matrix for the raw (distorted) image
                infoMsg.K.at(0) = ofx;
                infoMsg.K.at(4) = ofy;
                infoMsg.K.at(8) = 1;
                infoMsg.K.at(2) = ocx;
                infoMsg.K.at(5) = ocy;

                // Projection/camera matrix that specifies the intrinsic (camera) matrix of the processed (rectified) image
                infoMsg.P.at(0) = ofx;
                infoMsg.P.at(5) = ofy;
                infoMsg.P.at(10) = 1;
                infoMsg.P.at(2) = ocx;
                infoMsg.P.at(6) = ocy;

                ROS_INFO("   Input [%dpx*%dpx (%.3f)] Output: [%dpx*%dpx (%.3f)]",
                         inWidth, inHeight, static_cast<float>(inWidth)/inHeight,
                         outWidth, outHeight, static_cast<float>(outWidth)/outHeight);
                ROS_INFO("   NEW MODEL:  Focal: [%.1f, %.1f] Center: [%.1f, %.1f]", ofx, ofy, ocx, ocy);

            }




            // The distortion parameters
            infoMsg.D.clear();
            infoMsg.D.push_back(fov);
            infoMsg.D.push_back(0);
            infoMsg.D.push_back(0);
            infoMsg.D.push_back(0);
            infoMsg.D.push_back(0);

            // Update infoMsgPtr, passed by reference
            infoMsgPtr = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo(infoMsg));








        }

};

#endif // CAMERAATAN_HPP
