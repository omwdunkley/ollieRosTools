#ifndef CAMERAATAN_HPP
#define CAMERAATAN_HPP
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <ros/ros.h>
#include <ollieRosTools/aux.hpp>
//#include <ollieRosTools/PreProcNode_paramsConfig.h>

//#include <g2o/types/slam3d/parameter_camera.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <opencv2/highgui/highgui.hpp>
#include <image_geometry/pinhole_camera_model.h>

/**
  * Implements the ATAN camera model. For similar notation and more information see:
  * 'Straight Lines Have to Be Straight' (2001) by Frédéric Devernay and Olivier Faugeras
  */

using namespace Eigen;



class CameraATAN {

    public:
        //sensor_msgs::CameraInfo camInfoSynthetic;
        image_geometry::PinholeCameraModel pinholeSynthetic;
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



        cv::Mat rectify(const cv::Mat& imgIn, sensor_msgs::CameraInfo camInfo){
            ROS_ASSERT(USE_SYNTHETIC);
            pinholeSynthetic.fromCameraInfo(camInfo);

            infoMsgPtr = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo(camInfo));

            ROS_INFO("CAM > RECTIFYING");
            cv::Mat imgOut;


            if(USE_SYNTHETIC){
                if  (interpolation>-1){
                    pinholeSynthetic.rectifyImage(imgIn, imgOut, interpolation);
                    ROS_INFO("CAM [SYN] < RECTIFYIED");
                } else {
                    imgOut = imgIn;
                    ROS_INFO("CAM [SYN] < RECTIFICATION OFF, PASS THROUGH");
                }
            }


            return imgOut;
        }



        cv::Mat rectify(const cv::Mat& imgIn){
            /// Rectifiy image using precomputed matricies and camInfoMessage
            ROS_INFO("CAM > RECTIFYING");
            cv::Mat imgOut;

            ROS_ASSERT(!USE_SYNTHETIC);





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

                ROS_INFO("CAM = WARPING RECTIFICATION");
                cv::remap(imgIn, imgOut, rect_mapx, rect_mapy, interpolation);



//                cv::Mat testP;
//                cv::remap(imgOut, testP, rect_mapInvX, rect_mapInvY, interpolation);
//                cv::imshow("out",testP);
//                cv::imshow("in",imgIn);
//                cv::waitKey(30);
                ROS_INFO("CAM < RECTIFYIED");
            } else {
                // Skip, just copy header
                imgOut = imgIn;

                // draw unrectified grid

//                int step = 25;
//                int shift = 8;
//                for (int x=-outWidth*2; x<outWidth*2; x+=step){
//                    std::vector<cv::Point> contour;
//                    for (int y=-outHeight*2; y<outHeight*2; y+=step){
//                        const float ix = (x - infoMsgPtr->K.at(2)) / infoMsgPtr->K.at(0);
//                        const float iy = (y - infoMsgPtr->K.at(5)) / infoMsgPtr->K.at(4);
//                        const float r = sqrt(ix*ix + iy*iy);
//                        const float fac = r<0.01 ? 1:atan(r * d2t)/(fov*r);
//                        const float ox = ifx*fac*ix+icx;
//                        const float oy = ify*fac*iy+icy;
//                        contour.push_back(cv::Point(ox*8, oy*8));
//                    }
//                    const cv::Point *pts = (const cv::Point*) cv::Mat(contour).data;
//                    int npts = cv::Mat(contour).rows;
//                    cv::polylines(imgOut, &pts,&npts, 1, false, CV_RGB(0,255,0),1,CV_AA, 3);
//                }
//                for (int y=-outHeight*2; y<outHeight*2; y+=step){
//                    std::vector<cv::Point> contour;
//                    for (int x=-outWidth*2; x<outWidth*2; x+=step){
//                        const float ix = (x - infoMsgPtr->K.at(2)) / infoMsgPtr->K.at(0);
//                        const float iy = (y - infoMsgPtr->K.at(5)) / infoMsgPtr->K.at(4);
//                        const float r = sqrt(ix*ix + iy*iy);
//                        const float fac = r<0.01 ? 1:atan(r * d2t)/(fov*r);
//                        const float ox = ifx*fac*ix+icx;
//                        const float oy = ify*fac*iy+icy;
//                        contour.push_back(cv::Point(ox*8,oy*8));
//                    }
//                    const cv::Point *pts = (const cv::Point*) cv::Mat(contour).data;
//                    int npts = cv::Mat(contour).rows;
//                    cv::polylines(imgOut, &pts,&npts, 1, false, CV_RGB(0,255,0),1, CV_AA, 3);

//                }



                ROS_INFO("CAM < RECTIFICATION OFF, PASS THROUGH");
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
        void rotatePoint(cv::KeyPoint& inout, const cv::Point2d& center, const double angleRad) const {
            rotatePoint(inout.pt, center, angleRad);
        }

        /// TODO DO this with eigen, and we might have some+- mistakes somewhere along the way
        KeyPoints rotatePoints(const KeyPoints& keypoints, const double angleRad, bool aroundOptical=true) const{            
            // Copy key points
            KeyPoints keypointsOut = keypoints;
            ROS_INFO("CAM > Rotating [%lu] Keypoints [%.f] degrees around the [%s] center", keypointsOut.size(), angleRad*toDeg, aroundOptical ? "Optical":"Image");

            // Get center
            cv::Point2d center;
            if (aroundOptical){
                center = cv::Point2d(infoMsgPtr->K[2], infoMsgPtr->K[5]); //cx, cy
            } else {
                center = cv::Point2d(infoMsgPtr->width/2., infoMsgPtr->height/2.);
            }

            // Rotate around center
            for (uint i=0; i<keypointsOut.size(); ++i){
                rotatePoint(keypointsOut[i].pt, center, angleRad);
            }


//            /// DRAW
//            cv::Mat img = cv::Mat::zeros(infoMsgPtr->height, infoMsgPtr->width, CV_8UC3);
//            // Draw Chosen center
//            cv::circle(img, center, 10, CV_RGB(255,0,0),3, CV_AA);
//            // Draw line parallel to x
//            cv::line(img,
//                     cv::Point(0,                 center.y),
//                     cv::Point(infoMsgPtr->width, center.y),
//                     CV_RGB(0,255,0), 1, CV_AA);
//            // Draw line parallel to y
//            cv::line(img,
//                     cv::Point(center.x, infoMsgPtr->height),
//                     cv::Point(center.x, 0),
//                     CV_RGB(0,255,0), 1, CV_AA);
//            // Draw line parallel from optical center to image center
//            cv::line(img,
//                     cv::Point2d(infoMsgPtr->K[2], infoMsgPtr->K[5]),
//                     cv::Point2d(infoMsgPtr->width/2., infoMsgPtr->height/2.),
//                     CV_RGB(0,0,255), 1, CV_AA);
//            for (uint i=0; i<keypointsOut.size(); ++i){
//                cv::circle(img, keypointsOut[i].pt, 1, CV_RGB(255,0,0), 1, CV_AA);
//                cv::circle(img, keypoints[i].pt,    1, CV_RGB(0,255,0), 1, CV_AA);
//            }
//            cv::imshow("center", img);
//            cv::waitKey(10);
            ROS_INFO("CAM < Rotated");

            return keypointsOut;
        }

/*
        Points2 rotatePoints(const Points2& points, const double angleRad, bool aroundOptical=false) const{
            Points2 pointsOut = points;

            // Get center
            cv::Point2d center;
            if (aroundOptical){
                center = cv::Point2d(infoMsgPtr->K[2], infoMsgPtr->K[5]); //cx, cy
            } else {
                center = cv::Point2d(infoMsgPtr->width/2., infoMsgPtr->height/2.);
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
                     cv::Point2d(infoMsgPtr->K[2], infoMsgPtr->K[5]),
                     cv::Point2d(infoMsgPtr->width/2., infoMsgPtr->height/2.),
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
*/



        // Takes distorted points and rectifies them
        KeyPoints rectifyPoints(const KeyPoints& keypoints, bool pointsFromCamera=true){
            KeyPoints kps =keypoints;

            if (USE_SYNTHETIC){
                if (pointsFromCamera && interpolation>=0 ){
                    /// Incoming points are rectified
                    // do nothing

                } else {
                    for (uint i=0; i<kps.size(); ++i){
                        kps[i].pt = pinholeSynthetic.rectifyPoint(keypoints[i].pt);
                    }
                }

            } else {



                if ((pointsFromCamera && interpolation>=0 )){
                    /// Incoming points are rectified
                    // do nothing


                } else {
                    Points2f points;
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


        // Takes rectified points and distorts them NOT IMPLEMENTED
        Matrix<float, Dynamic, 2, RowMajor> unrectifyPoints(const Points2f& points, bool force=false){
            Points2f pts;

            Matrix<float, Dynamic, 2, RowMajor> f2d;

            if (USE_SYNTHETIC){
                if ((interpolation>=0 && !force)){
                    /// Incoming points are rectified
                } else {


                }
            } else {



                //            const float ix = (x - infoMsgPtr->K.at(2)) / infoMsgPtr->K.at(0);
                //            const float iy = (y - infoMsgPtr->K.at(5)) / infoMsgPtr->K.at(4);
                //            const float r = sqrt(ix*ix + iy*iy);
                //            const float fac = r<0.01 ? 1:atan(r * d2t)/(fov*r);
                //            const float ox = ifx*fac*ix+icx;
                //            const float oy = ify*fac*iy+icy;
                //            contour.push_back(cv::Point(ox,oy));


                f2d = Map<Matrix<float, Dynamic, 2, RowMajor> >(cv::Mat(points).ptr<float>(),points.size(), 2); //O(1)
                if ((interpolation>=0 && !force)){
                    /// Incoming points are rectified
                    f2d = Map<Matrix<float, Dynamic, 2, RowMajor> >(cv::Mat(points).ptr<float>(),points.size(), 2); //O(1)
                } else {
                    f2d.col(0).array() -= infoMsgPtr->K.at(2);
                    f2d.col(1).array() -= infoMsgPtr->K.at(5);
                    f2d.col(0) /= infoMsgPtr->K.at(0);
                    f2d.col(1) /= infoMsgPtr->K.at(4);
                    const MatrixXf r = f2d.rowwise().norm();
                    const ArrayXf fac = (r* d2t).array().tan() / (fov*r).array();
                    f2d.array().colwise() *= fac;
                    f2d.col(0) *= ifx;
                    f2d.col(1) *= ify;
                    f2d.col(0).array() += icx;
                    f2d.col(1).array() += icy;
                }
            }

            pts.reserve(pts.size());
            ROS_ASSERT_MSG(0, "CAM = NOT IMPLEMENTED unrectifyPoints");
            return f2d;
        }


        // Computes bearing vectors and rectified points from keypoints
        void bearingVectors(const KeyPoints& keypoints, MatrixXd& bearings, MatrixXd& pointsRectified) const{
            Points2f points;
            cv::KeyPoint::convert(keypoints, points);
            bearingVectors(points, bearings, pointsRectified);
        }

        void bearingVectors(const Points2f& points, MatrixXd& bearings, MatrixXd& pointsRectified) const{
            /// Given 2d points, compute the bearing vecotrs that point from the
            /// optical center in the direction of the features.
            // Note that this _class_ might be rectifying images (in which case the input points are rectified and we just use the pinhole model)
            // or the images/points are not yet rectified in which case we must first rectify the 2d points


            Matrix<float, Dynamic, 2, RowMajor> f2d;


            if (USE_SYNTHETIC){
                if (interpolation>=0){
                    /// Incoming points are rectified
                    f2d = Map<Matrix<float, Dynamic, 2, RowMajor> >(cv::Mat(points).ptr<float>(),points.size(), 2); //O(1)
                } else {
                    /// Incoming points are not rectified
                    f2d = Map<Matrix<float, Dynamic, 2, RowMajor> >(cv::Mat(points).ptr<float>(),points.size(), 2); //O(1)
                    f2d.col(0).array() -= pinholeSynthetic.cx() -0.5;
                    f2d.col(1).array() -= pinholeSynthetic.cy() -0.5;
                    f2d.col(0) /= pinholeSynthetic.fx();
                    f2d.col(1) /= pinholeSynthetic.fy();

                    pointsRectified = f2d.cast<double>();
                    bearings = f2d.rowwise().homogeneous().cast<double>();
                    bearings.rowwise().normalize();

                }
            } else {

                if (interpolation>=0){
                    /// Incoming points are rectified
                    f2d = Map<Matrix<float, Dynamic, 2, RowMajor> >(cv::Mat(points).ptr<float>(),points.size(), 2); //O(1)
                } else {
                    /// Incoming points are not rectified
                    //                MatrixXf f2dCV(points.size(),2);
                    //                for (uint i=0; i<points.size(); ++i){
                    //                    const double ox = (points[i].x - icx) / ifx;
                    //                    const double oy = (points[i].y - icy) / ify;
                    //                    const double r = sqrt(ox*ox + oy*oy);
                    //                    const double fac = tan(r * fov) / (r*d2t);
                    //                    f2dCV(i,0) = fac*ox;
                    //                    f2dCV(i,1) = fac*oy; // ofy*fac*oy+ocy;
                    //                }
                    //                std::cout << "RECTIFIED CV:" << std::endl << f2dCV << std::endl;
                    f2d = Map<Matrix<float, Dynamic, 2, RowMajor> >(cv::Mat(points).ptr<float>(),points.size(), 2); //O(1)
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
                }


                // Make homogenious, transpose 3xN
                const MatrixXf f2dh = f2d.transpose().colwise().homogeneous();

                // Project 2d -> 3d rays
                bearings = Pinv.solve(f2dh).cast<double>();
                bearings.colwise().normalize();
                bearings.transposeInPlace(); // Nx3
                pointsRectified = f2d.cast<double>();
            }
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
            ROS_INFO("CAM > SETTING PARAM");

            this->outZoom = zoomFactor;
            interpolation = PTAMRectify;

            if (!USE_SYNTHETIC){

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
            }
            ROS_INFO("CAM < PARAM SET");
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
        double fx,fy,cx,cy,fov; // normalised, ie [0 1]
        double d2t, ifx, ify, icx, icy;

        // Set according to imcoming images
        int inWidth, inHeight;

        /// User changeable variables
        // Rectified image size
        int outWidth, outHeight;

        // Rectified image scale and scale method
        double outZoom;
        ZoomType zoomType;
        // interpolation method (<0 means off)
        int interpolation;
        // Allow resize or not
        bool outSizeIsInSize;


        void initialise(){
            /// Depending on the incoming image size, outgoing image size/zoom, update the camera_info message and then precompute the warping matrices
            ROS_INFO("CAM > INITIALISING NEW CAMERA MATRIX");

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

                ROS_INFO("CAM = Default camera matrix without rectification [%dpx*%dpx (%.3f)]",
                         inWidth, inHeight, static_cast<double>(inWidth)/inHeight);


            } else {
                /// RECIFYING IMAGE

                // Output paramters
                double ofx ,ofy, ocx, ocy;


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
                    const double radL = icx/ifx;
                    const double radR = (inWidth-1 - icx)/ifx;
                    const double radT = icy/ify;
                    const double radB = (inHeight-1 - icy)/ify;

                    // Find radii to each corner distored
                    const double radTL = sqrt(pow(radT, 2) + pow(radL, 2));
                    const double radTR = sqrt(pow(radT, 2) + pow(radR, 2));
                    const double radBL = sqrt(pow(radB, 2) + pow(radL, 2));
                    const double radBR = sqrt(pow(radB, 2) + pow(radR, 2));

                    // Radii to each undistored corner: equation (14) inverse distortion function [see reference at top]
                    double invTL = tan(radTL * fov)/d2t * outZoom;
                    double invTR = tan(radTR * fov)/d2t * outZoom;
                    double invBL = tan(radBL * fov)/d2t * outZoom;
                    double invBR = tan(radBR * fov)/d2t * outZoom;

                    double hor, vert, invHor, invVert;
                    if (zoomType == FULL_MIN){
                        ROS_INFO("CAM = Initialising calibration: Zoom Full Min");
                        // std::maximum distorted
                        hor  =  std::min(std::min(radBR, radTR), std::min(radBL, radTL));
                        vert =  std::min(std::min(radTR, radTL), std::min(radBL, radBR));
                        // std::maximum undistroted
                        invHor  = std::min(std::min(invBR, invTR), std::min(invBL, invTL));
                        invVert = std::min(std::min(invTR, invTL), std::min(invBL, invBR));

                        // Ratios define output
                        ofx = ifx * ((hor ) / (invHor )) * ((double)outWidth /(double)inWidth );
                        ofy = ify * ((vert) / (invVert)) * ((double)outHeight/(double)inHeight);
                        ocx = invHor/hor*ofx/ifx*icx;
                        ocy = invVert/vert*ofy/ify*icy;
                    } else {
                        ROS_INFO("CAM = Initialising calibration: Zoom Full Max");
                        // std::maximum distorted
                        hor  = std::max(radBR, radTR) + std::max(radBL, radTL);
                        vert = std::max(radTR, radTL) + std::max(radBL, radBR);
                        // std::maximum undistroted
                        invHor  = std::max(invBR, invTR) + std::max(invBL, invTL);
                        invVert = std::max(invTR, invTL) + std::max(invBL, invBR);

                        // Ratios define output
                        ofx = ifx * ((hor ) / (invHor )) * ((double)outWidth /(double)inWidth );
                        ofy = ify * ((vert) / (invVert)) * ((double)outHeight/(double)inHeight);
                        ocx = std::max(invBL/radBL, invTL/radTL)*ofx/ifx*icx;
                        ocy = std::max(invTL/radTL, invTR/radTR)*ofy/ify*icy;
                    }



                } else if (zoomType == CROP){
                    ROS_INFO("CAM = Initialising calibration: Zoom Crop");
                    /// Zoom, pan and stretch so the whole output image is valid. Might crop sides of image depending on distortion
                    // find left-most and right-most radius
                    double radL = (icx)/ifx;
                    double radR = (inWidth-1 - icx)/ifx;
                    double radT = (icy)/ify;
                    double radB = (inHeight-1 - icy)/ify;

                    // Undisorted radii to each side
                    double invL = tan(radL * fov)/d2t* outZoom;
                    double invR = tan(radR * fov)/d2t* outZoom;
                    double invT = tan(radT * fov)/d2t* outZoom;
                    double invB = tan(radB * fov)/d2t* outZoom;

                    // Ratios define output
                    ofx = ifx * ((radL + radR) / (invL + invR)) * ((double)outWidth /(double)inWidth);
                    ofy = ify * ((radT + radB) / (invT + invB)) * ((double)outHeight/(double)inHeight);
                    ocx = (invL/radL)*ofx/ifx*icx;
                    ocy = (invT/radT)*ofy/ify*icy;


                } else { //if (zoomType == MANUAL){
                    /// Just manual scale, no panning
                    ROS_INFO("CAM = Initialising calibration: Manual Zoom: %.2f", outZoom);
                    ofx = fx * outWidth  * outZoom;
                    ofy = fy * outHeight * outZoom;
                    ocx = cx * outWidth  - 0.5;
                    ocy = cy * outHeight - 0.5;
                }


                ROS_INFO("CAM = Computing warp matrix");
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

//                //calculate angle and magnitude
//                cv::Mat magnitude, angle;
//                cv::cartToPolar(rect_mapx, rect_mapy, magnitude, angle, true);
//                //translate magnitude to range [0;1]
//                double mag_max=60;
//                cv::minMaxLoc(magnitude, 0, &mag_max);
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
//                cv::imshow("camera refectification", bgr); cv::waitKey(1000);






                //            rect_mapInvX = cv::Mat(inHeight, inWidth, CV_32FC1, cv::Scalar(0));
                //            rect_mapInvY = cv::Mat(inHeight, inWidth, CV_32FC1, cv::Scalar(0));
                //            for (int x=0; x<inWidth; ++x){
                //                for (int y=0; y<inHeight; ++y){
                //                    const double ox = (x - icx) / ifx;
                //                    const double oy = (y - icy) / ify;
                //                    const double r = sqrt(ox*ox + oy*oy);
                //                    const double fac = tan(r * fov)/(2*r*tan(fov/2));
                //                    rect_mapInvX.at<double>(y,x) = ofx*fac*ox+ocx;
                //                    rect_mapInvY.at<double>(y,x) = ofy*fac*oy+ocy;
                //                }
                //            }

                ROS_INFO("CAM = Computing camera matrix and its inverse");
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

                ROS_INFO("CAM = Input [%dpx*%dpx (%.3f)] Output: [%dpx*%dpx (%.3f)]",
                         inWidth, inHeight, static_cast<double>(inWidth)/inHeight,
                         outWidth, outHeight, static_cast<double>(outWidth)/outHeight);
                ROS_INFO("CAM < INITIALISED NEW MODEL:  Focal: [%.1f, %.1f] Center: [%.1f, %.1f]", ofx, ofy, ocx, ocy);

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
