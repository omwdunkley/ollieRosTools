#ifndef CAMERAATAN_HPP
#define CAMERAATAN_HPP

#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <ollieRosTools/PreProcNode_paramsConfig.h>



class CameraATAN {
    public:
        CameraATAN():
            fx(0.425198),
            fy(0.579108),
            cx(0.477752),
            cy(0.581669),
            s(0.970746),
            inWidth(720),
            inHeight(576),
            outWidth(720),
            outHeight(576),
            interpolation(1)
        {
            initialise();
        }


        void rectify(const cv::Mat& imgIn, cv::Mat& imgOut, sensor_msgs::CameraInfoPtr& camInfo ) const{
            /// Rectifiy image using precomputed matricies and camInfoMessage
            if (interpolation>=0){
                cv::remap(imgIn, imgOut, rect_mapx, rect_mapy, interpolation);
            } else {
                imgIn.copyTo(imgOut);
            }
            *camInfo = infoMsg;

        }


        ollieRosTools::PreProcNode_paramsConfig& setParameter(ollieRosTools::PreProcNode_paramsConfig &config, uint32_t level){
            scale = config.zoom;
            outWidth = config.width;
            outHeight = config.height;
            interpolation = config.PTAMRectify;

            fx = config.fx;
            fy = config.fy;
            cx = config.cx;
            cy = config.cy;
            s = config.s;

            initialise();
            return config;
        }



    private:

        // Warping matricies
        cv::Mat rect_mapx, rect_mapy;

        // Intermediate PTAM params
        double d2t;
        double ifx;
        double ify;
        double icx;
        double icy;        
        sensor_msgs::CameraInfo infoMsg;

        // User changeable variables / PTAM parameters
        float fx,fy,cx,cy,s;


        int inWidth, inHeight;

        // Rectified image size
        int outWidth, outHeight;
        // Rectified image scale
        double scale;
        // interpolation method
        int interpolation;





        void precomputeWarp(){
            /// Precompute warping matrix that rectifies the image
            rect_mapx = cv::Mat(outHeight, outWidth, CV_32FC1, cv::Scalar(0));
            rect_mapy = cv::Mat(outHeight, outWidth, CV_32FC1, cv::Scalar(0));


            // output
            const double ofx = fx * outWidth  * scale;
            const double ofy = fy * outHeight * scale;
            const double ocx = cx * outWidth  - 0.5;	// TODO: -0.5 here or not?
            const double ocy = cy * outHeight - 0.5;

            for (int x=0; x<outWidth; ++x){
                for (int y=0; y<outHeight; ++y){
                    const float ix = (x - ocx) / ofx;
                    const float iy = (y - ocy) / ofy;
                    const float r = sqrt(ix*ix + iy*iy);
                    const float fac = r==0?1:atan(r * d2t)/(s*r);
                    rect_mapx.at<float>(y,x) = ifx*fac*ix+icx; //TODO check y,x instead of x,y
                    rect_mapy.at<float>(y,x) = ify*fac*iy+icy;
                }
            }
        }



        void initialise(){
            /// Using the supplied PTAM calibration parameters, precompute the rectification
            /// warps and build a camera_info msg

            ROS_INFO("Initialising calibration");

//            outWidth = imageWidth;
//            outHeight = imageHeight;

            d2t = 2.0f * tan(s / 2.0f);
            ifx = fx * inWidth ;
            ify = fy * inHeight;
            icx = cx * inWidth - 0.5; //TODO: need -0.5?
            icy = cy * inHeight - 0.5;

            infoMsg.width = inWidth;
            infoMsg.height = inHeight;
            infoMsg.distortion_model = "plumb_bob";

            infoMsg.R.at(0) = 1;
            infoMsg.R.at(4) = 1;
            infoMsg.R.at(8) = 1;

            infoMsg.K.at(0) = ifx;
            infoMsg.K.at(4) = ify;
            infoMsg.K.at(8) = 1;
            infoMsg.K.at(2) = icx;
            infoMsg.K.at(5) = icy;

            infoMsg.P.at(0) = ifx;
            infoMsg.P.at(5) = ify;
            infoMsg.P.at(10) = 1;
            infoMsg.P.at(2) = icx;
            infoMsg.P.at(6) = icy;

            infoMsg.D.clear();
            infoMsg.D.push_back(s);
            infoMsg.D.push_back(0);
            infoMsg.D.push_back(0);
            infoMsg.D.push_back(0);
            infoMsg.D.push_back(0);

            precomputeWarp();

            ROS_INFO("ATAN MODEL:  SIZE: [%d, %d] CENTER: [%.1f, %.1f]", inWidth, inHeight, icx, icy);
            ROS_INFO("  --> Uses PinHole Model: SIZE: [%.1f, %.1f]", ifx, ify);

        }



};

#endif // CAMERAATAN_HPP
