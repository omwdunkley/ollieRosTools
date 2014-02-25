#ifndef CAMERAMODEL_HPP
#define CAMERAMODEL_HPP

#include <ollie_vo/Aux.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <fstream>





/// GENERAL ///////////////////////////////////////////////////////////////////////////////////

class CameraModel
{
public:
    CameraModel(){
        initialised = false;
    }

    // Returns 3d unit sphere features from unrectified keypoints
    virtual void getBearingVectors(const KeyPoints&in_unrectified, Points3d& out) const = 0;
    virtual void getBearingVectorsRect(const KeyPoints&in_rectified, Points3d& out) const = 0;
    virtual cv::Mat getRectifiedImage(const cv::Mat& img_in) const = 0;
    cv::Point2d getCenter() const {return center;}
    void setInterpolation(const int interpolation_id){interpolation = interpolation_id;}

    // REMOVE THIS AND DO IT PROPERLY    
    virtual void initialise(const image_geometry::PinholeCameraModel& pinhole_model) = 0;

    bool isInit() const {return initialised;}

protected:
    cv::Point2d center;
    int interpolation;
    int width, height;
    float scale;
    bool initialised;
};


/// PINHOLE ////////////////////////////////////////////////////////////////////////////////////

class CameraPinHole: public CameraModel
{
public:
    CameraPinHole(){
        ROS_INFO("Using cameraInfo message as calibration source");
    }

    void initialise(const image_geometry::PinholeCameraModel& pinhole_model){        
        model = pinhole_model;
        center.x=model.cx();
        center.y=model.cy();
        initialised = true;
        ROS_INFO("PINHOLE MODEL:  SIZE: [%3.1f, %3.1f] CENTER: [%2.1f, %2.1f]", model.fx(), model.fy(), model.cx(), model.cy());
    }

    /// Get bearing vectors from UNrectified image points
    void getBearingVectors(const KeyPoints& in_unrectified, Points3d& out) const{
        out.clear();
        out.reserve(in_unrectified.size());
        cv::Point3d pt3d;

        for (uint i=0; i<in_unrectified.size(); ++i){
            cv::Point2d pt2d = model.rectifyPoint(in_unrectified[i].pt);


            pt3d = model.projectPixelTo3dRay(pt2d);


//            ROS_INFO("%d   [%.2f %.2f] -> [%.2f %.2f] ->  |[%.2f, %.2f, %.2f]| = %.3f",
//                     i,
//                     in_unrectified[i].pt.x, in_unrectified[i].pt.y,
//                     pt2d.x, pt2d.y,
//                     pt3d.x, pt3d.y, pt3d.z, cv::norm(pt3d));

            out.push_back(pt3d * (1./cv::norm(pt3d)));


        }
    }

    /// Get bearing vectors from rectified image points
    void getBearingVectorsRect(const KeyPoints& in_rectified, Points3d& out) const{
        out.clear();
        out.reserve(in_rectified.size());
        cv::Point3d pt3d;
        for (uint i=0; i<in_rectified.size(); ++i){
            pt3d = model.projectPixelTo3dRay(in_rectified[i].pt);
            out.push_back(pt3d  * (1./ cv::norm(pt3d)));
        }
    }


    cv::Mat getRectifiedImage(const cv::Mat& img_in) const{
        cv::Mat img_rectified;
        model.rectifyImage(img_in, img_rectified, interpolation);
        return img_rectified;
    }



private:
   image_geometry::PinholeCameraModel model;
};





#endif // CAMERAMODEL_HPP
