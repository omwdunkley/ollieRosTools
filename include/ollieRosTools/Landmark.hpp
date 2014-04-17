#ifndef LANDMARK_HPP
#define LANDMARK_HPP

#include <deque>

#include <Eigen/StdVector>
#include <Eigen/Core>
#include <Eigen/Geometry>


#include <ollieRosTools/aux.hpp>
#include <ollieRosTools/Frame.hpp>






bool noRef(const LandmarkPtr& p);

class Landmark {
private:
    /// Containers
    // XYZ position in world frame
    Point3d xyz;
    // Frame that observed this landmark
    FramePtrs seenFrom;
    // Descriptor / Keypoint / image point Id. id = descriptor row of seenFrom
    Ints pointIds;


    /// Meta
    static int pIdCounter;
    int id;


    /// Visibility settings
    // threshold angle between a keyframe and the direction from which this point was observed in
    static double angleConeThresh;
    static double angleFOVThresh;
    static double distThreshRatio; //ratio from 0,1. Eg 0.3 = error should be within 30% of original distance
    static double distThresh;      //distance error should be within this distance, in meters

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // create a new point
    Landmark(const Eigen::Vector3d& point){
        xyz = point;
        id = ++pIdCounter;
    }

    // returns the unique landmark ID
    int getId() const {
        return id;
    }

    void setPosition(const Point3d& pos){
        xyz = pos;
    }

    // Returns the number of observations, ie the nr of frames that view this point
    uint getObservationsNr() const {
        ROS_ASSERT(check());
        return seenFrom.size();
    }

    // get the frame at observation i
    const FramePtr& getObservationFrame(const int i) const {
        ROS_ASSERT(i>0 && i<static_cast<int>(seenFrom.size()));
        return seenFrom[i];
    }

    // get the bearing vector to this point from observation i
    const Bearing getObservationBearing(const int i) const {
        ROS_ASSERT(i>0 && i<<static_cast<int>(seenFrom.size()));
        return seenFrom[i]->getBearing(pointIds[i]);
    }

    // Returns all frames that saw this point
    const FramePtrs& getObservationIds() const {
        return seenFrom;
    }

    // return descriptor of observation i
    const cv::Mat getObservationDesc(const int i) const{
        ROS_ASSERT(i>0 && i<<static_cast<int>(seenFrom.size()));
        return seenFrom[i]->getDescriptor(pointIds[i]);
    }

    // return all descriptors
    const cv::Mat getObservationDescs() const{
        cv::Mat descs;
        for (uint i=0; i<pointIds.size(); ++i){
            descs.push_back(seenFrom[i]->getDescriptor(pointIds[i]));
        }
        return descs;
    }

    // return the position this point is at
    const Point3d& getPosition(){
        return xyz;
    }

    // add an observation. Seen by frame f with f.keypoint[i]
    void addObservation(const FramePtr& f, const int id){
        seenFrom.push_back(f);
        pointIds.push_back(id);
    }

    // returns true if the given frame might provide a similar observation.
    // Frame must have an estiamted position in its pose member
    bool visibleFrom(const FramePtr& f) const{
        for (uint i=0; i<pointIds.size(); ++i){
            // Check Distance
            const Eigen::Vector3d xyz_kf = seenFrom[i]->getOpticalCenter();
            const Eigen::Vector3d xyz_f  = f->getOpticalCenter();
            Eigen::Vector3d bvKF2P = (xyz-xyz_kf);   // normalised later
            Eigen::Vector3d bvF2P  = (xyz-xyz_f);    // normalised later
            const double distP2KF  = bvKF2P.norm();
            const double distP2F   = bvF2P.norm();
            const double dist_diff = abs(distP2KF-distP2F);


            // Check P similar distance to F as from where it was observered
            if (dist_diff<distThreshRatio*distP2KF || dist_diff<distThresh){
                // Check F in FOV of P from which it was seen
                bvKF2P.normalize();
                bvF2P.normalize();
                const double angleCone = 1.0 - bvF2P.dot(bvKF2P);
                if (angleCone<angleConeThresh){
                    // check P in FOV of F
                    const Eigen::Vector3d bvFOptAxis = f->getOpticalAxisBearing();
                    const double angleFOV = 1.0 - bvF2P.dot(bvFOptAxis);
                    if (angleFOV<angleFOVThresh){
                        return true;
                        // Point is in FOV of camera,
                        // being seen from a similar distance,
                        // from a similar side
                    }
                }
            }
        }

        // Looked at all possible observations, none are similar
        return false;
    }

    const cv::Mat getClosestDescriptor(){
        ROS_ASSERT_MSG(false, "[LMK] = NOT IMPLEMENTED");
        return cv::Mat();
    }

    // simply reset the id counter
    static void reset(){
        ROS_INFO("[LMK] = Resetting Landmark IDs");
        pIdCounter = 0;
    }

    // print id, xyz, nr of ovservations, and observations
    friend std::ostream& operator<< (std::ostream& stream, const Landmark& point) {
        stream << "LANDMARK [ID:" << std::setw(5) << std::setfill(' ') << point.id << "]"
               << "[XYZ:"
               << std::setw(4) << std::setfill(' ') << std::setprecision(1) << point.xyz[0] << ","
               << std::setw(4) << std::setfill(' ') << std::setprecision(1) << point.xyz[1] << ","
               << std::setw(4) << std::setfill(' ') << std::setprecision(1) << point.xyz[1] << "]"
               << "[Obs: " << std::setw(3) << std::setfill(' ') << point.seenFrom.size() << " = ";

        for (uint i=0; i<point.pointIds.size(); ++i){
            stream << "(" << std::setw(5) << std::setfill(' ') << point.seenFrom[i]->getId() << "|" << std::setw(3) << std::setfill(' ') << point.seenFrom[i]->getKfId() << ")";
        }

        stream << " ]";
        return stream;
    }

    // Checks if the landmark is in a consistence state
    bool check() const {
        ROS_ASSERT(seenFrom.size()>0);
        ROS_ASSERT(pointIds.size()==seenFrom.size());
        return true;

    }



};

#endif // LANDMARK_HPP
