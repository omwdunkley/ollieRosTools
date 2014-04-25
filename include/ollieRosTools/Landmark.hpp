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
    int currentObs; // This can be set to get default observation related stuff. Usually set to which kf saw it last


    /// Visibility settings
    // threshold angle between a keyframe and the direction from which this point was observed in
    static double angleConeThresh;
    static double angleFOVThresh;
    static double distThreshRatio; //ratio from 0,1. Eg 0.3 = error should be within 30% of original distance
    static double distThresh;      //distance error should be within this distance, in meters


    static uint visible;
    static uint failedDist;
    static uint failedAngle;
    static uint failedFov;
    static uint totalObs;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // create a new point
    Landmark(const Eigen::Vector3d& point){
        xyz = point;
        id = ++pIdCounter;
        currentObs = -1;
    }

    virtual ~Landmark(){
        ROS_INFO(OVO::colorise("LMK = Destroying Landmark [%d]:",OVO::FG_DGRAY).c_str(),getId());
        //ROS_INFO_STREAM(*this);
    }

    static void resetStats(){
        visible=0;
        failedDist=0;
        failedAngle=0;
        failedFov=0;
        totalObs=0;
    }

    static void printStats(){
        ROS_INFO("LMK = LM Projection States: \n\t[%4d] Observations\n\t[%4d] Visible\n\t[%4d] Failed Dist\n\t[%4d] Failed Angle\n\t[%4d] Failed FOV", totalObs, visible, failedDist, failedAngle, failedFov);
        resetStats();
    }

    // Remove an observation from frame f
    void removeObservation(const Frame* f){
        const int fid = f->getId();
        const int kfid = f->getKfId();
        ROS_INFO(("LMK = Removing Observation LMK[%3d] <- Frame[%3d|%3d]. Observations left: [" + OVO::colorise("%2d", (seenFrom.size()>2 ? OVO::FG_GREEN : (seenFrom.size()==2 ? OVO::FG_YELLOW:OVO::FG_RED))) +"]").c_str(), getId(), fid,  kfid, static_cast<int>(seenFrom.size())-1);
        // reset current obs, incase it was pointing to the to be deleted frame
        currentObs = -1;
        FramePtrs::iterator it = std::find(seenFrom.begin(), seenFrom.end(), fid);
        ROS_ASSERT(it != seenFrom.end());

        // Remove the corresponding frameptr and pointId
        seenFrom.erase(it);
        pointIds.erase(pointIds.begin()+std::distance(seenFrom.begin(), it));
    }


    // returns the unique landmark ID
    int getId() const {
        return id;
    }

    int getCurrentObs() const {
        return currentObs;
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
    const FramePtr& getObservationFrame(int i=-1) const {
        if (i<0){
            i=currentObs;
        }
        ROS_ASSERT(i>=0 && i < static_cast<int>(seenFrom.size()));
        return seenFrom[i];
    }

    // get the bearing vector to this point from observation i
    const Bearing getObservationBearing(int i=-1) const {
        if (i<0){
            i=currentObs;
        }
        ROS_ASSERT(i>=0 && i < static_cast<int>(seenFrom.size()));
        return seenFrom[i]->getBearing(pointIds[i]);
    }

    // Returns all frames that saw this point
    const FramePtrs& getObservationIds() const {
        return seenFrom;
    }

    // return descriptor of observation i
    const cv::Mat getObservationDesc(int i=-1) const {
        if (i<0){
            i=currentObs;
        }
        ROS_ASSERT(i>=0 && i < static_cast<int>(seenFrom.size()));
        return seenFrom[i]->getDescriptor(pointIds[i]);
    }

    // return all descriptors
    const cv::Mat getObservationDescs() const {
        cv::Mat descs;
        for (uint i=0; i<pointIds.size(); ++i){
            descs.push_back(seenFrom[i]->getDescriptor(pointIds[i]));
        }
        return descs;
    }

    // return the position this point is at
    const Point3d& getPosition() const {
        return xyz;
    }

    // add an observation. Seen by frame f with f.keypoint[i]
    void addObservation(const FramePtr& f, const int id){
        seenFrom.push_back(f);
        pointIds.push_back(id);
        //setCurrentObs(seenFrom.size()-1);
    }

    // Sets the current observation, ie the frame which is currently base suited to see it
    // this is used when maching against the map and we are guessing which observation is the
    // best common one
    void setCurrentObs(const int i){
        ROS_ASSERT(static_cast<int>(seenFrom.size())>i);
        currentObs = i;
    }

    // returns an id>=0 if the given frame might provide a similar observation from observation i
    // Frame must have an estiamted position in its pose member
    // Also sets the current observation to the first candidate found (check in reverse chronological order)
    bool visibleFrom(const FramePtr& f) {
        // go in reverese, more likely that newer ones are more visible

        const Eigen::Vector3d xyz_f  = f->getOpticalCenter(); //position we are observing from
        const Eigen::Vector3d bvFOptAxis = f->getOpticalAxisBearing(); // camera axis we are observing along

        for (int i=static_cast<int>(pointIds.size())-1; i>=0; --i){
        //for (uint i=0; i<pointIds.size(); ++i){
            // Check Distance
            ++totalObs;
            const Eigen::Vector3d xyz_kf = seenFrom[i]->getOpticalCenter();

            Eigen::Vector3d bvKF2P = (xyz-xyz_kf);   // vector: seen from -> this point (normalised later)
            Eigen::Vector3d bvF2P  = (xyz-xyz_f);    // vector: seeing from -> this point (normalised later)
            const double distP2KF  = bvKF2P.norm();  // distance originally seen from
            const double distP2F   = bvF2P.norm();   // distance we might see it from now
            const double dist_diff = abs(distP2KF-distP2F); // difference in distance


            // Check P similar distance to F as from where it was observered
            if (dist_diff<distThreshRatio*distP2KF || dist_diff<distThresh){
                // Check F in FOV of P from which it was seen
                bvKF2P/=distP2KF; //.normalize();
                bvF2P/=distP2F; //.normalize();
                const double angleCone = 1.0 - bvF2P.dot(bvKF2P);
                if (angleCone<angleConeThresh){
                    // check P in FOV of F

                    const double angleFOV = 1.0 - bvF2P.dot(bvFOptAxis);
                    if (angleFOV<angleFOVThresh){
                        ++visible;
                        setCurrentObs(i);
                        return true; //return the first one

                        // Point is in FOV of camera,
                        // being seen from a similar distance,
                        // from a similar side
                    } else {
                        ++failedFov;
                    }
                } else {
                    ++failedAngle;
                }
            } else {
                ++failedDist;
            }
        }

        // Looked at all possible observations, none are similar
        return false;
    }

//    const cv::Mat getClosestDescriptor(){
//        ROS_ASSERT_MSG(false, "LMK = NOT IMPLEMENTED");
//        return cv::Mat();
//    }

    // simply reset the id counter
    static void reset(){
        ROS_INFO("[LMK] = Resetting Landmark IDs");
        pIdCounter = 0;
    }

    // print id, xyz, nr of ovservations, and observations
    friend std::ostream& operator<< (std::ostream& stream, const Landmark& lm) {
        stream << "LMK [ID:" << std::setw(5) << std::setfill(' ') << lm.id << "]"
               /*<< "[XYZ:"
               << std::setw(4) << std::setfill(' ') << std::setprecision(1) << lm.xyz[0] << ","
               << std::setw(4) << std::setfill(' ') << std::setprecision(1) << lm.xyz[1] << ","
               << std::setw(4) << std::setfill(' ') << std::setprecision(1) << lm.xyz[1] << "]"*/
               << "[Obs: " << std::setw(3) << std::setfill(' ') << lm.seenFrom.size() << "] = ";

        for (uint i=0; i<lm.pointIds.size(); ++i){
            if (lm.getCurrentObs()>=0 && i==static_cast<uint>(lm.getCurrentObs())){
                stream << OVO::colorise("(",OVO::FG_GREEN,OVO::BG_DEFAULT, false) << std::setw(4) << std::setfill(' ') << lm.seenFrom[i]->getId() << "|" << std::setw(3) << std::setfill(' ') << lm.seenFrom[i]->getKfId() << OVO::colorise(") ",OVO::FG_GREEN);
            } else {
                stream << "(" << std::setw(4) << std::setfill(' ') << lm.seenFrom[i]->getId() << "|" << std::setw(3) << std::setfill(' ') << lm.seenFrom[i]->getKfId() << ") ";
            }
        }
        return stream;
    }

    geometry_msgs::Point getMarker() const {
        geometry_msgs::Point p;
        tf::pointEigenToMsg(xyz, p);
        return p;
    }

    // Checks if the landmark is in a consistence state
    bool check() const {
        ROS_ASSERT(seenFrom.size()>0);
        ROS_ASSERT(pointIds.size()==seenFrom.size());
        ROS_ASSERT(static_cast<int>(seenFrom.size())>currentObs);
        return true;

    }



};

// useful for using KF in a map
//bool operator <(LandmarkPtr const& lhs, LandmarkPtr const& rhs);

inline bool operator==(const LandmarkPtr& lhs, const LandmarkPtr& rhs){return lhs->getId() == rhs->getId();}
inline bool operator!=(const LandmarkPtr& lhs, const LandmarkPtr& rhs){return !operator==(lhs,rhs);}
inline bool operator< (const LandmarkPtr& lhs, const LandmarkPtr& rhs){return lhs->getId() < rhs->getId();}
inline bool operator> (const LandmarkPtr& lhs, const LandmarkPtr& rhs){return  operator< (rhs,lhs);}
inline bool operator<=(const LandmarkPtr& lhs, const LandmarkPtr& rhs){return !operator> (lhs,rhs);}
inline bool operator>=(const LandmarkPtr& lhs, const LandmarkPtr& rhs){return !operator< (lhs,rhs);}

namespace OVO {
    void landmarks2points(const LandMarkPtrs& lms, Points3d& points, const Ints& ind=Ints());
}




#endif // LANDMARK_HPP
