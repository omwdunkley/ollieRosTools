#ifndef LANDMARK_HPP
#define LANDMARK_HPP

#include <deque>

#include <Eigen/StdVector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/unordered_map.hpp>


#include <ollieRosTools/aux.hpp>
//#include <ollieRosTools/Frame.hpp>
class Frame;
typedef cv::Ptr<Frame> FramePtr;
typedef std::deque<cv::Ptr<Frame> > FramePtrs;







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

    typedef cv::Ptr<Landmark> Ptr;
    typedef std::deque<Landmark::Ptr> Ptrs;
    typedef boost::unordered_map<int,Landmark::Ptr> IntMap;

    // create a new point
    Landmark(const Eigen::Vector3d& point){
        xyz = point;
        id = ++pIdCounter;
        currentObs = -1;
    }

    Landmark(){
        // Not instanciated!
        id = -1;
    }

    inline bool instanciated(){
        return id>=0;
    }

    virtual ~Landmark(){
        if (id>=0){
            //ROS_INFO(OVO::colorise("LMK = Destroying Landmark [%d]:",OVO::FG_DGRAY).c_str(),getId());
        }
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
    void removeObservation(const int fid, const int kfid);

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
        return seenFrom.size();
    }

    // get the frame at observation i
    const FramePtr getObservationFrame(const int i=-1) const {
        const int j = i<0 ? currentObs:i;
        ROS_ASSERT(j>=0 && j < static_cast<int>(seenFrom.size()));
        return seenFrom[j];
    }

    // Returns all frames that saw this point
    const FramePtrs& getObservationIds() const {
        return seenFrom;
    }

    // get the bearing vector to this point from observation i
    const Bearing getObservationBearing(const int i=-1) const;

    // return descriptor of observation i
    const cv::Mat getObservationDesc(const int i=-1) const;

    // return all descriptors
    const cv::Mat getObservationDescs() const;

    // return the position this point is at
    const Point3d& getPosition() const {
        return xyz;
    }

    // add an observation. Seen by frame f with f.keypoint[i]
    void addObservation(const FramePtr f, const int id){
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
    bool visibleFrom(const FramePtr f) ;


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
    friend std::ostream& operator<< (std::ostream& stream, const Landmark& lm);

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

bool noRef(const Landmark::Ptr& p);


inline bool operator==(const Landmark::Ptr& lhs, const Landmark::Ptr& rhs){return lhs->getId() == rhs->getId();}
inline bool operator!=(const Landmark::Ptr& lhs, const Landmark::Ptr& rhs){return !operator==(lhs,rhs);}
inline bool operator< (const Landmark::Ptr& lhs, const Landmark::Ptr& rhs){return lhs->getId() < rhs->getId();}
inline bool operator> (const Landmark::Ptr& lhs, const Landmark::Ptr& rhs){return  operator< (rhs,lhs);}
inline bool operator<=(const Landmark::Ptr& lhs, const Landmark::Ptr& rhs){return !operator> (lhs,rhs);}
inline bool operator>=(const Landmark::Ptr& lhs, const Landmark::Ptr& rhs){return !operator< (lhs,rhs);}



namespace OVO {
    void landmarks2points(const Landmark::IntMap& lms, Points3d& points, const Ints& ind=Ints());
    void landmarks2points(const Landmark::Ptrs& lms, Points3d& points, const Ints& ind=Ints());
}





#endif // LANDMARK_HPP
