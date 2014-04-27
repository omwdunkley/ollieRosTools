#include <ollieRosTools/Landmark.hpp>
#include <ollieRosTools/Frame.hpp>
#include <boost/unordered_map.hpp>

int Landmark::pIdCounter = 0;
/// TODO: should be in dynamic reconfigure
double Landmark::angleConeThresh = OVO::angle2error(45); // default error within +- 8px
double Landmark::angleFOVThresh  = OVO::angle2error(55); // default error within +- 8px
double Landmark::distThreshRatio = 0.7; // default distance error within +-70% tolerance of original distance allowed
double Landmark::distThresh      = 2.0; // 2m tolerance allowed


uint Landmark::visible     = 0;
uint Landmark::failedDist  = 0;
uint Landmark::failedAngle = 0;
uint Landmark::failedFov   = 0;
uint Landmark::totalObs    = 0;

bool noRef(const Landmark::Ptr& p){
    return p->getObservationsNr()==0;
}


// Converts landmarks to vector of eigen points
void OVO::landmarks2points(const Landmark::IntMap& lms, Points3d& points, const Ints& ind){
    points.clear();
    if (ind.size()>0){
        points.reserve(ind.size());
        for (uint i=0; i<ind.size();++i){
            points.push_back(lms.at(ind[i])->getPosition());
        }
    } else {
        points.reserve(lms.size());
        for (uint i=0; i<ind.size();++i){
            points.push_back(lms.at(i)->getPosition());
        }
    }
}
// Converts landmarks to vector of eigen points
void OVO::landmarks2points(const Landmark::Ptrs& lms, Points3d& points, const Ints& ind){
    points.clear();
    if (ind.size()>0){
        points.reserve(ind.size());
        for (uint i=0; i<ind.size();++i){
            points.push_back(lms[ind[i]]->getPosition());
        }
    } else {
        points.reserve(lms.size());
        for (uint i=0; i<ind.size();++i){
            points.push_back(lms[i]->getPosition());
        }
    }
}


// Remove an observation from frame f
void Landmark::removeObservation(const int fid, const int kfid){
    //ROS_INFO(("LMK = Removing Observation LMK[%3d] <- Frame[%3d|%3d]. Observations left: [" + OVO::colorise("%2d", (seenFrom.size()>2 ? OVO::FG_GREEN : (seenFrom.size()==2 ? OVO::FG_YELLOW:OVO::FG_RED))) +"]").c_str(), getId(), fid,  kfid, static_cast<int>(seenFrom.size())-1);
    // reset current obs, incase it was pointing to the to be deleted frame
    currentObs = -1;
    FramePtrs::iterator it = std::find(seenFrom.begin(), seenFrom.end(), fid);
    ROS_ASSERT(it != seenFrom.end());

    FramePtrs seenFrom2;
    Ints pointIds2;
    for (uint i=0; i<seenFrom.size(); ++i){
        if (seenFrom[i]->getId()!=fid){
            seenFrom2.push_back(seenFrom[i]);
            pointIds2.push_back(pointIds[i]);
        }
    }
    std::swap(seenFrom2, seenFrom);
    std::swap(pointIds2, pointIds);

    // Remove the corresponding FramePtr and pointId

    //seenFrom.erase(it);
    //pointIds.erase(pointIds.begin()+std::distance(seenFrom.begin(), it));
}

// get the bearing vector to this point from observation i
const Bearing Landmark::getObservationBearing(const int i) const {
    ROS_ASSERT(check());
    const int j = i<0 ? currentObs:i;
    ROS_ASSERT(j>=0 && j < static_cast<int>(seenFrom.size()));
    return seenFrom[j]->getBearing(pointIds[j]);
}



// return descriptor of observation i
const cv::Mat Landmark::getObservationDesc(int i) const {
    ROS_ASSERT(check());
    const int j = i<0 ? currentObs:i;
    ROS_ASSERT(j>=0 && j < static_cast<int>(seenFrom.size()));
    return seenFrom[j]->getDescriptor(pointIds[j]);
}

// return all descriptors
const cv::Mat Landmark::getObservationDescs() const {
    ROS_ASSERT(check());
    cv::Mat descs;
    for (uint i=0; i<pointIds.size(); ++i){
        descs.push_back(seenFrom[i]->getDescriptor(pointIds[i]));
    }
    return descs;
}

// returns an id>=0 if the given frame might provide a similar observation from observation i
// Frame must have an estiamted position in its pose member
// Also sets the current observation to the first candidate found (check in reverse chronological order)
bool Landmark::visibleFrom(const FramePtr f) {
    // go in reverese, more likely that newer ones are more visible
    ROS_ASSERT(check());

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

// print id, xyz, nr of ovservations, and observations
std::ostream& operator<< (std::ostream& stream, const Landmark& lm) {
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


//// useful for std::map<Landmark::Ptr, ...> myMap();
//bool operator <(Landmark::Ptr const& lhs, Landmark::Ptr const& rhs){
//    return lhs->getId() < rhs->getId();
//}

