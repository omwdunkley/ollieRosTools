#include <ollieRosTools/Landmark.hpp>

typedef cv::Ptr<Landmark> LandmarkPtr;
typedef std::deque<LandmarkPtr> LandMarkPtrs;

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

bool noRef(const LandmarkPtr& p){
    return p->getObservationsNr()==0;
}


// Converts landmarks to vector of eigen points
void OVO::landmarks2points(const LandMarkPtrs& lms, Points3d& points, const Ints& ind){
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

//// useful for std::map<LandmarkPtr, ...> myMap();
//bool operator <(LandmarkPtr const& lhs, LandmarkPtr const& rhs){
//    return lhs->getId() < rhs->getId();
//}

