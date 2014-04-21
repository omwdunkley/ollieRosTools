#include <ollieRosTools/Landmark.hpp>

typedef cv::Ptr<Landmark> LandmarkPtr;
typedef std::deque<LandmarkPtr> LandMarkPtrs;

int Landmark::pIdCounter = 0;
double Landmark::angleConeThresh = OVO::px2error(4); // default error within +- 4px
double Landmark::angleFOVThresh  = OVO::px2error(4); // default error within +- 4px
double Landmark::distThreshRatio = 0.3; // default distance error within +-30% tolerance of original distance allowed
double Landmark::distThresh      = 0.2; // 20cm tolerance allowed


bool noRef(const LandmarkPtr& p){
    return !(*(p.refcount)>1);
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
