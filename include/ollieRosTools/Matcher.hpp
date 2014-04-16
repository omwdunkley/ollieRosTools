#ifndef MATCHER_HPP
#define MATCHER_HPP

#include <algorithm>    // std::max
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>

#include <ollieRosTools/VoNode_paramsConfig.h>
//#include <ollieRosTools/Map.hpp>
class OdoMap;
#include <ollieRosTools/Frame.hpp>
#include <ollieRosTools/aux.hpp>





/// MATCHING UTILITY FUNCTIONS

// Returns true if match not good enough
bool matchBad(const cv::DMatch& match, const float thresh);

// Returns true if match good enough
bool matchGood(const cv::DMatch& match, const float thresh);

// Return the smallest, largest distance along with the total nr of matches
void minMaxTotal(const DMatches& ms, float& minval, float& maxval, uint& total);

// returns true if the distances are sorted by size, lowest first
bool isSorted(const DMatches& ms);

// sorts matches by increaseing distance
void sortMatches(DMatches& ms);

// Reduces vector of vectors DMatchesKNN to a single vector DMatches
void matchKnn2single(const DMatchesKNN& msknn, DMatches& ms, const size_t maxPerMatch);





/// MATCH FILTERING FUNCTIONS

// Filter out matches that are not unique. Specifically unqiue = dist1/dist2 > similarity
void matchFilterUnique(const DMatchesKNN& msknn, DMatches& ms, const float similarity);

// Same as above but preservers length and order. Filters in place
void matchFilterUnique(DMatchesKNN& msknn, const float similarity);

// Keeps the best N matches
void matchFilterBest(DMatches& ms, const uint max_nr);

// Remove all matches below threshold.
void matchFilterThreshold(DMatches& ms, float thresh, bool is_sorted=false);

// remove matches that are ratio * worse than the best
void matchFilterRatio(DMatches& ms, const float ratio, const bool is_sorted=false);

// Only allows matches where matches from ms1 and ms2 match to eachother
void matchSymmetryTest(const DMatchesKNN& ms1,const DMatchesKNN& ms2, DMatches& msSym);




/// MATCH REFINEMENT FUNCTIONS

// Does KLT Refinement over matches. Provide all kps, matches chose subset. Returns matches that passed and updated kps
void kltRefine(const KeyPoints& qKps, const KeyPoints& tKps, DMatches& matches, KeyPoints& qKpsRefined);




/// MASK MAKING FUNCTIONS

// Returns a mask where all intersecionts of rows[queryOk] = 1 and cols[trainOk] = 1 are 1 else 0
cv::Mat makeMask(const int qSize, const int tSize, const Ints& queryOk=Ints(), const Ints& trainOk=Ints());

// Makes a mask that prefilters potential matches by using a predicted position - image plane version
cv::Mat makeDisparityMask(int qSize, int tSize, const Points2f& queryPoints, const Points2f& trainPoints, const float maxDisparity, const Ints& queryOk=Ints(), const Ints& trainOk=Ints());

// Makes a mask that prefilters potential matches by using a predicted bearing vector
cv::Mat makeDisparityMask(int qSize, int tSize, const Bearings& queryPoints, const Bearings& trainPoints, const float maxBVError, const OVO::BEARING_ERROR methodR = OVO::BVERR_DEFAULT, const Ints& queryOk=Ints(), const Ints& trainOk=Ints());






/// MATCHING CLASS

class Matcher
{
public:
    Matcher();
    void setParameter(ollieRosTools::VoNode_paramsConfig &config, uint32_t level);

    // Match f against map with with an initial pose estimate
    void matchMap(const OdoMap& map, FramePtr& f, FramePtr& f_close, const Ints& fMask=Ints());

    // Match f against map withOUT pose estimate = WE ARE LOST
    void matchMap(const OdoMap& map, FramePtr& f, const Ints& fMask=Ints());

    // Match f against frame with an initial pose estimate
    void matchFrame(FramePtr& kf, FramePtr& f, FramePtr& f_close, const Ints& kfMask=Ints(), const Ints& fMask=Ints());

    // Match f against frame withOUT pose estimate = WE ARE LOST
    void matchFrame(FramePtr& kf, FramePtr& f, const Ints& kfMask=Ints(), const Ints& fMask=Ints());



private:
    cv::Ptr<cv::DescriptorMatcher> matcher;
    ollieRosTools::VoNode_paramsConfig config_pre;
    void updateMatcher(const int type, const int size, const bool update=false);

    // Matching with masks and filters on input descriptors
    void match(const cv::Mat& dQuery, const cv::Mat& dTrain, DMatches& matches, double& time, const cv::Mat mask=cv::Mat());

    // Does KLT Refinement over matches. Provide all kps, matches chose subset. Returns matches that passed and updated kps
    void kltRefine(FramePtr& fQuery, FramePtr& fTrain, DMatches& matches);



    /// Matcher Settings
    int   m_norm;     // L1 or L2, Automatic for binary types
    float m_unique;   // 0 = off
    float m_thresh;   // 0 = no threshold
    uint   m_max;      // 0 = unlimited
    bool  m_doUnique;
    bool  m_doThresh;
    bool  m_doMax;
    bool  m_doSym;
    bool  orb34; // remember if we are using orb WTK 3 or 4

    /// KLT Settings
    cv::Point klt_window;
    int klt_levels;
    cv::TermCriteria klt_criteria;
    int klt_flags;
    double klt_eigenThresh;
    bool klt_refine; // used to refine descriptor matches. Similar to corner refinement

    /// Cached
    int descType; // descriptor type (eg binary vs float)
    int descSize; // descriptor size (eg 64, 448, 256, etc)

};

#endif // MATCHER_HPP


