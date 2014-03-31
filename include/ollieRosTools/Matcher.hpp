#ifndef MATCHER_HPP
#define MATCHER_HPP
#include <algorithm>    // std::max
#include <ollieRosTools/VoNode_paramsConfig.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <ollieRosTools/aux.hpp>
#include <ollieRosTools/Frame.hpp>
//#include <ollieRosTools/PointMap.hpp>



// Sort matches by response and take nr best
// NOTE: Usually you wont need this as the matchers sort the output by distance
void matchClip(DMatches& ms, const uint max_nr, bool is_sorted=true);

// Return the smallest and largest distance along with the total nr of matches
void minMaxTotal(const DMatches& ms, float& minval, float& maxval, uint& total);

// Calculate disparity and filter pixels by disparity if required
DMatches disparityFilter(const DMatches& in, FramePtr& f1, FramePtr& f2, const float maxDisparity, bool disparityFilter);

// Filter out matches that are not unique. Specifically unqiue = dist1/dist2 > similarity
void matchFilterUnique(const DMatchesKNN& msknn, DMatches& ms, const float similarity, const bool keep_sorted = false);

// Reduces vector of vectors DMatchesKNN to a single vector DMatches
void matchKnn2single(const DMatchesKNN& msknn, DMatches& ms, const bool keep_sorted=false);

// remove matches that are ratio * worse than the best
void matchFilterRatio(DMatches& ms, const float ratio, const bool is_sorted=false);

// Remove all matches below threshold.
void matchThreshold(DMatches& ms, float thresh, bool is_sorted=false);

// Compute average/median distance of a set of matches
float matchAverageDistance(const DMatches& ms);
float matchMedianDistance(DMatches& ms, const bool is_sorted=false);

// Returns true if match not good enough
bool matchBad(const cv::DMatch& match, const float thresh);

// Returns true if match good enough
bool matchGood(const cv::DMatch& match, const float thresh);


class Matcher
{
public:
    Matcher();
    void setParameter(ollieRosTools::VoNode_paramsConfig &config, uint32_t level);




    // Match frame-frame
    void match(FramePtr& f1,
               FramePtr& f2,
               DMatches& matches,                           // matches out
               double& time,
               const cv::Mat& mask=cv::Mat(),
               const Ints& maskIdx=Ints()
            );


    // Match frame-map
//    void match( FramePtr& frame,
//                PointMap& map,
//                DMatchesKNN& matches,
//                float& disparity,
//                float& time
//            );

    float getMaxDisp() const{return m_pxdist;}

private:
    cv::Ptr<cv::DescriptorMatcher> matcher;
    ollieRosTools::VoNode_paramsConfig config_pre;
    void updateMatcher(const int type, const bool update=false);

    int m_type;
    int m_norm;
    int m_sym_neighbours; // 0 = no symmetry
    float m_unique; // 0 = off
    float m_thresh; // 0 = no threshold
    float m_ratio; //1 = off
    int m_max; // 0 = unlimited
    float m_pxdist;


    bool m_doUnique;
    bool m_doThresh;
    bool m_doRatio;
    bool m_doMax;
    bool m_doSym;
    bool m_doPxdist;

    int descType;

};

#endif // MATCHER_HPP


// Add matching stuff
// disparity
// setParam
// frame-frame
// frame-map




