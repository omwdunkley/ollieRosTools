#ifndef MATCHER_HPP
#define MATCHER_HPP
#include <algorithm>    // std::max
#include <ollieRosTools/VoNode_paramsConfig.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <ollieRosTools/aux.hpp>
//#include <ollieRosTools/KeyFrame.hpp>
//#include <ollieRosTools/PointMap.hpp>



// Sort matches by response and take nr best
// NOTE: Usually you wont need this as the matchers sort the output by distance
void match_clip(DMatches& ms, const uint max_nr, bool is_sorted);

// Return the smallest and largest distance along with the total nr of matches
void minMaxTotal(const DMatches& ms, double& minval, double& maxval, uint& total);

// Calculate disparity and filter pixels by disparity if required
DMatches disparityFilter(const DMatches& in, KeyFrame& f1, KeyFrame& f2, const double maxDisparity, bool disparityFilter);

// Filter out matches that are not unique. Specifically unqiue = dist1/dist2 > similarity
void match_filter_unique(const DMatchesKNN& msknn, DMatches& ms, const double similarity, const bool keep_sorted = false);

// Reduces vector of vectors DMatchesKNN to a single vector DMatches
void match_knn2single(const DMatchesKNN& msknn, DMatches& ms, const bool keep_sorted=false);

// remove matches that are ratio * worse than the best
void match_filter_ratio(DMatches& ms, const double ratio, const bool is_sorted=false);

// Remove all matches below threshold.
void match_threshold(DMatches& ms, double thresh, bool is_sorted=false);

// Compute average/median distance of a set of matches
double matchAverageDistance(const DMatches& ms);
double matchMedianDistance(DMatches ms);

// Returns true if match not good enough
bool match_bad(const cv::DMatch& match, const double thresh);
// Returns true if match good enough
bool match_good(const cv::DMatch& match, const double thresh);


class Matcher
{
public:
    Matcher();
    ollieRosTools::VoNode_paramsConfig& setParameter(ollieRosTools::VoNode_paramsConfig &config, uint32_t level);




    // Match frame-frame
    void match(KeyFrame& f1,
               KeyFrame& f2,
               DMatches& matches,                           // matches out
               double& disparity,
               double& time,
               const cv::Mat& mask=cv::Mat(),
               const Ints& maskIdx=Ints()
            );


    // Match frame-map
    void match( KeyFrame& frame,
                PointMap& map,
                DMatchesKNN& matches,
                double& disparity,
                double& time
            );

    double getMaxDisp() const{return m_pxdist;}

private:
    cv::Ptr<cv::DescriptorMatcher> matcher;
    ollieRosTools::VoNode_paramsConfig config_pre;
    void updateMatcher(const int type, const bool update=false);

    int m_type;
    int m_norm;
    int m_sym_neighbours; // 0 = no symmetry
    double m_unique; // 0 = off
    double m_thresh; // 0 = no threshold
    double m_ratio; //1 = off
    int m_max; // 0 = unlimited
    int m_pxdist;


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




