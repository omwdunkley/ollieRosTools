#ifndef MATCHER_HPP
#define MATCHER_HPP

#include <algorithm>    // std::max
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>

#include <ollieRosTools/VoNode_paramsConfig.h>
#include <ollieRosTools/Frame.hpp>
#include <ollieRosTools/Landmark.hpp>
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
cv::Mat makeMask(const int qSize, const int tSize, const Ints& queryOk=Ints(), const Ints& trainOk=Ints(), const bool pseudoInverse=false);

// Makes a mask that prefilters potential matches by using a predicted position - image plane version
//cv::Mat makeDisparityMask(int qSize, int tSize, const Points2f& queryPoints, const Points2f& trainPoints, const double maxDisparity, const Ints& queryOk=Ints(), const Ints& trainOk=Ints());

// Makes a mask that prefilters potential matches by using a predicted bearing vector
//cv::Mat makeDisparityMask(int qSize, int tSize, const Bearings& queryBearings, const Bearings& trainBearings, const double maxBVError, const OVO::BEARING_ERROR methodR = OVO::BVERR_DEFAULT, const Ints& queryOk=Ints(), const Ints& trainOk=Ints());

// Makes a mask that prefilters potential matches by using a predicted bearing vector - optimise eigen version that only works with BVERR_OneMinusAdotB
cv::Mat makeDisparityMask(int qSize, int tSize, const MatrixXd& queryBearings, const MatrixXd& trainBearings, const double maxBVError, const OVO::BEARING_ERROR methodR = OVO::BVERR_DEFAULT, const Ints& queryOk=Ints(), const Ints& trainOk=Ints(), const bool pseudoInverse=false);
//cv::Mat makeDisparityMask(int qSize, int tSize, const Bearings& queryBearings, const Bearings& trainBearings, const double maxBVError, const OVO::BEARING_ERROR methodR = OVO::BVERR_DEFAULT, const Ints& queryOk=Ints(), const Ints& trainOk=Ints());

// Makes a mask where disparity must be in a range. Here the Ints refer to keypoints we should ignore
cv::Mat makeDisparityTriangulationMask(const Ints& f1bad, const Ints& f2bad, const Eigen::MatrixXd& bv1, const Eigen::MatrixXd& bv2, const double minDis=OVO::angle2error(5), const double maxDis=OVO::angle2error(90));






/// MATCHING CLASS

class Matcher{
    private:
        /// ENUMS
        enum Prediction {PRED_BLIND,     // Dont use pose or imu to predict descriptor location
                    PRED_KF,    // Use the KF desc locations (make sure to use a relaxed thresh) 2d->BV vs 2d->BV
                    PRED_KF_IMU, // Predict the KF desc locations by using the imu to obtain a relative rotation. 2d->BV->IMU_BV vs 2d->BV
                    PRED_POSE,       // Use a pose to initialise the desc locations 3d->BV vs 2d->BV
                    PRED_POSE_IMU     // Use a pose and apply the relative rotation to it to predict the desc locations 3d->BV->IMU_BV vs 2d->BV
                   };

        /// Matching
        // Matching with masks and filters on input descriptors
        void match(const cv::Mat& dQuery, const cv::Mat& dTrain, DMatches& matches, double& time, const cv::Mat mask=cv::Mat());

        // Does KLT Refinement over matches. Provide all kps, matches chose subset. Returns matches that passed and updated kps
        void kltRefine(FramePtr fQuery, FramePtr fTrain, DMatches& matches, double& time);

        // Updates the matcher according to the latest settings
        void updateMatcher(const int type, const int size, const bool update=false);


        /// Members
        cv::Ptr<cv::DescriptorMatcher> matcher;
        ollieRosTools::VoNode_paramsConfig config_pre;



        /// Matcher Settings
        int   m_norm;     // L1 or L2, Automatic for binary types
        float m_unique;   // 0 = off
        float m_thresh;   // 0 = no threshold
        uint  m_max;      // 0 = unlimited
        bool  m_doUnique;
        bool  m_doThresh;
        bool  m_doMax;
        bool  m_doSym;
        bool  orb34; // remember if we are using orb WTK 3 or 4
        Prediction m_pred;
        double m_bvDisparityThresh;

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


    public:
        Matcher();

        // Set parameters
        void setParameter(ollieRosTools::VoNode_paramsConfig &config, uint32_t level);

        // Match f against map. Returns angular disparity error
        double matchMap(const cv::Mat& mapD, Landmark::Ptrs& lms, FramePtr f, DMatches& matches, double& time, const Ints& fMask=Ints());

        // Match f against kframe. Returns angular disparity error
        double matchFrame(FramePtr f, FramePtr kf, DMatches& matches, double& time, const Ints& fMask=Ints(), const Ints& kfMask=Ints(), const FramePtr fClose = FramePtr(), bool triangulation=false);





};

#endif // MATCHER_HPP


