#include <ollieRosTools/Matcher.hpp>


/// CLASS FUNCTIONS
Matcher::Matcher(){
    m_unique = 0; // 0 = off
    m_thresh = 0; // 0 = no threshold
    m_max = 0; // 0 = unlimited
    descType=-1;
    descSize=-1;
    updateMatcher(CV_8U,205);
    klt_window = cv::Size(15*2+1,15*2+1);
    klt_criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
    klt_levels = 3;
    klt_flags = 0; // cv::OPTFLOW_LK_GET_MIN_EIGENVALS
    klt_eigenThresh=0.0001;
    klt_refine = false;
}

// general matching.
// Input: descriptors,masks,inds.
// Output: matches with applied filters.
void Matcher::match(const cv::Mat& dQuery, const cv::Mat& dTrain, DMatches& matches, double& time, const cv::Mat mask){

    ros::WallTime m0 = ros::WallTime::now();
    matches.clear();

    updateMatcher(dQuery.type(), dQuery.cols, false);



    ROS_ASSERT_MSG(mask.empty() || matcher->isMaskSupported(), "Mask given as input but not supported!!");

    if (dQuery.rows == 0){
        ROS_WARN("MAT = No query descriptors to match against");
        time = 0.0;
        return;
    }
    if (dTrain.rows == 0){
        ROS_WARN("MAT = No train descriptors to match against");
        time = 0.0;
        return;
    }

    int idx = m_doUnique*4 + m_doSym*2+ m_doThresh ;
    // U - unique neighbour test
    // S - sym for flann
    // T - threshold
    // R - ratio test
    DMatchesKNN matchesKNN;
    DMatchesKNN q2t, t2q;
    ROS_INFO("MAT > Matching [%d vs %d] with [Case: %d]", dQuery.rows, dTrain.rows, idx);
    switch(idx){

        case 0: // - - -
            // Best match for each keypoint
            matcher->match(dQuery, dTrain, matches, mask);
            break;
        case 1: // - - T
            // Matches within matching distance, then take X best
            matcher->radiusMatch(dQuery, dTrain, matchesKNN, m_thresh, mask);
            matchKnn2single(matchesKNN, matches, 3); //max 3 matches per query
            break;
        case 2: // - S -
            // Symmetric matches
            // cross check should be enabled, so just use KNN=1
            //matcher->knnMatch(dQuery, dTrain, matchesKNN, 1, mask);
            //matchKnn2single(matchesKNN, matches, 1); //max 3 matches per query
            // above doesnt work with crosscheck due to masks...
            matcher->knnMatch(dQuery,dTrain,q2t, 1, mask); //transpose mask as going the other way
            matcher->knnMatch(dTrain,dQuery,t2q, 1, mask.t()); //transpose mask as going the other way
            matchSymmetryTest(q2t, t2q, matches);
            break;
        case 3: // - S T
            // First do symmetric matching, then remove those above threshold
            //matcher->knnMatch(dQuery, dTrain, matchesKNN, 1, mask);
            //matchKnn2single(matchesKNN, matches, 1);
            matcher->knnMatch(dQuery,dTrain,q2t, 1, mask); //transpose mask as going the other way
            matcher->knnMatch(dTrain,dQuery,t2q, 1, mask.t()); //transpose mask as going the other way
            matchSymmetryTest(q2t, t2q, matches);
            matchFilterThreshold(matches, m_thresh); /// presort??
            break;
        case 4: // U - -
            // simple unique test
            matcher->knnMatch(dQuery, dTrain, matchesKNN, 2, mask);
            matchFilterUnique(matchesKNN, matches, m_unique);
            break;
        case 5: // U - T
            matcher->knnMatch(dQuery, dTrain, matchesKNN, 2, mask);
            matchFilterUnique(matchesKNN, matches, m_unique);
            matchFilterThreshold(matches, m_thresh); /// presort??
            break;
        case 6: // U S -
            matcher->knnMatch(dQuery,dTrain,q2t, 2, mask);
            matcher->knnMatch(dTrain,dQuery,t2q, 2, mask.t()); //transpose mask as going the other way
            matchFilterUnique(q2t, m_unique);
            matchFilterUnique(t2q, m_unique);
            matchSymmetryTest(q2t, t2q, matches);
            break;
        case 7: // U S T
            matcher->knnMatch(dQuery,dTrain,q2t, 2, mask); //transpose mask as going the other way
            matcher->knnMatch(dTrain,dQuery,t2q, 2, mask.t()); //transpose mask as going the other way
            matchFilterUnique(q2t, m_unique);
            matchFilterUnique(t2q, m_unique);
            matchSymmetryTest(q2t, t2q, matches);
            matchFilterThreshold(matches, m_thresh); /// presort??
            break;
        default:
            ROS_ASSERT_MSG(0, "Unknown Matching case %d", idx);
    }
    ROS_INFO("MAT < Matched [%lu]", matches.size());

    if (matches.size()==0){
        time = (ros::WallTime::now()-m0).toSec();
        ROS_WARN("MAT < Matcher returned no matches for case [%d]  [%d vs %d] in [%.1fms]", idx, dQuery.rows, dTrain.rows, time);
        return;
    }


    // At this stage everything should be sorted by matching distance
    if (m_doMax && matches.size()>m_max){
        sortMatches(matches);
        //ROS_ASSERT_MSG(isSorted(matches), "MAT = Expecing matches to be sorted, but they are not");
        matchFilterBest(matches, m_max);
    }



    time = (ros::WallTime::now()-m0).toSec();
    float ratio = static_cast<float>(matches.size()) / std::min(dQuery.rows, dTrain.rows);
    ROS_WARN_COND(ratio<0.1f && matches.size()<100, "MAT = Only matched %.1f%% and <100 Matches accepted", 100.f*ratio);
    ROS_INFO("MAT < Matching finished [%d vs %d] = %.1f%% = %lu matches in [%.1fms]", dQuery.rows, dTrain.rows, 100.f*ratio, matches.size(), time*1000);
}



// Match f against map. F should have a decent post estimate. lms and mapD should be prefiltered based on frame-landmark observability
double Matcher::matchMap(const cv::Mat& mapD, std::vector<LandmarkPtr>& lms, FramePtr& f, DMatches& matches, double& time, const Ints& fMask){
    ROS_ASSERT(f->poseEstimated());
    ros::WallTime t0 = ros::WallTime::now();

    /// Get descriptors, possibly masking out some
    const cv::Mat& qD =  f->getDescriptors();
    Eigen::MatrixXd qBV =  f->getBearings();

    ROS_INFO("MAT [H] > Matching Frame[%d|%d] with [%d] bearing against [%lu] Map points", f->getId(), f->getKfId(), qD.rows, lms.size());


    /// Get map points as bearings in frame f
    Eigen::Affine3d inverseSolution;
    inverseSolution = f->getPose().inverse();
    Eigen::MatrixXd tBV(lms.size(),3);
    for(int i=0; i<tBV.rows(); ++i){
        // change to frame f
        tBV.row(i) = inverseSolution * lms[i]->getPosition();
    }
    // Normalise to unit vectors
    tBV.rowwise().normalize();


    cv::Mat mask = makeDisparityMask(qD.rows, mapD.rows, qBV, tBV, m_bvDisparityThresh, OVO::BVERR_DEFAULT, fMask); /// TODO: should be a different disparity thresh, a much smaller one

    /// Do the actual matching
    match(qD, mapD, matches, time, mask);

    double disparity = -1;
    double disparitySum = 0;
    if (matches.size()>0){
//        /// TODO: implemente klt over image patches
//        // Optional KLT refinement
//        if (klt_refine  ){
//            kltRefine(f, kf, matches, time);
//        }
        // Compute disparity of matches
        Doubles error;
        error.reserve(matches.size());

        for(uint i=0; i<matches.size(); ++i){
            //double d =1.0-(qBV.row(matches[i].queryIdx) * tBV.row(matches[i].trainIdx).transpose());
            double d = OVO::errorNormalisedBV(qBV.block<1,3>(matches[i].queryIdx,0),tBV.block<1,3>(matches[i].trainIdx,0), OVO::BVERR_OneMinusAdotB);
            disparitySum+=d;
            error.push_back(d);
        }

        disparity = OVO::medianApprox<double>(error);
    }
    ROS_INFO(OVO::colorise("MAT [H] < Matched [%lu] matches with [%f][%f] disparity in [%.1fms]", OVO::FG_BLUE).c_str(), matches.size(), disparity, disparitySum/matches.size(),1000.*(ros::WallTime::now()-t0).toSec());
    return disparity;
}


// MATCHING AGAINST KEYFRAME
/// TODO - instead of using kf bearing points, use the maskKF for the descriptors and use corresponding world points for bearings! (dont forget to normalise)
double Matcher::matchFrame(FramePtr& f, FramePtr& kf, DMatches& matches, double& time, const Ints& fMask, const Ints& kfMask, const FramePtr& fClose){
    ROS_INFO("MAT [H] > matchFrame QueryFrame[%d|%d] vs TrainFrame[%d|%d]", f->getId(), f->getKfId(), kf->getId(), kf->getKfId());

    /// Get descriptors, possibly masking out some
    const cv::Mat& qD =  f->getDescriptors();
    const cv::Mat& tD = kf->getDescriptors();
    Eigen::MatrixXd qBV =  f->getBearings();
    const Eigen::MatrixXd& tBV = kf->getBearings();
    cv::Mat mask;

    ros::WallTime t0 = ros::WallTime::now();

    /// Do masking predictions on masked bearing vectors
    if (m_pred==PRED_KF){
        mask = makeDisparityMask(qD.rows, tD.rows, qBV, tBV, m_bvDisparityThresh, OVO::BVERR_DEFAULT, fMask, kfMask);
    } else if (m_pred==PRED_KF_IMU){
        // use the kf-f imu difference to unrotate bearings
        Eigen::Matrix3d relRot;
        OVO::relativeRotation(kf->getImuRotation(), f->getImuRotation(), relRot); // BV_kf = R*BV_f = BV_f'*R'
        qBV *= relRot.transpose();
        mask = makeDisparityMask(qD.rows, tD.rows, qBV, tBV, m_bvDisparityThresh, OVO::BVERR_DEFAULT, fMask, kfMask);
    } else if (m_pred==PRED_POSE) {
        ROS_ASSERT(!fClose.empty());
        ROS_ASSERT(fClose->poseEstimated());
        // Use Rotation from close frame as estimate
        Eigen::Matrix3d relRot = kf->getPose().linear().transpose() * fClose->getPose().linear(); // Rotation difference between KF and Fclose
        qBV *= relRot.transpose();
        mask = makeDisparityMask(qD.rows, tD.rows, qBV,  tBV, m_bvDisparityThresh, OVO::BVERR_DEFAULT, fMask, kfMask);
        ROS_ERROR("NOT TESTED");
    } else if (m_pred==PRED_POSE_IMU) {
        // use the kf->closeFrame transformation + closeFrame-f imu difference to unrotate bearings
        ROS_ASSERT(!fClose.empty());
        ROS_ASSERT(fClose->poseEstimated());
        // Fclose -> F via imu
        Eigen::Matrix3d relRot;
        OVO::relativeRotation(fClose->getImuRotation(), f->getImuRotation(), relRot); // BV_kf = R*BV_f = BV_f'*R'
        // KF -> FClose via known Transform
        relRot = relRot*kf->getPose().linear().transpose()*fClose->getPose().linear(); //Apply rotation difference between KF and Fclose
        qBV *= relRot.transpose();
        mask = makeDisparityMask(qD.rows, tD.rows, qBV,  tBV, m_bvDisparityThresh, OVO::BVERR_DEFAULT, fMask, kfMask);
        ROS_ERROR("NOT TESTED");
    } else {
        // default - match everything with everyting
        mask = makeMask(qD.rows, tD.rows, fMask, kfMask);
    }

    /// Do the actual matching
    match(qD, tD, matches, time, mask);

    double disparity = -1;
    double disparitySum = 0;
    if (matches.size()>0){
        // Optional KLT refinement
        if (klt_refine  ){
            kltRefine(f, kf, matches, time);
        }
        // Compute disparity of matches
        Doubles error;
        error.reserve(matches.size());        

        for(uint i=0; i<matches.size(); ++i){
            //double d =1.0-(qBV.row(matches[i].queryIdx) * tBV.row(matches[i].trainIdx).transpose());
            double d = OVO::errorNormalisedBV(qBV.block<1,3>(matches[i].queryIdx,0),tBV.block<1,3>(matches[i].trainIdx,0), OVO::BVERR_OneMinusAdotB);
            disparitySum+=d;
            error.push_back(d);
        }

        disparity = OVO::medianApprox<double>(error);
    }
    ROS_INFO(OVO::colorise("MAT [H] < Matched [%lu] matches with [%f][%f] disparity in [%.1fms]", OVO::FG_BLUE).c_str(), matches.size(), disparity, disparitySum/matches.size(),1000.*(ros::WallTime::now()-t0).toSec());
    return disparity;
}


// Does KLT Refinement over matches. Returns matches that passed. Also updates keypoints of fQuery
void Matcher::kltRefine(FramePtr& fQuery, FramePtr& fTrain, DMatches& matches, double& time){
    ROS_ASSERT(matches.size()>0);
    ROS_INFO("MAT [M] > Doing KLT refinement over [%lu] matches. Window size [%d]", matches.size(), klt_window.x);

    KeyPoints& qKps = fQuery->getKeypoints();
    const KeyPoints& tKps = fTrain->getKeypoints();


    ros::WallTime t0 = ros::WallTime::now();

    /// extract matched keypoint points and align them
    KeyPoints qKpsMatched, tKpsMatched;    OVO::vecAlignMatch<KeyPoints, KeyPoints>(qKps, tKps, qKpsMatched, tKpsMatched, matches);

    /// Convert to points for KLT
    Points2f qPtsPredicted;
    Points2f tPtsMatched;
    cv::KeyPoint::convert(qKpsMatched, qPtsPredicted);
    cv::KeyPoint::convert(tKpsMatched, tPtsMatched);

    UChars klt_status;
    Floats klt_error;
    cv::calcOpticalFlowPyrLK(fTrain->getPyramid(klt_window, klt_levels),  // input
                             fQuery->getPyramid(klt_window, klt_levels),  // input
                             tPtsMatched,                                 // input
                             qPtsPredicted, klt_status, klt_error,        // output row
                             klt_window, klt_levels,  klt_criteria, cv::OPTFLOW_USE_INITIAL_FLOW, klt_eigenThresh); // settings

    // update KeyPoints, update matches
    DMatches filteredMatches;
    filteredMatches.reserve(klt_status.size()); //over estimate
    for (size_t i=0; i<klt_status.size(); ++i){
        if (klt_status[i]){
            filteredMatches.push_back(matches[i]);
            qKps[matches[i].queryIdx].pt = qPtsPredicted[i];
        }
    }
    double t1 = (ros::WallTime::now()-t0).toSec();
    time += t1;
    ROS_INFO("MAT [M] > Finished doing KLT refinement. [%lu/%lu] successful in [%.1fms]", filteredMatches.size(), matches.size(),1000*t1);

}


// Updates the matcher type with new settings
void Matcher::updateMatcher(const int type, const int size, const bool update){
    if(type==descType && size==descSize && !update){
        return;
    }

    descType=type;
    descSize=size;

    //bool sym = m_doSym && !m_doUnique; // can only do KNN=1 matching if cross check is True
    bool sym = false; // For now hardcode off. We will mostly likely use masks and this doesnt allow them
    if (type==CV_8U){ // BINARY DESCRIPTOR
        if (orb34){
            // orb 3,4:
            matcher = new cv::BFMatcher(cv::NORM_HAMMING2, sym); //2bit
            ROS_INFO("MAT [H] = Updated Matcher: BRUTEFORCE [BINARY 2 bit%s]", sym?" Sym":"");
        } else {
            matcher = new cv::BFMatcher(cv::NORM_HAMMING, sym); //1bit
            ROS_INFO("MAT [H] = Updated Matcher: BRUTEFORCE [BINARY 1 bit %s]", sym?" Sym":"");
        }
    } else if (type==CV_32F){ // FLOAT DESCRIPTOR
        matcher = new cv::BFMatcher(m_norm, sym);
        ROS_INFO("MAT [H] = Updated Matcher: BRUTEFORCE [FLOAT L%d %s]", m_norm==2?1:2 , sym?" Sym":"");
    } else {
        ROS_ERROR("MAT [H] = Unknown Desc Type: %d", type);
    }
}


// Set matcher params from dynamic reconfigre. Scales thresholds depending on extracttor
void Matcher::setParameter(ollieRosTools::VoNode_paramsConfig &config, uint32_t level){
    ROS_INFO("MAT [H] > SETTING PARAM");

    //////////////////////////////////////////////////////////////// MATCHER
    //TODO add FLANN vars

    m_unique            = config.match_unique;
    m_norm              = config.match_norm;
    m_thresh            = config.match_thresh;
    m_max               = config.match_max;
    m_bvDisparityThresh = OVO::px2error(config.match_bvDisparityThresh);
    m_pred              = static_cast<Prediction>(config.match_prediction);
    ROS_INFO("MAT [H] = Disparity theshold: %f Pixels = %f Degrees = %f error", config.match_bvDisparityThresh, OVO::px2degrees(config.match_bvDisparityThresh), m_bvDisparityThresh );

    if(!USE_IMU){
        if ( m_pred == PRED_KF_IMU || m_pred == PRED_POSE_IMU)
        m_pred = PRED_BLIND;
        config.match_prediction = m_pred;
        ROS_WARN("MAT [H] = Cannot use IMU Priors for matching, as we are not using an IMU. Turned off");
    }

    // For ease of use later
    m_doUnique = m_unique;
    m_doSym    = config.match_symmetric>0;
    m_doThresh = m_thresh>0;
    m_doMax    = m_max>0;


    orb34 = config.extractor==7 || config.extractor==8;
    updateMatcher(descType, true);

    klt_window      = cv::Size(config.klt_window*2+1, config.klt_window*2+1);
    klt_criteria    = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, config.klt_iter, config.klt_eps);
    klt_levels      = config.klt_levels;
    klt_flags       = config.klt_eigVal ? (cv::OPTFLOW_LK_GET_MIN_EIGENVALS) : 0;
    klt_eigenThresh = config.klt_thresh;
    klt_refine      = config.match_subpix;
    ROS_WARN_COND(klt_refine && config.kp_subpix, "Not recommended to set match_subpix and kp_subpix at the same time");


    // Cache to detect fugure differences
    config_pre = config;


    switch(config.extractor) {
        // (U)SURF 128
        case 0:
        case 2:
            m_thresh*=0.1f;
            break;
        // (U)SURF 64
        case 1:
        case 3:
            m_thresh*=0.1f;
            break;

        // SIFT
        case 4:
        case 5:
            m_thresh*=40.f;
            break;

        // orb 2
        case 6:
            m_thresh*=14.f;
            break;
        // orb 3
        case 7:
            m_thresh*=6.f;
            break;
        // orb 4
        case 8:
            m_thresh*=6.f;
            break;
        // brisk
        case 20:
            m_thresh*=1.f;
            break;

        // Brief 16
        case 200:
        case 203:
            m_thresh*=4.f;
            break;
        // Brief 32
        case 201:
        case 204:
            m_thresh*=7.5f;
            break;
        // Brief 64
        case 202:
        case 205:
            m_thresh*=17.f;
            break;


        // AKAZE 486 3 CH
        case 30: case 40: case 50: case 60:
            m_thresh*=20.f;
            break;
        // AKAZE 486 1 CH
        case 31: case 41: case 51: case 61:
            m_thresh*=7.f;
            break;
        // AKAZE 160 1,3 CH
        case 32: case 42: case 52: case 62:
        case 33: case 43: case 53: case 63:
            m_thresh*=6.f;
            break;
        // AKAZE 64 1,3 CH
        case 34: case 44: case 54: case 64:
        case 35: case 45: case 55: case 65:
            m_thresh*=2.5f;
            break;


        ///FREAK
        case 206:
        case 207:
            m_thresh= 50 + 5*m_thresh;
            break;
        default:
            ROS_WARN("Not normalised matching distance for descriptor [%d]", config.extractor);

    }


    ROS_INFO("MAT [H] < PARAM SET");

}





/// MATCHING UTILITY FUNCTIONS

// Returns true if match not good enough
bool matchBad(const cv::DMatch& match, const float thresh){
    return (match.distance > thresh);
}


// Returns true if match good enough
bool matchGood(const cv::DMatch& match, const float thresh){
    return (match.distance < thresh);
}


// Return the smallest, largest distance along with the total nr of matches
void minMaxTotal(const DMatches& ms, float& minval, float& maxval, uint& total){
    minval = INFINITY;
    maxval = -INFINITY;
    for (uint x=0; x < ms.size(); ++x){
        float r = ms[x].distance;
        if( r < minval ) {
            minval = r;
        }
        if( r > maxval ){
            maxval = r;
        }
    }
    total = ms.size();
}


// returns true if the distances are sorted by size, lowest first
bool isSorted(const DMatches& ms){    
    ROS_INFO("MAT [U] = Checking if matches are sorted");
    for (uint i=1; i<ms.size(); ++i){
        //if (ms[i-1].distance > ms[i].distance){
        if (ms[i]<ms[i-1]){
            return false;
        }
    }
    return true;
}


// sorts matches by increaseing distance
void sortMatches(DMatches& ms){
    std::sort(ms.begin(), ms.end());
}


// Reduces vector of vectors DMatchesKNN to a single vector DMatches
void matchKnn2single(const DMatchesKNN& msknn, DMatches& ms, const size_t maxPerMatch){
    ms.clear();
    ms.reserve(msknn.size()*maxPerMatch); // just guess
    // TO DO some kind of insertion sort could be better?
    for(size_t i=0; i<msknn.size(); ++i) {
        size_t maxpm = maxPerMatch==0 ? msknn[i].size() : std::min(maxPerMatch,msknn[i].size());
        for(size_t j=0; j<maxpm; ++j) {
            // some kind of insertion sort might be better..
            ms.push_back(msknn[i][j]);
        }
    }
}








/// MATCH FILTERING FUNCTIONS

// Filter out matches that are not unique. Specifically unqiue = dist1/dist2 > similarity
void matchFilterUnique(const DMatchesKNN& msknn, DMatches& ms, const float similarity){
    ROS_INFO("MAT [U] > Filtering [%lu] Matches by uniqueness", msknn.size());
    ms.clear();
    ms.reserve(msknn.size());

    for (uint m = 0; m < msknn.size(); ++m){
        // We need at least two so that we can compare
        if (msknn[m].size() < 2){
            continue;
        }
        const cv::DMatch &m1 = msknn[m][0];
        const cv::DMatch &m2 = msknn[m][1];
        if(m1.distance <= similarity * m2.distance) {
            // some kind of insertion sort would be nice
            ms.push_back(m1);
        }
    }
    ROS_INFO("MAT [U] < Matches [%lu/%lu] remaining after uniqueness test", ms.size(), msknn.size());
}


// Same as above but preservers length and order. Filters in place
void matchFilterUnique(DMatchesKNN& msknn, const float similarity){
    ROS_INFO("MAT [U] > Filtering [%lu] Matches by uniqueness in place", msknn.size());
    DMatchesKNN out;
    out.reserve(msknn.size());
    uint counter = 0;
    for (uint m = 0; m < msknn.size(); ++m){
        // We need at least two so that we can compare
        if (msknn[m].size() < 2){
            // not enough matches, nothing to compare, fail, add empty
            out.push_back(DMatches());
            continue;
        }
        const cv::DMatch &m1 = msknn[m][0];
        const cv::DMatch &m2 = msknn[m][1];
        if(m1.distance <= similarity * m2.distance) {
            // pass, add it
            out.push_back(DMatches(1, m1));
            ++counter;
        } else {
            // fail, add empty
            out.push_back(DMatches());
        }
    }
    ROS_INFO("MAT [U] < Matches [%u/%lu] remaining after uniqueness test", counter, msknn.size());
    std::swap(out, msknn);
}


// Keeps the best N matches
void matchFilterBest(DMatches& ms, const uint max_nr){
    ROS_INFO("MAT [U] = Capping top [%u/%lu] matches", max_nr, ms.size());
    ROS_ASSERT(isSorted(ms));
    if (ms.size()>max_nr){
        ms.erase(ms.begin()+MIN(max_nr,ms.size()),ms.end());
    }
}


// Remove all matches below threshold.
void matchFilterThreshold(DMatches& ms, float thresh, bool is_sorted){
    ROS_INFO("MAT [U] > Thresholding [%lu] matches by distance [%f]", ms.size(), thresh);
    uint s = ms.size();
    ROS_ASSERT_MSG(!is_sorted || isSorted(ms), "Expecting matches to be sorted, but they are not");
    if (is_sorted){
        // Logarithmic version
        DMatches::iterator low = std::lower_bound (ms.begin(), ms.end(), thresh, matchGood);
        ms.erase( low, ms.end() );
    } else {
        // O(N)
        ms.erase( std::remove_if( ms.begin(), ms.end(), boost::bind(matchBad, _1, thresh)), ms.end() );
    }

    ROS_INFO("MAT [U] < Thresholded matches by distance [%lu / %u] left", ms.size(), s);
}


// remove matches that are ratio * worse than the best
void matchFilterRatio(DMatches& ms, const float ratio, const bool is_sorted){
    ROS_ASSERT_MSG(!is_sorted || isSorted(ms), "Expecing matches to be sorted, but they are not");


    if (ms.size()>1){
        float best;
        float worst;

        if (is_sorted){
            best = ms[0].distance;
            worst = ms.back().distance;
        } else {
            uint nrd;
            minMaxTotal(ms, best, worst, nrd);
        }

        const double thresh = std::max(1e-8f,ratio*best);
        ROS_INFO("MAT [U] > Filtering [%lu] matches by ratio. Range: [%f - %f], [%f] thresh", ms.size(), best, worst, thresh);

        if (worst>thresh){
            uint s = ms.size();
            matchFilterThreshold(ms, thresh, is_sorted);
            ROS_INFO("MAT [U] < [%lu/%u] left", ms.size(), s);
        } else {
            ROS_INFO("MAT [U] < None to Ratio Filter");
        }

    }
}

// Only allows matches where matches from ms1 and ms2 match to eachother
void matchSymmetryTest(const DMatchesKNN& ms1,const DMatchesKNN& ms2, DMatches& msSym) {
    ROS_INFO("MAT [U] > Symmetrical Check [%lu vs %lu]", ms1.size(), ms2.size());
    msSym.clear();
    int size = std::min(ms1.size(), ms2.size());
    if (size==0){
        ROS_WARN("MAT [U] = NOTHING TO SYMMETRY MATCH");
    } else {
        msSym.reserve(size);
        //loop through smaller one
        if (ms1.size()<=ms2.size()){
            for (int i=0; i<size; ++i){
                const DMatches& m1v = ms1[i];
                if(m1v.size()>0){
                    const DMatches& m2v = ms2[ms1[i][0].trainIdx];
                    if(m2v.size()>0){
                        //ROS_INFO("Q: %d %lu %d %d",i,m1v.size(), mv1[0].queryIdx, m1v[0].trainIdx);
                        //ROS_INFO("T: %d %lu %d %d",i,m2v.size(), m2v[0].queryIdx, mv2[0].trainIdx);
                        if (m1v[0].queryIdx == m2v[0].trainIdx){
                            msSym.push_back(m1v[0]);
                        }
                    }
                }
            }
        } else {
            for (int i=0; i<size; ++i){
                const DMatches& m2v = ms2[i];
                if(m2v.size()>0){
                    const DMatches& m1v = ms1[ms2[i][0].trainIdx];
                    if(m1v.size()>0){
                        if (m2v[0].queryIdx == m1v[0].trainIdx){
                            // using matches going from f1 to f2, so reverse trainIdx and queryidx
                            msSym.push_back(m1v[0]);
                        }
                    }
                }
            }
        }
    }
    ROS_INFO("MAT < Symmetrical Check Done [%lu inliers] of [%lu vs %lu]", msSym.size(), ms1.size(), ms2.size());
}













/// MASK MAKING FUNCTIONS

// Returns a mask where all intersecionts of rows[queryOk] = 1 and cols[trainOk] = 1 are 1 else 0
cv::Mat makeMask(const int qSize, const int tSize, const Ints& queryOk, const Ints& trainOk){
    cv::Mat maskQ = cv::Mat::zeros(qSize, tSize, CV_8U);
    cv::Mat maskT = cv::Mat::zeros(qSize, tSize, CV_8U);
    if (queryOk.empty()){
        maskQ.setTo(1);
        ROS_INFO("MAT [L] = Keeping all [%d] Query Descriptors", qSize);
    } else {
        ROS_INFO("MAT [L] = Masking Query Descriptors");
        for (uint qi=0; qi<queryOk.size(); ++qi){
            maskQ.row(queryOk[qi]).setTo(1);
        }
    }
    if (trainOk.empty()){
        maskT.setTo(1);
        ROS_INFO("MAT [L] = Keeping all [%d] Query Descriptors", tSize);
    } else {
        ROS_INFO("MAT [L] = Masking Train Descriptors");
        for (uint ti=0; ti<trainOk.size(); ++ti){
            maskT.col(trainOk[ti]).setTo(1);
        }
    }
    return maskT & maskQ;
}

/*
// Makes a mask that prefilters potential matches by using a predicted position - image plane version
cv::Mat makeDisparityMask(int qSize, int tSize, const Points2f& queryPoints, const Points2f& trainPoints, const double maxDisparity, const Ints& queryOk, const Ints& trainOk){
    // We do the opposite first and then invert
    ROS_INFO("MAT [U] > Premasking 2d Disparities");
    // kd tree would be nice...worth the overhead?
    ros::WallTime t0 = ros::WallTime::now();

    uint counter=0;
    cv::Mat mask = cv::Mat::zeros(qSize, tSize, CV_8U);


    if (queryOk.empty()){
        if (trainOk.empty()){
            for(uint q=0; q<queryPoints.size(); ++q){
                for(uint t=0; t<trainPoints.size(); ++t){
                    if (cv::norm(queryPoints[q]-trainPoints[t])<maxDisparity){
                        mask.at<uchar>(q,t) = 1;
                        counter++;
                    }
                }
            }
        } else {
            for(uint q=0; q<queryPoints.size(); ++q){
                for(uint t=0; t<trainOk.size(); ++t){
                    if (cv::norm(queryPoints[q]-trainPoints[trainOk[t]])<maxDisparity){
                        mask.at<uchar>(q,trainOk[t]) = 1;
                        counter++;
                    }
                }
            }
        }
    } else {
        if (trainOk.empty()){
            for(uint q=0; q<queryOk.size(); ++q){
                for(uint t=0; t<trainPoints.size(); ++t){
                    if (cv::norm(queryPoints[queryOk[q]]-trainPoints[t])<maxDisparity){
                        mask.at<uchar>(queryOk[q],t) = 1;
                        counter++;
                    }
                }
            }
        } else {
            for(uint q=0; q<queryOk.size(); ++q){
                for(uint t=0; t<trainOk.size(); ++t){
                    if (cv::norm(queryPoints[queryOk[q]]-trainPoints[trainOk[t]])<maxDisparity){
                        mask.at<uchar>(queryOk[q],trainOk[t]) = 1;
                        counter++;
                    }
                }
            }
        }
    }

    ROS_INFO("MAT [U] < Finished premasking, matches reduced by [%.1f%%] in [%.1fms]", 100.f*(1.f-(static_cast<float>(counter)/(qSize*tSize))), (ros::WallTime::now()-t0).toSec()*1000.);
    return mask;
}
*/







/*
// Makes a mask that prefilters potential matches by using a predicted bearing vector
cv::Mat makeDisparityMask(int qSize, int tSize, const Bearings& queryBearings, const Bearings& trainBearings, const double maxBVError, const OVO::BEARING_ERROR method, const Ints& queryOk, const Ints& trainOk){
    // We do the opposite first and then invert
    ROS_INFO("MAT [U] > Premasking 3d Disparities");
    // kd tree would be nice...worth the overhead?
    ros::WallTime t0 = ros::WallTime::now();

    uint counter=0;
    cv::Mat mask = cv::Mat::zeros(qSize, tSize, CV_8U);

    if (queryOk.empty()){
        if (trainOk.empty()){
            for(uint q=0; q<queryBearings.size(); ++q){
                for(uint t=0; t<trainBearings.size(); ++t){
                    if ( OVO::errorBV(queryBearings[q],trainBearings[t], method)<maxBVError){
                        mask.at<uchar>(q,t) = 1;
                        counter++;
                    }
                }
            }
        } else {
            for(uint q=0; q<queryBearings.size(); ++q){
                for(uint t=0; t<trainOk.size(); ++t){
                    if ( OVO::errorBV(queryBearings[q],trainBearings[trainOk[t]], method)<maxBVError){
                        mask.at<uchar>(q,trainOk[t]) = 1;
                        counter++;
                    }
                }
            }
        }
    } else {
        if (trainOk.empty()){
            for(uint q=0; q<queryOk.size(); ++q){
                for(uint t=0; t<trainBearings.size(); ++t){
                    if ( OVO::errorBV(queryBearings[queryOk[q]],trainBearings[t], method)<maxBVError){
                        mask.at<uchar>(queryOk[q],t) = 1;
                        counter++;
                    }
                }
            }
        } else {
            for(uint q=0; q<queryOk.size(); ++q){
                for(uint t=0; t<trainOk.size(); ++t){
                    if ( OVO::errorBV(queryBearings[queryOk[q]],trainBearings[trainOk[t]], method)<maxBVError){
                        mask.at<uchar>(queryOk[q],trainOk[t]) = 1;
                        counter++;
                    }
                }
            }
        }
    }

    ROS_INFO("MAT [U] < Finished premasking [%d/%d], matches reduced by [%.1f%%] in [%.1fms]", counter, tSize*qSize, 100.f*(1.f-(static_cast<float>(counter)/(qSize*tSize))), (ros::WallTime::now()-t0).toSec()*1000.);
    return mask;
}
*/



/*
// Makes a mask that prefilters potential matches by using a predicted bearing vector
cv::Mat makeDisparityMask(int qSize, int tSize, const Bearings& queryBearings, const Bearings& trainBearings, const double maxBVError, const OVO::BEARING_ERROR method, const Ints& queryOk, const Ints& trainOk){
    // We do the opposite first and then invert

    // kd tree would be nice...worth the overhead?
    ros::WallTime t0 = ros::WallTime::now();

    ROS_ASSERT_MSG(method==OVO::BVERR_OneMinusAdotB, "Optimised eigen version only works for method=BVERR_OneMinusAdotB");
    uint counter=0;
    cv::Mat mask = cv::Mat::zeros(qSize, tSize, CV_8U);
    int total;

    if (queryOk.empty()){
        if (trainOk.empty()){
            // loop over all
            total = queryBearings.size()*trainBearings.size();
            ROS_INFO("MAT [U] > Premasking 3d Disparities for [%ld vs %ld] = %.1e combinations [Thresh = %f]",queryBearings.size(), trainBearings.size(), static_cast<double>(total),maxBVError);
            for(uint q=0; q<queryBearings.size(); ++q){
                for(uint t=0; t<trainBearings.size(); ++t){
                    if ( (1.0-(queryBearings[q] * trainBearings[t].transpose())) < maxBVError){
                       mask.at<uchar>(q,t) = 1;
                       ++counter;
                    }
                }
            }
        } else {
            total = queryBearings.size()*trainOk.size();
            ROS_INFO("MAT [U] > Premasking 3d Disparities for [%ld vs %lu] = %.1e combinations [Thresh = %f]",queryBearings.size(), trainOk.size(), static_cast<double>(total),maxBVError  );
            // Compute over full width and use index for height
            for(uint q=0; q<queryBearings.size(); ++q){
                for(uint t=0; t<trainOk.size(); ++t){
                    if ( (1.0-(queryBearings[q] * trainBearings[trainOk[t]].transpose())) < maxBVError){
                       mask.at<uchar>(q,trainOk[t]) = 1;
                       ++counter;
                    }
                }
            }

        }
    } else {

        if (trainOk.empty()){
            total = queryOk.size()*trainBearings.size();
            ROS_INFO("MAT [U] > Premasking 3d Disparities for [%lu vs %ld] = %.1e combinations [Thresh = %f]",queryOk.size(), trainBearings.size(), static_cast<double>(total),maxBVError  );
            // use full height, use indicies for width
            for(uint q=0; q<queryOk.size(); ++q){
                for(uint t=0; t<trainBearings.size(); ++t){
                    if ( (1.0-(queryBearings[queryOk[q]] * trainBearings[t].transpose())) < maxBVError){
                       mask.at<uchar>(queryOk[q],t) = 1;
                       ++counter;
                    }
                }
            }

        } else {
            // Use indicies for both
            total = queryOk.size()*trainOk.size();
            ROS_INFO("MAT [U] > Premasking 3d Disparities for [%lu vs %lu] = %.1e combinations [Thresh = %f]", queryOk.size(), trainOk.size(), static_cast<double>(total),maxBVError  );
            for(uint q=0; q<queryOk.size(); ++q){
                for(uint t=0; t<trainOk.size(); ++t){
                    if ( (1.0-(queryBearings[queryOk[q]] * trainBearings[trainOk[t]].transpose())) < maxBVError){
                       mask.at<uchar>(queryOk[q],trainOk[t]) = 1;
                       ++counter;
                    }
                }
            }
        }
    }

    ROS_INFO("MAT [U] < Marked [%d/%d] matches, reduced by [%.1f%%] in [%.1fms]", counter, total, 100.f*(1.f-(static_cast<float>(counter)/total)), (ros::WallTime::now()-t0).toSec()*1000.);
    return mask;
}
*/


// Makes a mask that prefilters potential matches by using a predicted bearing vector
cv::Mat makeDisparityMask(int qSize, int tSize, const Eigen::MatrixXd& queryBearings, const Eigen::MatrixXd& trainBearings, const double maxBVError, const OVO::BEARING_ERROR method, const Ints& queryOk, const Ints& trainOk){
    // We do the opposite first and then invert

    // kd tree would be nice...worth the overhead?
    ros::WallTime t0 = ros::WallTime::now();

    ROS_ASSERT_MSG(method==OVO::BVERR_OneMinusAdotB, "Optimised eigen version only works for method=BVERR_OneMinusAdotB");
    uint counter=0;
    cv::Mat mask = cv::Mat::zeros(qSize, tSize, CV_8U);
    int total;

    if (queryOk.empty()){
        if (trainOk.empty()){
            // loop over all
            total = queryBearings.rows()*trainBearings.rows();
            ROS_INFO("MAT [U] > Premasking 3d Disparities for [%ld vs %ld] = %.1e combinations [Thresh = %f]",queryBearings.rows(), trainBearings.rows(), static_cast<double>(total),maxBVError);
            for(uint q=0; q<queryBearings.rows(); ++q){
                for(uint t=0; t<trainBearings.rows(); ++t){
                    if ( (1.0-(queryBearings.row(q) * trainBearings.row(t).transpose())) < maxBVError){
                       mask.at<uchar>(q,t) = 1;
                       ++counter;
                    }
                }
            }
        } else {
            total = queryBearings.rows()*trainOk.size();
            ROS_INFO("MAT [U] > Premasking 3d Disparities for [%ld vs %lu] = %.1e combinations [Thresh = %f]",queryBearings.rows(), trainOk.size(), static_cast<double>(total),maxBVError  );
            // Compute over full width and use index for height
            for(uint q=0; q<queryBearings.rows(); ++q){
                for(uint t=0; t<trainOk.size(); ++t){
                    if ( (1.0-(queryBearings.row(q) * trainBearings.row(trainOk[t]).transpose())) < maxBVError){
                       mask.at<uchar>(q,trainOk[t]) = 1;
                       ++counter;
                    }
                }
            }

        }
    } else {

        if (trainOk.empty()){
            total = queryOk.size()*trainBearings.rows();
            ROS_INFO("MAT [U] > Premasking 3d Disparities for [%lu vs %ld] = %.1e combinations [Thresh = %f]",queryOk.size(), trainBearings.rows(), static_cast<double>(total),maxBVError  );
            // use full height, use indicies for width
            for(uint q=0; q<queryOk.size(); ++q){
                for(uint t=0; t<trainBearings.rows(); ++t){
                    if ( (1.0-(queryBearings.row(queryOk[q]) * trainBearings.row(t).transpose())) < maxBVError){
                       mask.at<uchar>(queryOk[q],t) = 1;
                       ++counter;
                    }
                }
            }

        } else {
            // Use indicies for both
            total = queryOk.size()*trainOk.size();
            ROS_INFO("MAT [U] > Premasking 3d Disparities for [%lu vs %lu] = %.1e combinations [Thresh = %f]", queryOk.size(), trainOk.size(), static_cast<double>(total),maxBVError  );
            for(uint q=0; q<queryOk.size(); ++q){
                for(uint t=0; t<trainOk.size(); ++t){
                    if ( (1.0-(queryBearings.row(queryOk[q]) * trainBearings.row(trainOk[t]).transpose())) < maxBVError){
                       mask.at<uchar>(queryOk[q],trainOk[t]) = 1;
                       ++counter;
                    }
                }
            }
        }
    }

    ROS_INFO("MAT [U] < Marked [%d/%d] matches, reduced by [%.1f%%] in [%.1fms]", counter, total, 100.f*(1.f-(static_cast<float>(counter)/total)), (ros::WallTime::now()-t0).toSec()*1000.);
    return mask;
}











/// OLD FUNCTIONS


//// cant use const here
//// will sort if is not sorted
//float matchMedianDistance(DMatches& ms, const bool is_sorted){
//    float median=0;
//    const size_t size = ms.size();
//    if (size>0){
//        if (!is_sorted){
//            sort(ms.begin(), ms.end());
//        }
//        if (size  %2 == 0)  {
//            median = (ms[size/2 -1].distance + ms[size / 2].distance) / 2;
//        }  else   {
//            median = ms[size/2].distance;
//        }
//    }
//    return median;
//}


//float matchAverageDistance(const DMatches& ms){
//  float s = 0;
//  const uint size = ms.size();
//  for (uint i=0; i<size; ++i){
//      s+=ms[i].distance;
//  }
//  return s/size;
//}


/*
float rotatedDisparity(FramePtr& f1, FramePtr& f2, const DMatches& ms, bool median=true ){
    if (!median){
        /// AVERAGE
        ROS_INFO("MAT > Computing [average] rotated disparity of [%lu] matches between Frames [%d|%d] vs [%d|%d]", ms.size(), f1->getId(), f1->getKfId(), f2->getId(), f2->getKfId());
        const uint size = ms.size();
        if (size==0){
            return 0.f;
        }
        const KeyPoints& p1 = f1->getRotatedKeypoints();
        const KeyPoints& p2 = f2->getRotatedKeypoints();
        float s = 0;
        for (uint i=0; i<ms.size(); ++i){
            s+=cv::norm( p1[ms[i].queryIdx].pt - p2[ms[i].trainIdx].pt );
        }
        float disparity = s/size;
        ROS_INFO("MAT < Average disparity = %.1f", disparity );
        return disparity;

    } else {
        /// MEDIAN
        ROS_INFO("FTR > Computing [median] rotated disparity of [%lu] matches between Frames [%d|%d] vs [%d|%d]", ms.size(), f1->getId(), f1->getKfId(), f2->getId(), f2->getKfId());
        if (ms.size()==0){
            return 0;
        }
        const KeyPoints& p1 = f1->getRotatedKeypoints();
        const KeyPoints& p2 = f2->getRotatedKeypoints();
        Floats disp;
        disp.reserve(ms.size());
        for (uint i=0; i<ms.size(); ++i){
            disp.push_back(cv::norm( p1[ms[i].queryIdx].pt - p2[ms[i].trainIdx].pt ));
        }
        uint middle = disp.size() / 2;
        nth_element(disp.begin(), disp.begin()+middle, disp.end());

        float disparity = disp[middle];
        ROS_INFO("MAT < Median disparity = %.1f", disparity );
        return disparity;
    }
}
*/






// Preserves sorting
// changes distance member of DMatch to disparity
/*
DMatches disparityFilter(const DMatches& in, FramePtr& fQuery, FramePtr& fTrain, const float maxDisparity, bool disparityFilterOn){
    if (disparityFilterOn){
        ROS_INFO("MAT > Disparity filtering [%lu] matches, disparity threshold [%.1f]", in.size(), maxDisparity);
    } else {
        ROS_INFO("MAT = Computing disparity for [%lu] matches", in.size());
    }

//    const cv::Size& size= fQuery->getImage().size();
//    cv::Mat img = cv::Mat::zeros(size,CV_8UC3);
    DMatches outMatches;

    const KeyPoints& kp2 = fTrain->getRotatedKeypoints();
    const KeyPoints& kp1 = fQuery->getRotatedKeypoints();


//    cv::Mat img1 = cv::Mat::zeros(size,CV_8UC3);
//    cv::Mat img2 = cv::Mat::zeros(size,CV_8UC3);
//    cv::drawKeypoints(img1,f1.getKPs(),img1,CV_RGB(255,0,0));
//    cv::drawKeypoints(img1,f1.getKPsUnRot(),img1,CV_RGB(0,0,255));
//    cv::drawKeypoints(img2,f2.getKPs(),img2,CV_RGB(255,0,0));
//    cv::drawKeypoints(img2,f2.getKPsUnRot(),img2,CV_RGB(0,0,255));

//    cv::imshow("img1",img1);
//    cv::imshow("img2",img2);


    outMatches.reserve(in.size());


    //float sum = 0; // just for the color
    for (uint m = 0; m < in.size(); ++m){

        const cv::Point2f p1 = kp1[in[m].queryIdx].pt;
        const cv::Point2f p2 = kp2[in[m].trainIdx].pt;

        const cv::Point2f diff = p1-p2;

        // this is L2 norm, L1 norm also option! Eg diff.x+diff.y
        float l2 = norm(diff);//diff.x*diff.x + diff.y*diff.y;
        if (!disparityFilterOn || l2 < maxDisparity){
            outMatches.push_back(in[m]);
            outMatches.back().distance = l2;
//            const int col =  250 - (l2/maxDisparity* 200);
//            cv::line(img, p1, p2, CV_RGB(255-col,col,0),1,CV_AA);
//            cv::circle(img, p1,2,CV_RGB(0,255,255),1,CV_AA);
//            cv::circle(img, p2,2,CV_RGB(255,0,255),1,CV_AA);
        } else {
//            const int col =  60 - (std::min(1.f,l2/maxDisparity)* 60);
//            cv::line(img, p1, p2, CV_RGB(60-col,col,0),1,CV_AA);
//            cv::circle(img, p1,2,CV_RGB(0,60,60),1,CV_AA);
//            cv::circle(img, p2,2,CV_RGB(60,0,60),1,CV_AA);
        }
    }
    if (disparityFilterOn){
        ROS_INFO("MAT = Keeping %lu/%lu after disparity check [thresh = %.1f]", outMatches.size(), in.size(), maxDisparity);
    }
    return outMatches;
}
*/










/*
// Does the matching and match filtering.
// Note that the distance member of each Dmatch will hold the disparity of the match in pixels
void Matcher::match( FramePtr& fQuery,
                     FramePtr& fTrain,
                     DMatches& matches,
                     double& time,
                     float maxDisparity, // bearing vector disparity error
                     int prediction, // None, bv disparity, imu rotated bv disparity imu
                     const Ints& maskIdxQuery,  //list of valid descriptor incicies for the query descriptors. Default = None
                     const Ints& maskIdxTrain  //list of valid descriptor incicies for the query descriptors. Default = None
                     ) {
    ROS_INFO("MAT > Matching Frame [%d/%d] with Frame [%d/%d]", fQuery->getId(), fQuery->getKfId(),fTrain->getId(), fTrain->getKfId() );
    matches.clear();

    // Update they type of matcher depending on the descriptor type (eg float vs uchar)
    updateMatcher(fQuery->getDescType(), fQuery->getDescSize());

    // Get descriptors
    cv::Mat dQuery = fQuery->getDescriptors();
    cv::Mat dTrain = fTrain->getDescriptors();

    // start timer afer we have descriptors
    ros::WallTime m0 = ros::WallTime::now();


    if (fQuery->getDescSize() != fTrain->getDescSize()){
        ROS_ERROR("MAT = Attempting to match different descriptor sizes");
    },
    if (fQuery->getDescType()!= fTrain->getDescType()){
        ROS_ERROR("MAT = Attempting to match different descriptor sizes");
    }


    /// Train/Update FLANN
    if (m_flann){
        if (!matcher->empty()) {
            Mats trained = matcher->getTrainDescriptors();
            if (trained[0].size()!=dTrain.size() || trainedId!=fTrain->getId()){
                matcher->clear();
            }
        }
        if (!matcher->empty()) {
            Mats v;
            v.push_back(dTrain);
            matcher->add(v);
            matcher->train();
            ROS_WARN("MAT = Training KD-TREE on train descriptors for frame [%d|%d]" fTrain->getId(), fTrain->getKfId());
        }
    }




    /// Removed unmasked entries
    if (!matcher->isMaskSupported()){
        // Manually remove rows from descriptor matrix
        if (!maskIdxTrain.empty()){
            cv::Mat temp;
            OVO::matReduceInd(dTrain, temp, maskIdxTrain);
            cv::swap(dTrain,temp);
            ROS_WARN("MAT = Mask not supported and [Train] mask on, temporarily removing unmasked rows, Keeping [%d/%d]", temp.rows, dTrain.rows);
        }
        if (!maskIdxQuery.empty()){
            cv::Mat temp;
            OVO::matReduceInd(dTrain, temp, maskIdxTrain);
            cv::swap(dTrain,temp);
            ROS_WARN("MAT = Mask not supported and [Query] mask on, temporarily removing unmasked rows, Keeping [%d/%d]", temp.rows, dTrain.rows);
        }
    }


    if (dQuery.rows == 0){
        ROS_WARN("MAT = No query descriptors to match against");
        time = 0;
        return;
    }
    if (dTrain.rows == 0){
        ROS_WARN("MAT = No train descriptors to match against");
        time = 0;
        return;
    }


    bool toSort = false;

    // turn off m_doSym flag if we are using brute force matcher without KNN, as these do m_sym internally
    // NOTE: do_sym is only internally done inside BFMatcher if KNN is used wih K=1


    int idx = 16*m_flann + m_doUnique*8 + m_doSym*4+ m_doThresh*2+ m_doRatio ;
    // U - unique neighbour test
    // S - sym for flann
    // T - threshold
    // R - ratio test

    DMatchesKNN matchesKNN;
    DMatchesKNN q2t, t2q;
    cv::Mat mask2;


    ROS_INFO("MAT > Matching [%d vs %d] with [Case: %d]",dQuery.rows, dTrain.rows, idx);
    switch(idx){
    case 0 : // - - - - nothing
        // Best match for each kp
            if (m_flann){
                matcher->match(dQuery,dTrain,matches,mask);
            } else if (m_doSym){
                matcher->knnMatch(dQuery,dTrain,matchesKNN,1,mask);
                matchKnn2single(matchesKNN, matches);
            } else {
                matcher->match(dQuery,dTrain,matches,mask);
            }
        break;
    case 1 : // - - - R
        matcher->match(dQuery,dTrain,matches,mask);
        if (m_doMax){
            sortMatches(matches);
            toSort = true;
        }
        matchFilterRatio(matches, m_ratio, toSort);
        break;
    case 2 : // - - T -
        matcher->radiusMatch(dQuery, dTrain, matchesKNN, m_thresh, mask);
        toSort = m_doMax; // flag we should sort
        matchKnn2single(matchesKNN, matches, toSort); //sorts
        break;
    case 3 : // - - T R
        matcher->radiusMatch(dQuery, dTrain, matchesKNN, m_thresh, mask); toSort = true;
        matchKnn2single(matchesKNN, matches, toSort);
        matchFilterRatio(matches, m_ratio, toSort);
        break;
    case 4 : // - S - -
        ROS_ERROR("MAT = CASE [%d] NOT IMPLEMENTED!", idx);
        break;
    case 5 : // - S - R
        ROS_ERROR("MAT = CASE [%d] NOT IMPLEMENTED!", idx);
        break;
    case 6 : // - S T -
        ROS_ERROR("MAT = CASE [%d] NOT IMPLEMENTED!", idx);
        break;
    case 7 : // - S T R
        ROS_ERROR("MAT = CASE [%d] NOT IMPLEMENTED!", idx);
        break;
    case 8 : // U - - -
        matcher->knnMatch(dQuery,dTrain,matchesKNN,2,mask);
        toSort = m_doMax;
        matchFilterUnique(matchesKNN, matches, m_unique, toSort);
        break;
    case 9 : // U - - R
        matcher->knnMatch(dQuery,dTrain,matchesKNN,2,mask);
        toSort = m_doMax;
        matchFilterUnique(matchesKNN, matches, m_unique, toSort);
        matchFilterRatio(matches, m_ratio, toSort);
        break;
    case 10: // U - T -
        matcher->knnMatch(dQuery,dTrain,matchesKNN,2,mask);
        toSort = m_doMax;
        matchFilterUnique(matchesKNN, matches, m_unique, toSort);
        matchFilterThreshold(matches, m_thresh, toSort);
        break;
    case 11: // U - T R
        matcher->knnMatch(dQuery,dTrain,matchesKNN,2,mask);
        toSort = true;
        matchFilterUnique(matchesKNN, matches, m_unique, toSort);
        matchFilterRatio(matches, m_ratio, toSort);
        matchFilterThreshold(matches, m_thresh, toSort);
        break;
    case 12: // U S - -
        matcher->knnMatch(dQuery,dTrain,q2t, 2);
        matcher->knnMatch(dTrain,dQuery,t2q, 2);
        matchFilterUnique(q2t, m_unique);
        matchFilterUnique(t2q, m_unique);
        matchSymmetryTest(q2t, t2q, matches);
        break;
    case 13: // U S - R
        ROS_ERROR("MAT = CASE [%d] NOT IMPLEMENTED!", idx);
        break;
    case 14: // U S T -
        ROS_ERROR("MAT = CASE [%d] NOT IMPLEMENTED!", idx);
        break;
    case 15: // U S T R
        ROS_ERROR("MAT = CASE [%d] NOT IMPLEMENTED!", idx);
        break;
    }
    ROS_INFO("MAT < Matched [%lu]", matches.size());

    if (matches.size()==0){
        time = (ros::WallTime::now()-m0).toSec();
        ROS_WARN("MAT < Matcher returned no matches  of [%d vs %d] in [%.1fms]", dQuery.rows, dTrain.rows, time);
    } else {

        // IF we masked, restore original indicies in matches
        if (maskIdxTrain.size()>0){
            ROS_INFO("MAT = Mask on, adding correct offset to masked items");
            for (uint m = 0; m < matches.size(); ++m){
                matches[m].trainIdx = maskIdxTrain[matches[m].trainIdx];
                matches[m].queryIdx = maskIdxQuery[matches[m].queryIdx];
            }
        }


        // filter if disparity too high
        // sets distance to be the l2 norm squared
        // preservers order
        matches = disparityFilter(matches, fQuery, fTrain, maxDisparity, maxDisparity>0);


        // At this stage everything should be sorted by matching distance
        if (m_doMax){
            ROS_ASSERT_MSG(isSorted(matches), "MAT = Expecing matches to be sorted, but they are not");
            matchFilterBest(matches, m_max);
        }


        if (klt_refine && matches.size()>0){
            ROS_INFO("MAT > Doing KLT refinement over [%lu] matches. Window size [%d]", matches.size(), klt_window.x);
            ros::WallTime t1 = ros::WallTime::now();

            /// extract matched points
            KeyPoints nfKPts = fQuery->getKeypoints();
            KeyPoints mNfKps;
            Points2f kfPointsMatched;
            OVO::vecAlignMatch<KeyPoints, Points2f>(nfKPts, fTrain->getPoints(), mNfKps, kfPointsMatched, matches);

            Points2f klt_predictedPts;
            cv::KeyPoint::convert(mNfKps, klt_predictedPts);

            std::vector<unsigned char> klt_status;
            std::vector<float>         klt_error;

            cv::calcOpticalFlowPyrLK(fTrain->getPyramid(klt_window, klt_levels),                       // input
                                     fQuery->getPyramid(klt_window, klt_levels),                       // input
                                     kfPointsMatched,                                                    // input
                                     klt_predictedPts, klt_status, klt_error,                            // output row
                                     klt_window, klt_levels,  klt_criteria, cv::OPTFLOW_USE_INITIAL_FLOW, klt_eigenThresh); // settings

            // update KeyPoints
            int ok   = 0;
            for (size_t i=0; i<klt_status.size(); ++i){
                if (klt_status[i]){
                    nfKPts[matches[i].queryIdx].pt = klt_predictedPts[i];
                    ++ok;
                }
            }
            fQuery->swapKeypoints(nfKPts);
            ROS_INFO("FTR > Finished doing KLT refinement. [%d / %lu] successful in [%.1fms]", ok, matches.size(),1000* (ros::WallTime::now()-t1).toSec());
        }


        time = (ros::WallTime::now()-m0).toSec();
        float ratio = static_cast<float>(matches.size()) / std::min(dQuery.rows, dTrain.rows);
        ROS_WARN_COND(ratio<0.1f && matches.size()<100, "MAT = Only matched %.1f%% and <100 Matches accepted", 100.f*ratio);
        ROS_INFO("MAT < Matching finished [%d vs %d = %.1f%% = %lu matches] in [%.1fms]", dQuery.rows, dTrain.rows, 100.f*ratio, matches.size(), time*1000);

    }
}
*/
