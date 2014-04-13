#include <ollieRosTools/Matcher.hpp>

Matcher::Matcher(){
    m_type = 1; // 0= BruteForce, 1 = FLANN
    m_sym_neighbours = 0; // 0 = no symmetry
    m_unique = 0; // 0 = off
    m_thresh = 0; // 0 = no threshold
    m_ratio = 1; //1 = off
    m_max = 0; // 0 = unlimited
    //m_pxdist = 100;
    descType=-1;
    descSize=-1;
    updateMatcher(CV_8U,205);
}






bool isSorted(const DMatches& ms){
    // returns true if the distances are sorted by size, lowest first
    ROS_INFO("Checking if matches are sorted");
    for (uint i=1; i<ms.size(); ++i){
        if (ms[i-1].distance > ms[i].distance){
            return false;
        }
    }
    return true;
}



void Matcher::updateMatcher(const int type, const int size, const bool update){


    if(type==descType && size==descSize && !update){
        return;
    }

    descType=type;
    descSize=size;


    if(m_type){ //FLANN
        if (type==CV_8U){ // BINARY DESCRIPTOR
            ROS_INFO("MAT = Updated Matcher: FLANN BINARY");
            //matcher = new cv::FlannBasedMatcher(new cv::flann::LshIndexParams(20,10,2),new cv::flann::SearchParams(32,0,true));
            matcher = new cv::FlannBasedMatcher(new cv::flann::LshIndexParams(6,12,1),new cv::flann::SearchParams(32,0,true));
        } else if (type==CV_32F){ // FLOAT DESCRIPTOR
            ROS_INFO("MAT = Updated Matcher: FLANN FLOAT");
            matcher = new cv::FlannBasedMatcher(new cv::flann::KDTreeIndexParams(4), new cv::flann::SearchParams(32, 0, true));
        } else {
            ROS_WARN("MAT = Unknown Desc Type: %d", type);
        }
    } else { //BRUTEFORCE
        bool sym = m_doSym && !m_doUnique; // cannot do KNN matching if this flag is set;
        if (type==CV_8U){ // BINARY DESCRIPTOR
            ROS_INFO("MAT = Updated Matcher: BRUTEFORCE BINARY");
            if (orb34){
                // orb 3,4:
                matcher = new cv::BFMatcher(cv::NORM_HAMMING2, sym); //2bit
            } else {
                matcher = new cv::BFMatcher(cv::NORM_HAMMING, sym); //1bit
            }

        } else if (type==CV_32F){ // FLOAT DESCRIPTOR
            ROS_INFO("MAT = Updated Matcher: BRUTEFORCE FLOAT");
            matcher = new cv::BFMatcher(m_norm, sym);
        } else {
            ROS_WARN("MAT = Unknown Desc Type: %d", type);
        }
    }


}




void Matcher::setParameter(ollieRosTools::VoNode_paramsConfig &config, uint32_t level){
    ROS_INFO("MAT > SETTING PARAM");

    //////////////////////////////////////////////////////////////// MATCHER
    //TODO add FLANN vars

    m_unique = config.match_unique;
    m_norm = config.match_norm;
    m_thresh = config.match_thresh;
    m_max = config.match_max;
    m_ratio = config.match_ratio;
    m_type = config.matcher;
    //m_pxdist = config.match_px;
    m_sym_neighbours = config.match_symmetric;

    // For ease of use later

    m_doUnique = m_unique>0;
    m_doSym    = m_sym_neighbours>0;
    m_doThresh = m_thresh>0;
    m_doRatio  = m_ratio>1;
    m_doMax    = m_max>0;
    //m_doPxdist = m_pxdist>1;

    orb34 = config.extractor==7 || config.extractor==8;
    updateMatcher(descType, true);

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

    ROS_INFO("MAT < PARAM SET");

}



/*
/// TODO: so far just a place holder
//void Matcher::match( FramePtr& frame,
//                     PointMap& map,
//                     DMatchesKNN& matches,                           // matches out
//                     float& disparity,
//                     float& time
//                     ) {


//    /// maximum number of kfs to match against
//    const uint maxKFMatch = 1; ///TODO
//    /// minimum required matches per KF
//    const uint minMatchPerKF = 5; ///TODO

//    ROS_INFO(" ---> MATCHING CF against %d MAP KFs", map.size());



//    ros::WallTime m0 = ros::WallTime::now();



//    /// Need some way of chosing which map frames are close to frame. For now just use last X frames
//    // idea: images should look similar (ssd of imu-rotated SBIs)
//    // idea: rotation should be similar
//    // idea: estimated distance should be similar
//    const FramePtr f = map.getClosestKF(frame);






//    matches.clear();
//    matches.resize(map.size(),DMatches());
//    float timeNotUsed;
//    floats disparities;
//    disparities.resize(map.size(),INFINITY);

//    /// todo dont show all the image

//    const bool wasDoingDispThresh = m_doPxdist;

//    for (uint m = 0; m < std::min(maxKFMatch, static_cast<uint>(map.size())); ++m){
//        //ROS_INFO("Matching frame against map frame[%d]", m);
//        cv::Mat mask;

//        /// Use mask to select valid pairs - only use desc that have associated 3d points
//        if (matcher->isMaskSupported() && !m_doSym){
//            mask = cv::Mat::zeros(frame.getDesc().rows, map.getKF(m).getDesc().rows, CV_8U);
//            Bools jok = map.getKF(m).getHasMapPoints();
//            for (uint j=0; j< jok.size(); ++j){
//                if (jok[j]){
//                    mask.col(j)=1;
//                }
//            }
//            match(frame, map.getKF(m), matches[m], disparities[m], timeNotUsed, mask);
//        } else {
//            // Mask not supported. Select subset of desc, rebuild matches afterwards
//            match(frame, map.getKF(m), matches[m], disparities[m], timeNotUsed, cv::Mat(), map.getKF(m).getMapPointIdx());
//        }

//        // Reject matches if there are too few
//        if (matches[m].size()<minMatchPerKF){
//            disparities[m] = INFINITY;
//            matches[m].clear();
//        }

//        /// only disparity threshold matches on the previous frame. not the rest
//        if (m==0){
//            m_doPxdist = false;
//        }

//    }

//    // reset to previous setting
//    m_doPxdist = wasDoingDispThresh;




//    /// TODO: FILTER
//    // Only take groups of matches that have a certain amount of matches, eg any group with < 10 matches should be disgarded
//    // for now we only use the previous frame
//    // return boolean if at least some good ones were found




//    /// Chose disparity to be the min disparity. Note we set disparity to inf if we dont consider a KF
//    /// Could also use disparity of KF with most matches

//    uint mostMatchKFID = 0;
//    int minDispKF = 0;
//    cv::minMaxIdx(disparities,&disparity, NULL, &minDispKF);
//    for(uint i=0; i<disparities.size(); ++i){
//        if (matches[i].size()>matches[mostMatchKFID].size()){
//            mostMatchKFID = i;
//        }
//        ROS_INFO("MAP KF[%d]: Matches %d  Disparity: %.2f",i,matches[i].size(), sqrt(disparities[i]));
//    }


//    ROS_INFO("Disparity [MIN    ] -> MAP KF[%d]=%f",minDispKF, sqrt(disparity));
//    ROS_INFO("Disparity [MATCHES] -> MAP KF[%d]=%f",mostMatchKFID, sqrt(disparities[mostMatchKFID]));


//    time = (ros::WallTime::now()-m0).toSec();


//}
*/



//
void symmetryTest(const DMatchesKNN& ms1,const DMatchesKNN& ms2, DMatches& msSym) {
    ROS_INFO("MAT > Symmetrical Check [%lu vs %lu]", ms1.size(), ms2.size());
    msSym.clear();
    int size = std::min(ms1.size(), ms2.size());
    if (size==0){
        ROS_WARN("MAT = NOTHING TO SYMMETRY MATCH");
    } else {
        msSym.reserve(size);

        //loop through smaller one
        if (ms1.size()<=ms2.size()){
            ROS_INFO("MAT = Query is smaller");
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
            ROS_INFO("MAT = Train is smaller");
            for (int i=0; i<size; ++i){
                const DMatches& m2v = ms2[i];
                if(m2v.size()>0){
                    const DMatches& m1v = ms1[i];
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


void sortMatches(DMatches& ms){
    std::sort(ms.begin(), ms.end());
}

// Does the matching and match filtering.
// Note that the distance member of each Dmatch will hold the disparity of the match in pixels
void Matcher::match( FramePtr& fQuery,
                     FramePtr& fTrain,
                     DMatches& matches,                     
                     double& time,
                     float maxDisparity,
                     const cv::Mat& mask, //only input if crosscheck=off and matcher=BruteForce
                     const Ints& maskIdx  //only inpuot if mask=empty. idx of valid descriptor rows
                     ) {
    ROS_INFO("MAT > Matching Frame [%d/%d] with Frame [%d/%d]", fQuery->getId(), fQuery->getKfId(),fTrain->getId(), fTrain->getKfId() );

    ROS_ASSERT_MSG(mask.empty(), "MAT = NOT SUPPORTED MASKING YET");
    ROS_ASSERT_MSG(maskIdx.empty(), "MAT = NOT SUPPORTED MASKING YET");

    matches.clear();

    // Update they type of matcher depending on the descriptor type (eg float vs uchar)
    updateMatcher(fQuery->getDescType(), fQuery->getDescSize());


    DMatchesKNN matchesKNN;

    const cv::Mat dQuery = fQuery->getDescriptors();
    cv::Mat dTrain = fTrain->getDescriptors();

    if (fQuery->getDescSize() != fTrain->getDescSize()){
        ROS_ERROR("MAT = Attempting to match different descriptor sizes");
    }
    if (fQuery->getDescType()!= fTrain->getDescType()){
        ROS_ERROR("MAT = Attempting to match different descriptor sizes");
    }



    ros::WallTime m0 = ros::WallTime::now();

    if (m_type){
        // if train frame id is the same
        // if descriptors are the same size/type
        // if matcher is the same
        // -> so nothing

        // if something changed:
        matcher->clear();
        Mats v;
        v.push_back(dTrain);
        matcher->add(v);
        matcher->train();
        ROS_ASSERT_MSG(false,"MAT = NEED TO IMPLEMENT THIS");

        // TODO: change the calling methods
    }



    // Mask not supported, use maskIDx instead
    if (maskIdx.size()>0){
        ROS_INFO("MAT = Mask on, temporarily removing masked items");
        cv::Mat temp;
        OVO::matReduceInd(dTrain, temp, maskIdx);
        cv::swap(dTrain,temp);
        ROS_INFO("MAT = Mask on, temporarily removing masked items, Keeping %d/%d", temp.rows, dTrain.rows);
        //matcher->set("crossCheck", false);
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




    // Manually do symmetric test
    bool toSort = false;

    // turn off m_doSym flag if we are using brute force matcher without KNN, as these do m_sym internally
    bool doSym =(m_doSym && (m_type==1 || m_doUnique) );


    int idx = m_doUnique*8 + doSym*4+ m_doThresh*2+ m_doRatio;
    // U - unique neighbour test
    // S - sym for flann
    // T - threshold
    // R - ratio test

    DMatchesKNN q2t, t2q;
    cv::Mat mask2;


    ROS_INFO("MAT > Matching [%d vs %d] with [Case: %d]",dQuery.rows, dTrain.rows, idx);
    switch(idx){
    case 0 : // - - - - nothing
        // Best match for each kp
        matcher->match(dQuery,dTrain,matches,mask);
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
        matchThreshold(matches, m_thresh, toSort);
        break;
    case 11: // U - T R
        matcher->knnMatch(dQuery,dTrain,matchesKNN,2,mask);
        toSort = true;
        matchFilterUnique(matchesKNN, matches, m_unique, toSort);
        matchFilterRatio(matches, m_ratio, toSort);
        matchThreshold(matches, m_thresh, toSort);
        break;
    case 12: // U S - -
        matcher->knnMatch(dQuery,dTrain,q2t, 2);
        matcher->knnMatch(dTrain,dQuery,t2q, 2);
        matchFilterUnique(q2t, m_unique);
        matchFilterUnique(t2q, m_unique);
        symmetryTest(q2t, t2q, matches);
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
        if (maskIdx.size()>0){
            ROS_INFO("MAT = Mask on, restoring temporarily removed masked items");
            for (uint m = 0; m < matches.size(); ++m){
                matches[m].trainIdx = maskIdx[matches[m].trainIdx];
            }
            //        if (m_doSym){
            //            matcher->set("crossCheck", m_doSym); //restore setting
            //        }
            ROS_WARN("NOT TESTED?");
        }



        // filter if disparity too high
        // sets distance to be the l2 norm squared
        // preservers order
        matches = disparityFilter(matches, fQuery, fTrain, maxDisparity, maxDisparity>0);


        // At this stage everything should be sorted by matching distance
        if (m_doMax){
            ROS_ASSERT_MSG(isSorted(matches), "MAT = Expecing matches to be sorted, but they are not");
            matchClip(matches, m_max);
        }


        //    // use median distance if sorted, as we can jsut take the middle element. Else compute average
        //    if (sorted){
        //        disparity = matchMedianDistance(matches, sorted);
        //    } else {
        //        disparity = matchAverageDistance(matches);
        //    }

        time = (ros::WallTime::now()-m0).toSec();
        float ratio = static_cast<float>(matches.size()) / std::min(dQuery.rows, dTrain.rows);
        ROS_WARN_COND(ratio<0.1f && matches.size()<100, "MAT = Only matched %.1f%% and <100 Matches accepted", 100.f*ratio);
        ROS_INFO("MAT < Matching finished [%d vs %d = %.1f%% = %lu matches] in [%.1fms]", dQuery.rows, dTrain.rows, 100.f*ratio, matches.size(), time*1000);

    }
}




// Cuts the tail of. Must be sorted by something to make else

void matchClip(DMatches& ms, const uint max_nr){
    ROS_INFO("MAT = Capping top [%lu/%u] matches", ms.size(), max_nr);
    if (ms.size()>max_nr){
        ms.erase(ms.begin()+MIN(max_nr,ms.size()),ms.end());
    }
}





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




// Returns true if match not good enough
bool matchBad(const cv::DMatch& match, const float thresh){
    return (match.distance > thresh);
}



// Returns true if match good enough
bool matchGood(const cv::DMatch& match, const float thresh){
    return (match.distance < thresh);
}


// Remove all matches below threshold.
void matchThreshold(DMatches& ms, float thresh, bool is_sorted){
    ROS_INFO("MAT > Thresholding [%lu] matches by distance [%f]", ms.size(), thresh);
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

    ROS_INFO("MAT < Thresholded matches by distance [%lu / %u] left", ms.size(), s);
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
        ROS_INFO("MAT > Filtering [%lu] matches by ratio. Range: [%f - %f], [%f] thresh", ms.size(), best, worst, thresh);

        if (worst>thresh){
            uint s = ms.size();
            matchThreshold(ms, thresh, is_sorted);
            ROS_INFO("MAT < [%lu/%u] left", ms.size(), s);
        } else {
            ROS_INFO("MAT < None to Ratio Filter");
        }

    }
}



// Reduces vector of vectors DMatchesKNN to a single vector DMatches
void matchKnn2single(const DMatchesKNN& msknn, DMatches& ms, const bool keep_sorted, const size_t maxPerMatch){
    ms.clear();
    ms.reserve(msknn.size()*2); // just guess
    // TO DO some kind of insertion sort could be better?
    for(size_t i=0; i<msknn.size(); ++i) {
        size_t maxpm = maxpm==0 ? msknn[i].size(): std::min(maxPerMatch,msknn[i].size());
        for(size_t j=0; j<maxpm; ++j) {
            // some kind of insertion sort might be better..
            ms.push_back(msknn[i][j]);
        }
    }

    // TO DO some kind of insertion sort could be better?
    if (keep_sorted){
        std::sort(ms.begin(),ms.end());
    }

}



// Filter out matches that are not unique. Specifically unqiue = dist1/dist2 > similarity
void matchFilterUnique(const DMatchesKNN& msknn, DMatches& ms, const float similarity, const bool keep_sorted){
    ROS_INFO("MAT > Filtering [%lu] Matches by uniqueness", msknn.size());
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
            ms.push_back(m1);
        }
    }
    // TO DO some kind of insertion sort could be better?
    if (keep_sorted){
        sortMatches(ms);
    }
    ROS_INFO("MAT < Matches [%lu/%lu] remaining after uniqueness test", ms.size(), msknn.size());
}

// Same as above but preservers length and order. Filters in place
void matchFilterUnique(DMatchesKNN& msknn, const float similarity){
    ROS_INFO("MAT > Filtering [%lu] Matches by uniqueness in place", msknn.size());
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
    ROS_INFO("MAT < Matches [%u/%lu] remaining after uniqueness test", counter, msknn.size());
    std::swap(out, msknn);
}





// Preserves sorting
// changes distance member of DMatch to disparity
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




