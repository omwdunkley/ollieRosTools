#include <ollieRosTools/Matcher.hpp>

Matcher::Matcher(){
    m_type = 1; // 0= BruteForce, 1 = FLANN
    m_sym_neighbours = 0; // 0 = no symmetry
    m_unique = 0; // 0 = off
    m_thresh = 0; // 0 = no threshold
    m_ratio = 1; //1 = off
    m_max = 0; // 0 = unlimited
    m_pxdist = 100;
    descType=-1;
    updateMatcher(CV_8U,205);
}









void Matcher::updateMatcher(int type, bool update){


    if(type==descType && !update){
        return;
    }

    descType=type;


    if(m_type){ //FLANN
        if (type==CV_8U){ // BINARY DESCRIPTOR
            ROS_INFO("Updated Matcher: FLANN BINARY");
            matcher = new cv::FlannBasedMatcher(new cv::flann::LshIndexParams(20,10,2),new cv::flann::SearchParams(32,0,true));
        } else if (type==CV_32F){ // FLOAT DESCRIPTOR
            ROS_INFO("Updated Matcher: FLANN FLOAT");
            matcher = new cv::FlannBasedMatcher(new cv::flann::KDTreeIndexParams(4), new cv::flann::SearchParams(32, 0, true));
        } else {
            ROS_WARN("Unknown Desc Type: %d", type);
        }
    } else { //BRUTEFORCE
        if (type==CV_8U){ // BINARY DESCRIPTOR
            ROS_INFO("Updated Matcher: BRUTEFORCE BINARY");
            matcher = new cv::BFMatcher(cv::NORM_HAMMING2, m_doSym);
            // orb 3,4:
            //matcher = new cv::BFMatcher(cv::NORM_HAMMING, m_doSym);
        } else if (type==CV_32F){ // FLOAT DESCRIPTOR
            ROS_INFO("Updated Matcher: BRUTEFORCE FLOAT");
            matcher = new cv::BFMatcher(m_norm, m_doSym);
        } else {
            ROS_WARN("Unknown Desc Type: %d", type);
        }
    }


}




ollie_vo::VoNode_paramsConfig& Matcher::setParameter(ollie_vo::VoNode_paramsConfig &config, uint32_t level){


    //////////////////////////////////////////////////////////////// MATCHER
    //TODO add FLANN vars

    m_unique = config.match_unique;
    m_norm = config.match_norm;
    m_thresh = config.match_thresh;
    m_max = config.match_max;
    m_ratio = config.match_ratio;
    m_type = config.matcher;
    m_pxdist = config.match_px*config.match_px;
    m_sym_neighbours = config.match_symmetric;

    // For ease of use later

    m_doUnique = m_unique>0;
    m_doSym    = m_sym_neighbours>0;
    m_doThresh = m_thresh>0;
    m_doRatio  = m_ratio>1;
    m_doMax    = m_max>0;
    m_doPxdist = m_pxdist>1;


    updateMatcher(descType, true);




    // Cache to detect fugure differences
    config_pre = config;

    return config;

}






/// TODO: so far just a place holder
void Matcher::match( KeyFrame& frame,
                     PointMap& map,
                     DMatchesKNN& matches,                           // matches out
                     double& disparity,
                     double& time
                     ) {


    /// maximum number of kfs to match against
    const uint maxKFMatch = 1; ///TODO
    /// minimum required matches per KF
    const uint minMatchPerKF = 5; ///TODO

    ROS_INFO(" ---> MATCHING CF against %d MAP KFs", map.size());



    ros::WallTime m0 = ros::WallTime::now();



    /// Need some way of chosing which map frames are close to frame. For now just use last X frames
    // idea: images should look similar (ssd of imu-rotated SBIs)
    // idea: rotation should be similar
    // idea: estimated distance should be similar
    const KeyFrame f = map.getClosestKF(frame);






    matches.clear();
    matches.resize(map.size(),DMatches());
    double timeNotUsed;
    Doubles disparities;
    disparities.resize(map.size(),INFINITY);

    /// todo dont show all the image

    const bool wasDoingDispThresh = m_doPxdist;

    for (uint m = 0; m < std::min(maxKFMatch, static_cast<uint>(map.size())); ++m){
        //ROS_INFO("Matching frame against map frame[%d]", m);
        cv::Mat mask;

        /// Use mask to select valid pairs - only use desc that have associated 3d points
        if (matcher->isMaskSupported() && !m_doSym){
            mask = cv::Mat::zeros(frame.getDesc().rows, map.getKF(m).getDesc().rows, CV_8U);
            Bools jok = map.getKF(m).getHasMapPoints();
            for (uint j=0; j< jok.size(); ++j){
                if (jok[j]){
                    mask.col(j)=1;
                }
            }
            match(frame, map.getKF(m), matches[m], disparities[m], timeNotUsed, mask);
        } else {
            // Mask not supported. Select subset of desc, rebuild matches afterwards
            match(frame, map.getKF(m), matches[m], disparities[m], timeNotUsed, cv::Mat(), map.getKF(m).getMapPointIdx());
        }

        // Reject matches if there are too few
        if (matches[m].size()<minMatchPerKF){
            disparities[m] = INFINITY;
            matches[m].clear();
        }

        /// only disparity threshold matches on the previous frame. not the rest
        if (m==0){
            m_doPxdist = false;
        }

    }

    // reset to previous setting
    m_doPxdist = wasDoingDispThresh;




    /// TODO: FILTER
    // Only take groups of matches that have a certain amount of matches, eg any group with < 10 matches should be disgarded
    // for now we only use the previous frame
    // return boolean if at least some good ones were found




    /// Chose disparity to be the min disparity. Note we set disparity to inf if we dont consider a KF
    /// Could also use disparity of KF with most matches

    uint mostMatchKFID = 0;
    int minDispKF = 0;
    cv::minMaxIdx(disparities,&disparity, NULL, &minDispKF);
    for(uint i=0; i<disparities.size(); ++i){
        if (matches[i].size()>matches[mostMatchKFID].size()){
            mostMatchKFID = i;
        }
        ROS_INFO("MAP KF[%d]: Matches %d  Disparity: %.2f",i,matches[i].size(), sqrt(disparities[i]));
    }


    ROS_INFO("Disparity [MIN    ] -> MAP KF[%d]=%f",minDispKF, sqrt(disparity));
    ROS_INFO("Disparity [MATCHES] -> MAP KF[%d]=%f",mostMatchKFID, sqrt(disparities[mostMatchKFID]));


    time = (ros::WallTime::now()-m0).toSec();


}



// Does the matching and match filtering.
// Note that the distance member of each Dmatch now holds the disparity of the pixels
void Matcher::match( KeyFrame& f1,
                     KeyFrame& f2,
                     DMatches& matches,                           // matches out
                     double& disparity,
                     double& time,
                     const cv::Mat& mask, //only input if crosscheck=off and matcher=BruteForce
                     const Ints& maskIdx  //only inpuot if mask=empty. idx of valid descriptor rows
                     ) {
    ros::WallTime m0 = ros::WallTime::now();
    matches.clear();

    updateMatcher(f1.getDescType());

    if(f1.getDescId()!=f2.getDescId()){
        ROS_WARN("Skipping match as both frames have different descriptor types: %d != %d",f1.getDescId(), f2.getDescId());
        return;
    }


    DMatchesKNN matchesKNN;
    const cv::Mat d1 = f1.getDesc();
    cv::Mat d2 = f2.getDesc();


    // Mask not supported, use maskIDx instead
    if (maskIdx.size()>0){
        cv::Mat temp;
        matReduceInd(d2,temp,maskIdx);
        cv::swap(d2,temp);
        //matcher->set("crossCheck", false);
    }


    // turn off m_doSym flag if we are using brute force matcher, as these do m_sym internally

    // Manually do symmetric test
    bool sorted = false;
    bool doSym =(m_doSym && m_type==1 );
    int idx = m_doUnique*8 + doSym*4+ m_doThresh*2+ m_doRatio;
    ROS_WARN_COND(doSym, "FLANN SYMMETRY MATCHER NOT IMPLEMENTED");
    switch(idx){
    case 0 : // - - - - nothing
        // Best match for each kp
        matcher->match(d1,d2,matches,mask); sorted = true;
        break;
    case 1 : // - - - R
        matcher->match(d1,d2,matches,mask); sorted = true;
        match_filter_ratio(matches, m_ratio, sorted);
        break;
    case 2 : // - - T -
        matcher->radiusMatch(d1, d2, matchesKNN, m_thresh, mask); sorted = m_doMax; // flag we should sort
        match_knn2single(matchesKNN, matches, sorted);
        break;
    case 3 : // - - T R
        matcher->radiusMatch(d1, d2, matchesKNN, m_thresh, mask); sorted = true;
        match_knn2single(matchesKNN, matches, sorted);
        match_filter_ratio(matches, m_ratio, sorted);
        break;
    case 4 : // - S - -
        break;
    case 5 : // - S - R
        break;
    case 6 : // - S T -
        break;
    case 7 : // - S T R
        break;
    case 8 : // U - - -
        matcher->knnMatch(d1,d2,matchesKNN,2,mask); sorted = m_doMax;
        match_filter_unique(matchesKNN, matches, m_unique, sorted);
        break;
    case 9 : // U - - R
        matcher->knnMatch(d1,d2,matchesKNN,2,mask); sorted = m_doMax;
        match_filter_unique(matchesKNN, matches, m_unique, sorted);
        match_filter_ratio(matches, m_ratio, m_doMax);
        break;
    case 10: // U - T -
        matcher->knnMatch(d1,d2,matchesKNN,2,mask); sorted = m_doMax;
        match_filter_unique(matchesKNN, matches, m_unique, sorted);
        match_threshold(matches, m_thresh, m_doMax);
        break;
    case 11: // U - T R
        matcher->knnMatch(d1,d2,matchesKNN,2,mask);sorted = true;
        match_filter_unique(matchesKNN, matches, m_unique, sorted);
        match_threshold(matches, m_thresh, sorted);
        match_filter_ratio(matches, m_ratio, sorted);
        break;
    case 12: // U S - -
        break;
    case 13: // U S - R
        break;
    case 14: // U S T -
        break;
    case 15: // U S T R
        break;
    }


    // IF we masked, restore original indicies in matches
    if (maskIdx.size()>0){
        for (uint m = 0; m < matches.size(); ++m){
            matches[m].trainIdx = maskIdx[matches[m].trainIdx];
        }
//        if (m_doSym){
//            matcher->set("crossCheck", m_doSym); //restore setting
//        }
    }



    // put disparity in matches
    // filter if disparity too high
    matches = disparityFilter(matches, f1, f2, m_pxdist, m_doPxdist);


    // At this stage everything should be sorted anyway...
    // Here is sorts by disparity. This might not be good...
    if (m_doMax){
        match_clip(matches, m_max, sorted);
    }

    disparity = matchMedianDistance(matches);
    //disparity = sqrt(matchAverageDistance(matches));

    time = (ros::WallTime::now()-m0).toSec();

}




// Sort matches by response and take nr best
// NOTE: Usually you wont need this as the matchers sort the output by distance
void match_clip(DMatches& ms, const uint max_nr, bool is_sorted){
    if (ms.size()>max_nr){
        if (!is_sorted){
            std::sort( ms.begin(),ms.end());
        }
        ms.erase(ms.begin()+MIN(max_nr,ms.size()),ms.end());
    }
}





// cant use const here, need a copy to sort
double matchMedianDistance(DMatches ms){
    double median=0;
    const size_t size = ms.size();
    if (size>0){
        sort(ms.begin(), ms.end());
        if (size  %2 == 0)  {
            median = (ms[size/2 -1].distance + ms[size / 2].distance) / 2;
        }  else   {
            median = ms[size/2].distance;
        }
    }
    return median;
}


double matchAverageDistance(const DMatches& ms){
  double s = 0;
  const uint size = ms.size();
  for (uint i=0; i<size; ++i){
      s+=ms[i].distance;
  }
  return s/size;
}




// Returns true if match not good enough
bool match_bad(const cv::DMatch& match, const double thresh){
    return (match.distance > thresh);
}



// Returns true if match good enough
bool match_good(const cv::DMatch& match, const double thresh){
    return (match.distance < thresh);
}


// Remove all matches below threshold.
void match_threshold(DMatches& ms, double thresh, bool is_sorted){
    if (is_sorted){
        // Logarithmic version
        DMatches::iterator low = std::lower_bound (ms.begin(), ms.end(), thresh, match_good);
        ms.erase( low, ms.end() );
    } else {
        // O(N)
        ms.erase( std::remove_if( ms.begin(), ms.end(), boost::bind(match_bad, _1, thresh)), ms.end() );
    }
}


// remove matches that are ratio * worse than the best
void match_filter_ratio(DMatches& ms, const double ratio, const bool is_sorted){

    if (ms.size()>1){
        double best;

        if (is_sorted){
            best = ms[0].distance;
        } else {
            double worst;
            uint nrd;
            minMaxTotal(ms, best, worst, nrd);
        }

        const double thresh = std::max(1e-8,ratio*best);
        ROS_INFO("Match Ratio: %d matches, %f closest distance, %f ratio, %f thresh, %d sorted", ms.size(), best, ratio, thresh, is_sorted);
        match_threshold(ms, thresh, is_sorted);
        ROS_INFO(" ---> %d left",ms.size());
    }
}



// Reduces vector of vectors DMatchesKNN to a single vector DMatches
void match_knn2single(const DMatchesKNN& msknn, DMatches& ms, const bool keep_sorted){
    ms.clear();
    ms.reserve(msknn.size()*2); // just guess
    // TO DO some kind of insertion sort could be better?
    for(size_t i=0; i<msknn.size(); ++i) {
        for(size_t j=0; j<msknn[i].size(); ++j) {
            ms.push_back(msknn[i][j]);
        }
    }

    // TO DO some kind of insertion sort could be better?
    if (keep_sorted){
        std::sort(ms.begin(),ms.end());
    }

}



// Filter out matches that are not unique. Specifically unqiue = dist1/dist2 > similarity
void match_filter_unique(const DMatchesKNN& msknn, DMatches& ms, const double similarity, const bool keep_sorted){
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
        std::sort(ms.begin(), ms.end());
    }
}



// Only accept matches where the pixel distance^2 between two matches is < thresh
/// TODO use l1 norm!
// Preserves sorting
// changes distance member of DMatch to disparity
DMatches disparityFilter(const DMatches& in, KeyFrame& f1, KeyFrame& f2, const double maxDisparity, bool disparityFilterOn){
    const cv::Size& size= f1.getImage().size();
    cv::Mat img = cv::Mat::zeros(size,CV_8UC3);
    DMatches out;

    const KeyPoints& kp1 = f1.getKPsUnRot();
    const KeyPoints& kp2 = f2.getKPsUnRot();



//    cv::Mat img1 = cv::Mat::zeros(size,CV_8UC3);
//    cv::Mat img2 = cv::Mat::zeros(size,CV_8UC3);
//    cv::drawKeypoints(img1,f1.getKPs(),img1,CV_RGB(255,0,0));
//    cv::drawKeypoints(img1,f1.getKPsUnRot(),img1,CV_RGB(0,0,255));
//    cv::drawKeypoints(img2,f2.getKPs(),img2,CV_RGB(255,0,0));
//    cv::drawKeypoints(img2,f2.getKPsUnRot(),img2,CV_RGB(0,0,255));

//    cv::imshow("img1",img1);
//    cv::imshow("img2",img2);



    out.reserve(in.size());


    double sum = 0; // just for the color
    for (uint m = 0; m < in.size(); ++m){

        const cv::Point2d p1 = kp1[in[m].queryIdx].pt;
        const cv::Point2d p2 = kp2[in[m].trainIdx].pt;

        const cv::Point2d diff = p1-p2;

        // this is L2 norm, L1 norm also option! Eg diff.x+diff.y
        double l2 = diff.x*diff.x + diff.y*diff.y;
        if (!disparityFilterOn || l2 < maxDisparity){
            out.push_back(in[m]);
            out.back().distance = l2;
            const int col =  250 - (l2/maxDisparity* 200);
            cv::line(img, p1, p2, CV_RGB(255-col,col,0),1,CV_AA);
            cv::circle(img, p1,2,CV_RGB(0,255,255),1,CV_AA);
            cv::circle(img, p2,2,CV_RGB(255,0,255),1,CV_AA);
            sum +=sqrt(l2);
        } else {
            const int col =  60 - (std::min(1.,l2/maxDisparity)* 60);
            cv::line(img, p1, p2, CV_RGB(60-col,col,0),1,CV_AA);
            cv::circle(img, p1,2,CV_RGB(0,60,60),1,CV_AA);
            cv::circle(img, p2,2,CV_RGB(60,0,60),1,CV_AA);
        }
    }

//    cv::circle(img, cv::Point2d(size.width/2, size.height/2), std::max(1.,sqrt(maxDisparity)), CV_RGB(255,255,0),1,CV_AA);
//    const double avg = (sum/out.size());
//    const int col =  250 - (avg/sqrt(maxDisparity)* 200);
//    cv::circle(img, cv::Point2d(size.width/2, size.height/2), std::max(1.,avg), CV_RGB(255-col,col,0),1,CV_AA);

//    //ROS_INFO("Left: %d/%d", out.size(), in.size());
//    cv::imshow("unrotated disparity", img); cv::waitKey(30);
    return out;
}



// Return the smallest, largest distance along with the total nr of matches
void minMaxTotal(const DMatches& ms, double& minval, double& maxval, uint& total){
    minval = INFINITY;
    maxval = -INFINITY;
    for (uint x=0; x < ms.size(); ++x){
        double r = ms[x].distance;
        if( r < minval ) {
            minval = r;
        }
        if( r > maxval ){
            maxval = r;
        }
    }
    total =ms.size();
}

