#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>


#include <Eigen/StdVector>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

//#include <g2o/math_groups/se3quat.h>
#include <g2o/solvers/structure_only/structure_only_solver.h>


#include <geometry_msgs/PoseArray.h>

#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/TranslationOnlySacProblem.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/triangulation/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>

#include <ollieRosTools/aux.hpp>
#include <ollieRosTools/Frame.hpp>
#include <ollieRosTools/Map.hpp>

#include <ollieRosTools/custom_types/edge_pose_landmark_reprojectBV.hpp>
#include <ollieRosTools/custom_types/vertex_landmarkxyz.hpp>
#include <ollieRosTools/custom_types/vertex_pose.hpp>
#include <ollieRosTools/custom_types/register_types.hpp>


namespace RP = opengv::sac_problems::relative_pose;





class Odometry
{

private:
    /// ENUMS
    // Keeps track of the internal state of the VO pipeline
    enum State {ST_WAIT_FIRST_FRAME, // waiting for the first ever keyframe
                ST_WAIT_INIT,        // waiting for initial initialisation
                ST_TRACKING,         // Initialised and triangulated points
                ST_LOST              // Initialised by no reference to keyframe
               };
    // Allows the user to manually initiate steps in the pipeline
    enum Control {CTR_DO_NOTHING, // No user input
                  CTR_DO_ADDKF,   // Force adding KF (if initialised, this adds one, if not, it forces initialisation)
                  CTR_DO_RESET    // resets everything, emptying the map and returning to WAIT_FIRST_FRAME state
               };
    // Different ways to chose the initial baseline
    enum BaselineMethod {BL_UNCHANGED=-1,    // Baseline is not altered
                         BL_FIXED,           // Baseline fixed to one meter
                         BL_MANUAL_BASELINE, // Baseline specified by the user
                         BL_MANUAL_AVGDEPTH, // Baseline adjusted so the specified avg depth is reached
                         BL_AUTO_BASELINE    // Baseline is estimated absolutey, eg by known markers or IMU, Barometer, etc
               };


    /// MEMBERS
    // Map
    OdoMap map;
    // state - init means triangulated
    State state;
    // control state set by user
    Control control;
    // Counts successive frames we could not do pose estimation on, when its abvoe a certain threshold we consider ourselves "lost"
    int lostCounter;
    // VO matches between last frame and key frame
    DMatches matchesVO;
    // all matches between last frame and key frame
    DMatches matches;
    // last computed disparity f vs f
    double disparity;
    // last computed disparity f vs map
    double disparityMap;
    // for outputting to tviz
    geometry_msgs::PoseArray trackPoses;
    visualization_msgs::Marker trackLines;


    /// SETTINGS
    // RELATIVE POSE - used by initialisation
    int voTriangulationMethod;
    bool voRelNLO;
    double voRelRansacThresh;
    int voRelRansacIter;
    int voRelPoseMethod;    
    int voAbsPoseMethod;
    int voAbsRansacIter;
    int lostThresh;
    double voAbsRansacThresh;
    bool voAbsNLO;
    double voInitDisparity;   // disparity required to trigger initialisation
    double voKfDisparity;     // disparity required to trigger new KF
    float frameQualityThreshold;
    float keyFrameQualityThreshold;
    BaselineMethod voBaselineMethod;
    double voBaseline;

    /// META
    // just for drawing
    double baselineInitial;
    double baselineCorrected;
    double timeVO;
    double timeMA;





    /// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// META FUNCTIONS /////////////////////////////////////////////////////////////////////////////////////////////////////
    // getter, setters, time keeping, drawing, etc


    /// Sets a state and shows a message if it changed
    void setState(State s){
        if (s!=state){
            ROS_INFO("ODO [M] = Changing state from [%s to %s]", getStateName(state).c_str(), getStateName(s).c_str());
            state = s;
        }
    }

    // If no argument is supplied, it returns the current state as a string. Else retures the state string specified
    std::string getStateName(int spec=-1){
        if (spec<0){
            spec = state;
        }
        switch(spec){
            case ST_WAIT_FIRST_FRAME: return "WAIT_FIRST";
            case ST_WAIT_INIT:        return "WAIT_INIT";
            case ST_TRACKING:         return "TRACKING";
            case ST_LOST:             return "LOST";
            default: ROS_ASSERT(0);   return "ERROR";
        }
    }

    // Returns the current state
    State getState() const {
        return state;
    }





    /// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// ODOMETRY UTILITY FUNCTIONS ///////////////////////////////////////////////////////////////////////////////////////
    // Self contained utility functions that do not change the class (should all be const)


    /// Triangulate points points3d using bearing vectors bv1 and bv2 and the estimated pose between them trans1to2.
    void triangulate(const Pose& trans1to2, const Bearings& bv1, const Bearings& bv2, Points3d& points3d) const{
        /// The 3D points are expressed in the frame of the first viewpoint.
        ros::WallTime t0 = ros::WallTime::now();
        ROS_INFO("OVO [U] > Triangulating");


        // Triangulate point pairs of each match
        opengv::relative_pose::CentralRelativeAdapter adapter(bv1, bv2, trans1to2.translation(), trans1to2.linear() );

        points3d.clear();
        points3d.reserve(bv1.size());

        if (voTriangulationMethod==0){
            //dont do
        } else if (voTriangulationMethod==1){
            for(uint i = 0; i < bv1.size(); ++i){
                points3d.push_back(opengv::triangulation::triangulate(adapter,i));
            }
        } else if (voTriangulationMethod==2){
            for(uint i = 0; i < bv1.size(); ++i){
               points3d.push_back(opengv::triangulation::triangulate(adapter,i));
            }
        }
        ROS_INFO("OVO [U] < Triangulated in [%f.1ms]",1000*(ros::WallTime::now()-t0).toSec() );
    }


/*
    Ints reprojectFilter(const Bearings& bv, const Points3d& pts3dFrame, OVO::BEARING_ERROR method = OVO::BVERR_NormAminusB){
        /// If the angle between bv[i] and pts3dFrame[i] is close enough, return i
        // bv is the bearing vector of a frame F
        // pts3dFrame are the points in frame F that the bearing vectors point to. They must be aligned
        ROS_ASSERT_MSG(bv.size() == pts3dFrame.size(), "Points must be aligned to bearing vectors");

        ROS_INFO("ODO > Projecting [%lu] points and comparing to bearing vectors",pts3dFrame.size());
        Eigen::VectorXd repjerr = OVO::reprojectErrPointsVsBV(pts3dFrame, bv, method);
        Ints inliers;
        inliers.reserve(pts3dFrame.size());
        for (uint i=0; i<repjerr.size(); ++i){
            if (repjerr[i]<voAbsRansacThresh){
                inliers.push_back(i);
            }
        }
        ROS_WARN_COND(inliers.size()< 15, "ODO = Very few points passed reprojection test");
        ROS_INFO("ODO = Min / Avg / Max RePrjErr  F = [%f, %f, %f] [Thresh: %f]", repjerr.minCoeff(), repjerr.mean(), repjerr.maxCoeff(), voAbsRansacThresh);
        ROS_INFO("ODO < Successfully projected [%lu/%lu] points",inliers.size(), pts3dFrame.size());
        return inliers;

    }
*/







    /// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// LOW LEVEL ODOMETRY FUNCTIONS ///////////////////////////////////////////////////////////////////////////////////////
    // Update local variables, perform VO methods, called by medium level functions


    /// adds a keyframe to the map
    // returns true on success
    bool addKf(FramePtr f){
        /// Start timer, show messages, check quality
        ros::WallTime t0 = ros::WallTime::now();
        ROS_INFO(OVO::colorise("ODO > ATTEMPTING TO MAKE FRAME [%d] KEYFRAME",OVO::FG_WHITE, OVO::BG_GREEN).c_str(), f->getId());
        const float quality = f->getQuality();
        if (quality < keyFrameQualityThreshold && quality>=0){
            ROS_WARN("ODO < FAILED TO ADD KEYFRAME, quality too bad [%f]", quality);
            return false;
        }


        double t=0;


        /// match against map, getting possible observations
        Landmark::Ptrs lms;

        disparity = map.match2Map(f, matches, lms, t); timeMA += t;

        // Show common observations
        std::map<FramePtr,int> kfObsCounter;
        for (uint i=0; i<matches.size(); ++i){
            ++kfObsCounter[lms[matches[i].trainIdx]->getObservationFrame()]; //histogram, count which kfs we f shared observations which
        }
        ROS_INFO(OVO::colorise("MAP = MATCHED Shared Observations with with Frame [%d|%d]:", OVO::FG_BLUE).c_str(), f->getId(), f->getKfId());
        ROS_INFO("   KEYFRAME   | OBS NR");
        for(std::map<FramePtr,int>::const_iterator it=kfObsCounter.begin(); it!=kfObsCounter.end(); ++it) {
            ROS_INFO("   [%3d|%4d] | %3d/%d",it->first->getId(), it->first->getKfId(), it->second,  it->first->getLandmarkRefNr());
        }




        // Check we could match enough
        if (matches.size()<5){
             ROS_WARN("ODO < FAILED TO ADD KEYFRAME, Not enough matches [%lu] after [%.1fms]", matches.size(),(ros::WallTime::now()-t0).toSec()*1000.);
             return false;
        }



        /// RECOMPUTE POSE USING RANSAC; KEEP INLIERS
        /// THIS IS OPTIONAL!
        bool reprojectMapMatches = true;
        if (reprojectMapMatches){
            ROS_INFO("ODO > Filtering [%lu] Matches with Ransac", matches.size());
            ros::WallTime t1 = ros::WallTime::now();
            matchesVO.clear();

            /// Get bearing vectors from current frame aligned with 3d land marks from keyframe aligned with matches
            // Indicies
            Ints fInd, mapInd;
            OVO::match2ind(matches, fInd, mapInd);
            // Bearings
            Bearings bvFMatched;
            OVO::matReduceInd(f->getBearings(), bvFMatched, fInd);
            // 3d points
            Points3d worldPts;
            OVO::landmarks2points(lms, worldPts, mapInd);


            /// DO ransac stuff
            // ransac output
            opengv::absolute_pose::CentralAbsoluteAdapter adapter(bvFMatched, worldPts);
            Ints inliers;
            if (voAbsPoseMethod==0){
                ROS_INFO("ODO = Using Estimated Pose as Prior");
                adapter.setR(f->getPose().linear());
            }
            boost::shared_ptr<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>absposeproblem_ptr(
                        new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(adapter,
                            static_cast<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::Algorithm>(voAbsPoseMethod)) );
            opengv::sac::Ransac<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
            ransac.sac_model_ = absposeproblem_ptr;
            ransac.threshold_ = voAbsRansacThresh*2; /// TODO: this should be a new dynamic reconfigure variable!
            ransac.max_iterations_ = voAbsRansacIter;

            ROS_INFO("ODO > Computing RANSAC absolute pose estimate vs MAP over [%lu matches] with [threshold = %f] ", matches.size(), ransac.threshold_);
            ransac.computeModel(1);

            // Set as output
            Pose transWtoF;
            transWtoF = ransac.model_coefficients_;
            std::swap(inliers, ransac.inliers_);
            OVO::vecReduceInd<DMatches>(matches, matchesVO, inliers);


            if (inliers.size()<5){
                ROS_WARN("ODO < RANSAC failed on Absolute Pose Estimation vs MAP with %lu/%lu inliers [%d iterations] [%.1fms]", inliers.size(), matches.size(),  ransac.iterations_, (ros::WallTime::now()-t1).toSec()*1000.);
                return false;
            } else {
                ROS_INFO("ODO < Absolute Ransac vs MAP done with %lu/%lu inliers [%d iterations] [%.1fms]", inliers.size(), matches.size(), ransac.iterations_,  (ros::WallTime::now()-t1).toSec()*1000.);
            }

            if (voAbsNLO){
                ROS_INFO("ODO > Doing NLO on Absolute Pose");
                adapter.sett(transWtoF.translation());
                adapter.setR(transWtoF.linear());
                ros::WallTime tNLO = ros::WallTime::now();
                Pose before = transWtoF;
                //Compute the pose of a viewpoint using nonlinear optimization. Using all available correspondences. Works for central and non-central case.
                //in:  adapter holding bearing vector to world point correspondences, the multi-camera configuration, plus the initial values.
                //out: Pose of viewpoint (position seen from world frame and orientation from viewpoint to world frame, transforms points from viewpoint to world frame).
                transWtoF = opengv::absolute_pose::optimize_nonlinear(adapter, inliers) ;
                ROS_INFO("ODO < NLO Finished in [%.1fms]", (ros::WallTime::now()-tNLO).toSec()*1000);
                ROS_INFO_STREAM("ODO = NLO Difference:\n  " << (before.inverse()*transWtoF).matrix());
            }

            // Update Pose

            Pose before = f->getPose();
            f->setPose(transWtoF);
            ROS_INFO_STREAM("ODO = Updated frame ["<<f->getId()<<"|"<<f->getKfId()<<"] pose. Difference:\n"<<(before.inverse()*transWtoF).matrix());
            ROS_INFO("ODO < Kept [%lu/%lu] = %f%% after ransac filtering in [%.1fms]", matchesVO.size(), matches.size(), static_cast<float>(matchesVO.size())/matches.size()*100.f, (ros::WallTime::now()-t1).toSec()*1000.);

            // Check we have enough matches left
            if (matchesVO.size()<5){
                 ROS_WARN("ODO < FAILED TO ADD KEYFRAME, Not enough matchesVO after filtering [%lu] after [%.1fms]", matchesVO.size(),(ros::WallTime::now()-t0).toSec()*1000.);
                 return false;
            }

         } else {// optional reprojectMapMatches
             matchesVO = matches;
         }



        /// Add observations to landmarks
        ROS_INFO("ODO = Adding [%lu] landmark observations from Frame [%d|%d]", matchesVO.size(), f->getId(), f->getKfId());
        std::map<FramePtr,int> kfObsCounterVO; //TODO: use std::map<FramePtr,int> kfObsCounter; // with key ->getId()
        for (uint i=0; i<matchesVO.size(); ++i){
            Landmark::Ptr lm = lms[matchesVO[i].trainIdx];
            f ->addLandMarkRef(matchesVO[i].queryIdx, lm);
            lm->addObservation( f, matchesVO[i].queryIdx);
            ++kfObsCounterVO[lm->getObservationFrame()]; //histogram, count which kfs we f shared observations which
        }
        ROS_INFO(OVO::colorise("ODO = ACTUAL Shared Observations with with Frame [%d|%d]:", OVO::FG_BLUE).c_str(), f->getId(), f->getKfId());
        ROS_INFO("   KEYFRAME   | OBS NR");
        for(std::map<FramePtr,int>::const_iterator it=kfObsCounterVO.begin(); it!=kfObsCounterVO.end(); ++it) {
            ROS_INFO("   [%3d|%4d] | %3d/%3d (%3d)",it->first->getId(), it->first->getKfId(), it->second, kfObsCounter[it->first], it->first->getLandmarkRefNr());
        }


        /// TRIANAGULATE NEW POINTS
        /// TODO (check baseline)
        // TODO: if this is fast enough we could loop through kfs

        ROS_INFO("ODO = Attempting to introduce new points");
        // match vs current kf
        DMatches matchesTri;
        double matchesTriTime;
        FramePtr kf = map.getLatestKF();
        double disparityTri = map.matchTriangulate(f, kf,matchesTri, matchesTriTime);

        ROS_INFO("ODO = [%lu] potential candidates matched with [%f] disparity, triangulating", matchesTri.size(), disparityTri);
        // triangulate
        Points3d points3d;
        Bearings bvF;
        Bearings bvKF;
        OVO::alignedBV(f->getBearings(), kf->getBearings(), matchesTri, bvF, bvKF);
        /// TODO: check baseline
        triangulate(kf->getPose().inverse()*f->getPose(), bvKF, bvF, points3d);

        // Reproject
        ROS_INFO("ODO = Triangulated [%lu] potential points, checing reprojection error. Thresh [%f]", points3d.size(),voAbsRansacThresh );
        Doubles error = OVO::reprojectErrPointsVsBV(points3d, bvKF, DMatches());
        DMatches matchesTriInlier;
        Ints inliers;
        for (uint i= 0; i<error.size(); ++i){
            if (error[i]<voAbsRansacThresh){ /// TODO: own dynamic reconf var here
                inliers.push_back(i);
            }
        }
        ROS_INFO("ODO = Successfully Projected [%lu/%lu] potential points", inliers.size(),points3d.size());

        // Extract inliers
        OVO::vecReduceInd(points3d, inliers);
        OVO::vecReduceInd(matchesTri, matchesTriInlier, inliers);
        OVO::transformPoints(kf->getPose(), points3d);
        map.pushKFWithLandmarks(f, points3d, matchesTriInlier);

        /// FOr every candidate keyframe, project map points and try to match. If successful, add observations
        /// Repeat with different keyframe pair

        // Add to map
        double time = (ros::WallTime::now()-t0).toSec();
        timeVO += time;

        ROS_INFO("ODO < KEYFRAME ADDED [%d|%d] in [%1.fms]", f->getId(), f->getKfId(), time*1000);
        return true;

    }


    /// Attempts to estiamte the pose with 2d-3d estiamtes
    bool absolutePose(FramePtr f){

        FramePtr kf = map.getLatestKF();
        ROS_INFO("ODO [L] > Doing VO Pose Estimate Frame [%d|%d] vs KeyFrame [%d|%d] with [%lu] matches", f->getId(), f->getKfId(), kf->getId(), kf->getKfId(), matches.size());

        matchesVO.clear();

        if (matches.size()==0){
            ROS_ERROR("ODO [L] < CANNOT DO Pose Estiamte without matches");
            return false;
        }


        // TWO OPTIONS - get landmarks via frame
        //             - get landmarks directly from map




        //ROS_INFO("ODO = Before Alignment [%lu Matches] [%ld bearings] [%lu world points] [%lu kf vo inliers]", matches.size(), bvf.rows(), worldPtsKF.size(), kf->getVoInliers().size());

        /// Get bearing vectors from current frame aligned with 3d land marks from keyframe aligned with matches
        // Indicies
        Ints fInd, kfInd;
        OVO::match2ind(matches, fInd, kfInd);

        // Bearings
        Bearings bvFMatched;
        OVO::matReduceInd(f->getBearings(), bvFMatched, fInd);

        // 3d points
        Points3d worldPts;
        const Landmark::Ptrs& landmarks = kf->getLandmarkRefs();
        OVO::landmarks2points(landmarks, worldPts, kfInd);

        //ROS_INFO("ODO = AFTER Alignment [%lu Matches] [%lu bearings] [%lu world points]", matches.size(), bvFMatched.size(), worldPts.size());



        /// DO ransac stuff
        // ransac output
        opengv::absolute_pose::CentralAbsoluteAdapter adapter(bvFMatched, worldPts);
        Ints inliers;




        ros::WallTime t0 = ros::WallTime::now();
        if (voAbsPoseMethod==0){            
            ROS_INFO("ODO = Using Relative Rotation Prior");
            ROS_ASSERT_MSG(USE_IMU, "Cannot use p2pIMU without IMU measurements!");
            /// Compute relative rotation using current and previous imu data
            Eigen::Matrix3d relRot;
            OVO::relativeRotation(kf->getImuRotation(), f->getImuRotation(), relRot);
            adapter.setR(kf->getPose().linear() * relRot);
        }


        /// DO RANSAC

        boost::shared_ptr<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>absposeproblem_ptr(
                    new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(adapter,
                        static_cast<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::Algorithm>(voAbsPoseMethod)) );
        opengv::sac::Ransac<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
        ransac.sac_model_ = absposeproblem_ptr;
        ransac.threshold_ = voAbsRansacThresh;
        ransac.max_iterations_ = voAbsRansacIter;

        ROS_INFO("ODO > Computing RANSAC absolute pose estimate over [%lu matches] with [threshold = %f] ", matches.size(), ransac.threshold_);
        ransac.computeModel(1);


        // Set as output
        Pose transWtoF;
        transWtoF = ransac.model_coefficients_;
        std::swap(inliers, ransac.inliers_);
        OVO::vecReduceInd<DMatches>(matches, matchesVO, inliers);


        if (inliers.size()<10){
            ROS_WARN("ODO < RANSAC failed on Absolute Pose Estimation with %lu/%lu inliers [%d iterations] [%.1fms]", inliers.size(), matches.size(),  ransac.iterations_, (ros::WallTime::now()-t0).toSec()*1000.);
            return false;
        } else {
            ROS_INFO("ODO < Absolute Ransac Done with %lu/%lu inliers [%d iterations] [%.1fms]", inliers.size(), matches.size(), ransac.iterations_,  (ros::WallTime::now()-t0).toSec()*1000.);
        }


        if (voAbsNLO){
            ROS_INFO("ODO > Doing NLO on Absolute Pose");
            adapter.sett(transWtoF.translation());
            adapter.setR(transWtoF.linear());
            ros::WallTime tNLO = ros::WallTime::now();
            Pose before = transWtoF;
            //Compute the pose of a viewpoint using nonlinear optimization. Using all available correspondences. Works for central and non-central case.
            //in:  adapter holding bearing vector to world point correspondences, the multi-camera configuration, plus the initial values.
            //out: Pose of viewpoint (position seen from world frame and orientation from viewpoint to world frame, transforms points from viewpoint to world frame).

            transWtoF = opengv::absolute_pose::optimize_nonlinear(adapter, inliers) ;
            ROS_INFO("ODO < NLO Finished in [%.1fms]", (ros::WallTime::now()-tNLO).toSec()*1000);
            ROS_INFO_STREAM("ODO = NLO Difference:\n  " << (before.inverse()*transWtoF).matrix());
        }


        f->setPose(transWtoF);


        /// Compute disparity of VO inliers



        // Compute disparity of previous frame vs current frame inliers
        Doubles errorF;
        // this is disparity vs map inliers
//        Doubles errorM;
//        const Eigen::Affine3d inverseSolution = transWtoF.inverse();
        errorF.reserve(matchesVO.size());
//        errorM.reserve(matchesVO.size());
        Eigen::MatrixXd qBV =  f->getBearings();
        const Eigen::MatrixXd& tBV = kf->getBearings();


//        // Error between measured bearing vector and estimated bearing vector
//        for(uint i=0; i<matchesVO.size(); ++i){
//            errorM.push_back(OVO::errorNormalisedBV(qBV.block<1,3>(matchesVO[i].queryIdx,0), (inverseSolution * worldPts[matchesVO[i].trainIdx]).normalized(), OVO::BVERR_OneMinusAdotB));
//        }

        // Unrotate bearing vectors to get mroe accurate disparity reading
        if (USE_IMU){
            Eigen::Matrix3d relRot;
            OVO::relativeRotation(kf->getImuRotation(), f->getImuRotation(), relRot); // BV_kf = R*BV_f = BV_f'*R'
            qBV *= relRot.transpose();
        }

        // Error between F and KF bearing vectors
        for(uint i=0; i<matchesVO.size(); ++i){
            errorF.push_back(OVO::errorNormalisedBV(qBV.block<1,3>(matchesVO[i].queryIdx,0),tBV.block<1,3>(matchesVO[i].trainIdx,0), OVO::BVERR_OneMinusAdotB));
        }
//        disparityMap = OVO::medianApprox<double>(errorM);
        disparity = OVO::medianApprox<double>(errorF);







        /// JUST FOR VIS!
        /// Align inliers

//        Bearings bvFVO;
//        Points3d worldPtsKFVO;
//        OVO::vecReduceInd<Bearings>(bvFMatched, bvFVO, inliers);

//        OVO::vecReduceInd<Points3d>(worldPts, worldPtsKFVO, inliers);
//        const Eigen::VectorXd repjerr = OVO::reprojectErrPointsVsBV(f->getPose(), worldPtsKFVO, bvFVO );
//        ROS_INFO("ODO = Min / Avg / Max RePrjErr  F = [%f, %f, %f]", repjerr.minCoeff(), repjerr.mean(), repjerr.maxCoeff());

        /*
        Ints FVOInd, KFVOInd;
        OVO::match2ind(matchesVO, FVOInd, KFVOInd);
        f->setWorldPoints(FVOInd, worldPtsKFVO, false); // No need for VO only mode
    */

        ROS_INFO(OVO::colorise("ODO [L] < Pose Estimated [%lu/%lu] inliers, Disparity FvsKF: [%f]" /*" FvsMap: [%f]"*/,OVO::FG_BLUE).c_str(), matches.size(), matchesVO.size(), disparity/*, disparityMap*/);
        return true;
    }


    /// Attempts to initialise VO between two frames, data associsation through matches, Sets the pose of frame f
    bool relativePoseInitialisation(FramePtr f, FramePtr kf, const DMatches& matches){
        ROS_INFO("ODO > Doing VO Initialisation");


        Points3d points3d;
        matchesVO.clear();

        /// Get unit features aligned
        Bearings bvF;
        Bearings bvKF;
        OVO::alignedBV(f->getBearings(), kf->getBearings(), matches, bvF, bvKF);
        opengv::relative_pose::CentralRelativeAdapter adapter(bvKF, bvF);
        Ints inliers;
        int iterations=-1;

        /// Compute relative transformation from KeyFrame KF to Frame F using ransac
        Pose transKFtoF;

        ros::WallTime t0 = ros::WallTime::now();
        if (voRelPoseMethod==4){
            /// Use IMU for rotation, compute translation
            ROS_INFO("Using IMU for relative rotation");
            // compute relative rotation
            Eigen::Matrix3d relativeRot;
            OVO::relativeRotation(kf->getImuRotation(), f->getImuRotation(), relativeRot);
            adapter.setR12(relativeRot);


            ///DO RANSAC for translation only
            boost::shared_ptr<RP::TranslationOnlySacProblem>relposeproblem_ptr(new RP::TranslationOnlySacProblem(adapter) );
            opengv::sac::Ransac<RP::TranslationOnlySacProblem> ransac;
            ransac.sac_model_ = relposeproblem_ptr;
            ransac.threshold_ = voRelRansacThresh;
            ransac.max_iterations_ = voRelRansacIter;
            ransac.computeModel(1);

            // Set as output
            iterations = ransac.iterations_;
            transKFtoF = ransac.model_coefficients_;
            std::swap(inliers, ransac.inliers_);
        } else {
            /// Compute R and T directly
            // DO RANSAC
            boost::shared_ptr<RP::
                    CentralRelativePoseSacProblem>relposeproblem_ptr( new RP::CentralRelativePoseSacProblem(adapter,
                            static_cast<RP::CentralRelativePoseSacProblem::Algorithm>(voRelPoseMethod)) );

            // create a RANSAC object and run
            opengv::sac::Ransac<RP::CentralRelativePoseSacProblem> ransac;
            ransac.sac_model_ = relposeproblem_ptr;
            ransac.threshold_ = voRelRansacThresh;
            ransac.max_iterations_ = voRelRansacIter;
            ransac.computeModel(1);

            // set output
            iterations = ransac.iterations_;
            transKFtoF = ransac.model_coefficients_;
            std::swap(inliers, ransac.inliers_);  //Get inliers
        }

        if (inliers.size()<10){
            ROS_WARN("ODO = RANSAC failed on Relative Pose Estimation with %lu/%lu inliers after [%d iterations] in [%.1fms]", inliers.size(), matches.size(), iterations, (ros::WallTime::now()-t0).toSec()*1000.);
            return false;
        } else {
            ROS_INFO("ODO = Relative Ransac Done with %lu/%lu inliers [%d iterations] in [%.1fms]", inliers.size(), matches.size(), iterations,(ros::WallTime::now()-t0).toSec()*1000.);
        }

        /// Experimental: NLO
        /// TODO: estimate yaw bias!
        if (voRelNLO){
            // check order
            ROS_INFO("ODO > Doing NLO on Relative Pose");
            ros::WallTime tNLO = ros::WallTime::now();
            Pose before = transKFtoF;
            adapter.sett12(transKFtoF.translation());
            adapter.setR12(transKFtoF.linear());
            transKFtoF = opengv::relative_pose::optimize_nonlinear(adapter, inliers);
            ROS_INFO("ODO < NLO Finished in [%.1fms]", (ros::WallTime::now()-tNLO).toSec()*1000);
            ROS_INFO_STREAM("ODO = NLO Difference:\n  " << (before.inverse()*transKFtoF).matrix());            
        }


        //printRPYXYZ(poseTF, "Ransac Output: ");


        ros::WallTime tTri = ros::WallTime::now();
        /// Align inliers
        Bearings bvFinlier;
        Bearings bvKFinlier;
        OVO::vecReduceInd<Bearings>(bvF, bvFinlier, inliers);
        OVO::vecReduceInd<Bearings>(bvKF, bvKFinlier, inliers);
        OVO::vecReduceInd<DMatches>(matches, matchesVO, inliers);

        /// Scale Pose
        baselineInitial = transKFtoF.translation().norm();

        if (USE_SYNTHETIC){
            ROS_WARN("ODO [SYN] = Manually adjusting the baseline");
            const FrameSynthetic* fSyn  = static_cast<FrameSynthetic*>(&*f); //ugly - has to be a better way..
            const FrameSynthetic* kfSyn = static_cast<FrameSynthetic*>(&*kf);
            Pose transKFtoFSyn = kfSyn->getGroundTruthCamPose().inverse() * fSyn->getGroundTruthCamPose();
            double baselineSyn =  transKFtoFSyn.translation().norm();
            transKFtoF.translation() *= 1.0/baselineInitial * baselineSyn;
        }

        /// TRIANGULATE points in the keyframe frame
        ROS_INFO("ODO = Generating new point cloud. Relative Pose baseline [%f]", baselineInitial);
        double meanDepth=0.0;
        switch (voBaselineMethod){
            case BL_UNCHANGED:
                ROS_INFO("ODO = Baseline left unchanged [%f]", baselineInitial);
                triangulate(transKFtoF, bvKFinlier, bvFinlier, points3d);
                break;
            case BL_FIXED:
                ROS_INFO("ODO = Baseline scaled [1.0]");
                transKFtoF.translation() *= 1.0/baselineInitial;
                triangulate(transKFtoF, bvKFinlier, bvFinlier, points3d);
                break;
            case BL_MANUAL_BASELINE:
                ROS_INFO("ODO = Baseline selected scaled [%f]", voBaseline);
                transKFtoF.translation() *= voBaseline/baselineInitial;
                triangulate(transKFtoF, bvKFinlier, bvFinlier, points3d);
                break;
            case BL_MANUAL_AVGDEPTH:
                ROS_INFO("ODO > Scaling Baseline so avg depth is [%f]", voBaseline);
                transKFtoF.translation() /= baselineInitial;
                // Triangulate with baseline = 1
                triangulate(transKFtoF, bvKFinlier, bvFinlier, points3d);
                // calculate mean depth
                for (uint i=0; i<points3d.size(); ++i){
                   meanDepth += points3d[i].norm();
                }
                meanDepth /=static_cast<double>(points3d.size());
                ROS_INFO("ODO = Baseline is [%f] with mean depth [%f]", baselineInitial, meanDepth);
                // modify points
                for (uint i=0; i<points3d.size(); ++i){
                    points3d[i] /= meanDepth/voBaseline;
                }
                // modify baseline
                transKFtoF.translation()/= meanDepth/voBaseline;
                ROS_INFO("ODO < Baseline updated to [%f] with mean depth [%f]", transKFtoF.translation().norm(), voBaseline);
                break;
            case BL_AUTO_BASELINE:
                ROS_ERROR("ODO = NOT IMPLEMENTED: Baseline being scaled using Markers");
                transKFtoF.translation()*= 1.0/baselineInitial;
                triangulate(transKFtoF, bvKFinlier, bvFinlier, points3d);
                break;

        }

        // just for drawing really
        baselineCorrected = transKFtoF.translation().norm();
        meanDepth = 0;
        for (uint i=0; i<points3d.size(); ++i){
           meanDepth += points3d[i].norm();
        }
        meanDepth /=static_cast<double>(points3d.size());
        ROS_INFO("ODO = Mean depth [%f]", meanDepth);
        ROS_INFO("ODO < New point cloud generated in [%.1fms]", (ros::WallTime::now()-tTri).toSec()*1000);


        /// Set pose of F from KF->F to world->F
        //f2->setPose(); //
        //f->setPose(kf->getPose() * transKFtoF);
        Pose transWtoF = kf->getPose() * transKFtoF; // should be this one
        //Pose transWtoF = transKFtoF * kf->getPose();
        f->setPose(transWtoF);

        /// print reprojection error stats
        //reprojectFilter(bvKFinlier, points3d);
        Doubles errors = OVO::reprojectErrPointsVsBV(points3d, bvKFinlier);
        disparityMap = OVO::medianApprox<double>(errors); //WARNING: median partially sorts, so the container is acutally changed

        /// put points from KF->Points frame to World->Points frame
        OVO::transformPoints(kf->getPose(), points3d);

        /// Update map
        map.initialiseMap(f, points3d, matchesVO);

        ROS_INFO("ODO < Initialised");
        return true;
    }





    /// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// MEDIUM LEVEL ODOMETRY FUNCTIONS ////////////////////////////////////////////////////////////////////////////////////
    // These functions should return booleans and should not change the state machine state. They call low level functions

    /// resets all internal structures
    void resetVO(){
        ROS_INFO("ODO [M] > RESETTING");
        setState(ST_WAIT_FIRST_FRAME);
        control = CTR_DO_NOTHING;
        baselineInitial = -1;
        baselineCorrected = -1;
        timeVO = -1;
        timeMA = -1;
        disparity = -1;
        disparityMap = -1;
        //map.reset();
        matchesVO.clear();
        matches.clear();
        trackPoses.poses.clear();
        trackLines.points.clear();
        lostCounter = 0;
        ROS_INFO("ODO [M] < RESET");
    }


    /// Adds the first KF
    bool setInitialKFVO(FramePtr frame){
        ROS_INFO("ODO [M] > SETTING INITIAL KEYFRAME");
        resetVO();

        const float quality = frame->getQuality();
        if (quality < keyFrameQualityThreshold && quality>=0){
            ROS_WARN("ODO [M] < FAILED TO ADD INITIAL KEYFRAME, quality too bad [%f < %f]", quality, keyFrameQualityThreshold);
            return false;
        }
        // add first keyframe
        map.pushKF(frame, true);
        ROS_INFO("ODO [M] < INITIAL KEYFRAME ADDED ");
        return true;
    }


    /// Computes 2d-2d matches vs latest KF
    bool trackVO(FramePtr frame){
        ROS_ASSERT(state==ST_WAIT_INIT || state == ST_TRACKING);
        // track against initial keyframe
        disparity = -1;
        ROS_INFO("ODO [M] > TrackVO - Tracking gainst [%s] Keyframe", state==ST_WAIT_INIT? "FIRST":"LAST");

        if (state == ST_WAIT_INIT){
            // tracking against first keyframe
            disparity = map.match2KF(frame, matches, timeMA, false);
            // matches = ....
            // TODO
        } else {
            // normal tracking against last keyframe
            disparity = map.match2KF(frame, matches, timeMA, true);
            // matches = ....
            // TODO
        }

        if (disparity>=0){
            ROS_INFO("ODO [M] < Tracking Success. Disparity = [%f]", disparity);
            return true;
        } else {
            ROS_INFO("ODO [M] < Tracking Failure. Disparity = [%f]", disparity);
            return false;
        }

    }


    /// Initialises the whole VO pipeline using predetermined 2d-2d matches. Does relativePose. Does Triangulation. Adds KF. Inits Map.
    bool initialiseVO(FramePtr frame){
        ROS_ASSERT(state==ST_WAIT_INIT);
        ROS_INFO("ODO [M] > ATTEMPTING INITIALISATION");
        disparityMap = -1;
        ros::WallTime t0 = ros::WallTime::now();


        /// RANSAC
        /// TRIANGULATE
        /// ADD KF
        /// UPDATE MAP
        bool okay = relativePoseInitialisation(frame, map.getLatestKF(), matches);

        timeVO = (ros::WallTime::now()-t0).toSec();

        if (okay){
            ROS_INFO("ODO [M] < INITIALISATION SUCCESS [%.1fms]", timeVO*1000.);
            return true;
        } else {
            ROS_WARN("ODO [M] < INITIALISATION FAIL [%.1fms]", timeVO*1000.);
            return false;
        }
    }


    /// Estiamte pose vs latest KF using predetermined 2d-3d matches
    bool estimatePoseVO(FramePtr frame){
        ROS_INFO("ODO [M] > DOING POSE ESTIMATION");
        ROS_ASSERT(state==ST_TRACKING);

        ros::WallTime t0 = ros::WallTime::now();

        // compute inlier only disparity
        disparity = -1;
        disparityMap = -1;
        bool okay = absolutePose(frame);

        timeVO = (ros::WallTime::now()-t0).toSec();
        if (okay){
            ROS_INFO("ODO [M] < POSE ESTIMATION SUCCESS [%.1fms]", timeVO*1000.);            
            return true;
        } else {
            ROS_WARN("ODO [M] < POSE ESTIMATION FAIL [%.1fms]", timeVO*1000.);            
            return false;
        }
    }


    /// Try to relocate against the map. Computes 2d-3d matches. Does absolutePose. Adds Kf. Updates Map
    bool relocateVO(FramePtr frame){
        ROS_INFO("ODO [M] > DOING RELOCALISATION");
        ROS_ASSERT(state==ST_LOST);
        ros::WallTime t0 = ros::WallTime::now();

        ROS_ERROR("NOT IMPLEMENTED");


        //lostCounter = 0; // found


        ++ lostCounter; // still lost


        ROS_WARN("ODO [M] < RELOCALISATION FAILED [%.1fms]", 1000* (ros::WallTime::now()-t0).toSec());
        return false;

    }






    /// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// HIGH LEVEL STATE MACHINE FUNCTIONS /////////////////////////////////////////////////////////////////////////////////
    // These functions change the state machine state and call the medium level functions

    /// We have not yet received any data
    // Valid Control:
    //    Nothing to control
    // Transition States
    //    FirstFrame <- if we dont add the frame as a KF (due to quality reasons for example)
    //    WaitInit   <- if we successfully add our first KF
    void voFirstFrame(FramePtr frame){
        ROS_INFO("ODO [H] > First Frame received");
        ROS_ASSERT(state==ST_WAIT_FIRST_FRAME);


        // No controls allowed here
        if (control != CTR_DO_NOTHING){
            control = CTR_DO_NOTHING;
            ROS_WARN("ODO = Cannot do any controls while in state [WAIT_FIRST]");
        }

        // Attempt to add keyframe
        if (setInitialKFVO(frame)){
            // First frame added
            setState(ST_WAIT_INIT);
            ROS_INFO("ODO [H] < First Frame added");
        } else {
            // Failed to add first frame
            setState(ST_WAIT_FIRST_FRAME);
            ROS_WARN("ODO [H] < Failed to add first frame");
        }
    }



    /// We have one keyframe and are trying to initialise against it. This is done if the previous disparity was big enough
    // Valid Control:
    //    RESET  -> Resets
    //    ADD_KF -> Force Keyframe addition
    // Transition States
    //    Tracking   <- If disparity was big enough and we successfully initialised (and added our second keyframe)    
    //    FirstFrame <- If control==RESET
    void voFirstFrameTrack(FramePtr frame){
        ROS_INFO("ODO [H] > Tracking against first KF");
        ROS_ASSERT(state==ST_WAIT_INIT);

        /// Check Global Reset
        if (control == CTR_DO_RESET){
            resetVO();
            ROS_WARN("ODO [H] < User Reset");
            return;
        }

        /// Do tracking vs first keyframe
        if (!trackVO(frame)) {
            // Failed to track
            ROS_WARN("ODO [H] < Failed to track for initialisation");
            setState(ST_WAIT_INIT);
            control = CTR_DO_NOTHING;
            return;
        }


        /// Add KF, Triangulate, initialise Map?
        if (control == CTR_DO_ADDKF || disparity >= voInitDisparity){
            control = CTR_DO_NOTHING;
            ROS_INFO(OVO::colorise("ODO [H] > Initialising [Disparity = %f/%f]",OVO::FG_MAGNETA).c_str(), disparity, voInitDisparity);
            if (initialiseVO(frame)){
                // Initialisation okay, start tracking
                setState(ST_TRACKING);
                ROS_INFO("ODO [H] < Initialisation Success");
            } else {
                // Init failed, keep trying
                setState(ST_WAIT_INIT);
                ROS_WARN("ODO [H] < Initialisation Fail");
                return;
            }
            // Does inititialisation
        }

    }



    /// We are currently tracking based on a pose estimate. Might trigger new KF based on previous disparity
    // Valid Control:
    //    RESET  -> Resets
    //    ADD_KF -> Force Keyframe addition
    // Transition States
    //    Tracking   <- If we compute a valid current pose or successfully add a new keyframe
    //    Lost       <- If we fail to localise or fail to add a keyframe
    //    FirstFrame <- If control==RESET
    void voTrack(FramePtr frame){
        ROS_WARN("ODO [H] > Tracking");
        ROS_ASSERT(state==ST_TRACKING);

        /// Check Global Reset
        if (control == CTR_DO_RESET){
            resetVO();
            ROS_WARN("ODO [H] < User Reset");
            return;
        }


        /// Do tracking vs last keyframe
        if (!trackVO(frame)) {
            // Failed to track
            lostCounter++;
            if (lostCounter>lostThresh){
                setState(ST_LOST);
            }
            ROS_WARN("ODO [H] < Tracking Failed: Failed to track [%d] last frames", lostCounter);
            control = CTR_DO_NOTHING;
            return;
        }

        /// Compute pose
        if (!estimatePoseVO(frame)){
            // Failed to compute pose
            ++lostCounter;
            if (lostCounter>lostThresh){
                setState(ST_LOST);
            }
            ROS_WARN("ODO [H] < Tracking fail: Failed to compute pose of [%d] last frames", lostCounter);
            control = CTR_DO_NOTHING;
            return;
        } else {
            if (lostCounter>0){
                ROS_INFO("ODO [H] Recovered after being lost for [%d] frames", lostCounter);
                lostCounter = 0;
            }
        }


        /// Add KF and and update Map
        if (control == CTR_DO_ADDKF || disparity >= voKfDisparity){
            control = CTR_DO_NOTHING;
            ROS_INFO("ODO [H] > Adding KF [Disparity = %f/%f", disparity, voKfDisparity);

            if (addKf(frame)){
                // Successfully added KF
                ROS_INFO("ODO [H] < KF added");
            } else {
                // Failed to add KF
                /// TODO: are we lost?
                //setState(ST_LOST);
                ROS_WARN("ODO [H] < Failed to add KF");
                ROS_WARN("ODO [H] < Tracking fail: Matching okay but KF could not be added");
                return;
            }
        }
        ROS_WARN("ODO [H] < Tracking Success");
    }



    /// We are lost and try to relocalise. If we manage to relocalise, make a new KF at that position
    // Valid Control:
    //    RESET -> Resets
    // Transition States
    //    Tracking   <- If we relocalise (adds a new KF)
    //    Lost       <- If we fail to relocalise
    //    FirstFrame <- If control==RESET
    void voRelocate(FramePtr frame){
        ROS_ASSERT(state==ST_LOST);
        ROS_WARN("ODO > We are LOST, trying to recover");

        /// Check Global Reset
        if (control == CTR_DO_RESET){
            resetVO();
            return;
        }

        if (control == CTR_DO_ADDKF){
            ROS_ERROR("ODO = Cannot add Keyframe while lost");
            control = CTR_DO_NOTHING;
        }

        /// Relocate, match, pose est, add kf
        if (relocateVO(frame)){
            // Successfully relocated and added KF
            setState(ST_TRACKING);
            lostCounter = 0;
            ROS_INFO("ODO < Relocalisation Success");
        } else {
            setState(ST_LOST);
            ROS_INFO("ODO < Relocalisation Failure");
        }
    }








public:
    /// ////////////////////////////////////////////////////////////////////////////////////// PUBLIC METHODS
    Odometry(){
        /// Init
        init_g2o_types();
        control = CTR_DO_NOTHING;
        state   = ST_WAIT_FIRST_FRAME;
        disparity = -1;
        matchesVO.clear();
        matches.clear();


        /// Meta
        baselineInitial = -1;
        baselineCorrected = -1;
        timeVO = -1;

        /// settings
        voRelRansacIter = 800;
        voRelRansacThresh = 4;
        voRelPoseMethod = 0;
        voTriangulationMethod = 1;
        voInitDisparity = 40;
        voAbsPoseMethod = 1;
        voAbsRansacIter = 800;
        voAbsRansacThresh = 4;
        frameQualityThreshold = 0.2;
        keyFrameQualityThreshold = 0.6;
        lostThresh = 10; // lost if 10 frames fail

        /// Set up output markers
        trackLines.ns = "TrackLines";
        trackLines.id = 0;
        trackLines.header.frame_id = WORLD_FRAME;
        trackLines.type = visualization_msgs::Marker::LINE_STRIP;
        trackLines.action = visualization_msgs::Marker::ADD;
        trackLines.scale.x = 0.01;
        trackLines.frame_locked = true;
        std_msgs::ColorRGBA col;
        const CvScalar RGB = CV_RGB(255,0,60);
        col.a = 0.7;
        col.r = RGB.val[2]/255;
        col.g = RGB.val[1]/255;
        col.b = RGB.val[0]/255;
        trackLines.color = col;


    }



    // a step in the VO pipeline. Main entry point for odometry
    void update(FramePtr frame){
        ROS_INFO("ODO > PROCESSING FRAME [%d] - Current in state [%s]", frame->getId(), getStateName().c_str());

        /// Skip frame immediatly if quality is too bad (should ust be used for extremely bad frames)
        const float quality = frame->getQuality();
        if (quality < frameQualityThreshold && quality>=0){
            ROS_WARN("ODO < SKIPPING FRAME [%d], Quality too poor [%f < %f]", frame->getId(), quality, frameQualityThreshold);
            cv::imshow("FAILED_QUALITY", frame->getVisualImage());
            cv::waitKey(20);
            return;
        }

        // STATE MACHINE

        switch (state) {
            case ST_WAIT_FIRST_FRAME:
                voFirstFrame(frame); // -> WAIT_INIT
                break;
            case ST_WAIT_INIT:
                voFirstFrameTrack(frame); // -> WAIT_INIT, TRACKING; RESET, ADD KF (replaces current), FORCE INIT
                break;
            case ST_TRACKING:
                voTrack(frame); // -> TRACKING, LOST; RESET, ADD KF
                break;
            case ST_LOST:
                voRelocate(frame); // -> LOST, TRACKING; RESET
                break;
        }

        if (frame->poseEstimated()){
            // pm points along x, which when plotting in the camera frame is useless, swap x and z
            geometry_msgs::Pose pm = frame->getPoseMarker(false, true);
            trackPoses.poses.push_back(pm);
            trackLines.points.push_back(pm.position);
        }

        ROS_INFO("ODO < PROCESSED FRAME [%d]", frame->getId());

    }


    // Gets a visual image of the current state
    cv::Mat getVisualImage(){
        ROS_INFO("ODO > GETTING VISUAL IMAGE [State: %s]", getStateName().c_str());
        // Get matching image
        cv::Mat image;
        if (state==ST_WAIT_FIRST_FRAME){
            image = cv::Mat::zeros(720,576,CV_8UC3);
            OVO::drawTextCenter(image, "NO IMG", CV_RGB(255,0,0), 4, 3);
            return image;
        }

        const FramePtr f = map.getCurrentFrame();
        const FramePtr kf = map.getLatestKF();
        cv::drawMatches(f->getVisualImage(), KeyPoints(0),kf->getVisualImage(), KeyPoints(0), DMatches(0), image);


        // Draw state
        OVO::drawTextCenter(image.colRange(0,image.cols/2).rowRange(0,60), getStateName(), CV_RGB(200, 200, 0), 1.5, 2);

        // Overlay baseline
        if (baselineInitial>=0){
            OVO::putInt(image, baselineInitial, cv::Point(10,9*25), CV_RGB(0,96*2,0), false , "BLD:");
        }
        if (baselineCorrected>=0){
            OVO::putInt(image, baselineCorrected, cv::Point(10,10*25), CV_RGB(0,96*2,0), false , "BLC:");
        }

        OVO::putInt(image, map.getKeyframeNr(), cv::Point2f(image.cols-95,6*25), CV_RGB(120,80,255), true,  "KFS:");
        OVO::putInt(image, map.getLandmarkNr(), cv::Point2f(image.cols-95,7*25), CV_RGB(120,80,255), true,  "LMS:");

        if (lostCounter>0){
            OVO::putInt(image, static_cast<float>(lostCounter)/lostThresh*100.f, cv::Point2f(image.cols/2-95,8*25), OVO::getColor(-lostCounter*2,lostThresh, lostCounter), true,  "LOST:","%");
        }


        /// THESE ONLY WORK WHEN THE CURRENT MATCHES WERE VS THE CURRENT KEYFRAME!
        if (kf->getId()==f->getId()){
            // THis is the case when we have just updated the map, so the current keyframe is actually also the latest kf

            if (map.getKeyframeNr()>1){
                const FramePtr prekf = map.getLatestKF(1);
                if (matches.size()>0){
                    OVO::drawFlow(image, f->getPoints(), prekf->getPoints(), matches, CV_RGB(0,200,0), 1.1);
                    OVO::putInt(image, matches.size(), cv::Point(10,2*25), CV_RGB(0,200,0),  true,"MA:");
                }
                if (matchesVO.size()>0){
                    OVO::drawFlow(image, f->getPoints(), prekf->getPoints(), matchesVO, CV_RGB(200,0,200),1.1);
                    OVO::putInt(image, matchesVO.size(), cv::Point(10,3*25), CV_RGB(200,0,200),  true,"VO:");
                }
            }



        } else {
            if (matches.size()>0){
                OVO::drawFlow(image, f->getPoints(), kf->getPoints(), matches, CV_RGB(0,200,0), 1.1);
                OVO::putInt(image, matches.size(), cv::Point(10,2*25), CV_RGB(0,200,0),  true,"MA:");
            }
            if (matchesVO.size()>0){
                OVO::drawFlow(image, f->getPoints(), kf->getPoints(), matchesVO, CV_RGB(200,0,200),1.1);
                OVO::putInt(image, matchesVO.size(), cv::Point(10,3*25), CV_RGB(200,0,200),  true,"VO:");
            }
        }

        // Project in world points
//        if (){
//        }

        // Project in estiamted point positions
//        if (){
//        }


        // show timings
        if (timeMA>0){
            OVO::putInt(image, timeMA*1000., cv::Point(10,image.rows-3*25), CV_RGB(200,0,200), false, "M:");
        }
        if (timeVO>0){
            OVO::putInt(image, timeVO*1000., cv::Point(10,image.rows-2*25), CV_RGB(200,0,200), false, "O:");
        }

        if (disparity>0){
            if (state==ST_WAIT_INIT){
                OVO::putInt(image,disparity/voInitDisparity*100, cv::Point(10,4*25), OVO::getColor(0,voInitDisparity,disparity),  true,"DF:","%");
            } else {
                OVO::putInt(image,disparity/voKfDisparity*100, cv::Point(10,4*25), OVO::getColor(0,voKfDisparity,disparity),  true,"DF:","%");
            }
        }
        if (disparityMap>0){
            OVO::putInt(image,disparityMap/voAbsRansacThresh*100, cv::Point(10,5*25), OVO::getColor(0,voAbsRansacThresh,disparityMap),  true,"DM:","%");

        }


        // show user controls
        // show flow
        // show tracker matches
        // show map statistics

        ROS_INFO("ODO < RETURNING VISUAL IMAGE");
        return image;

    }


    // Get all KFs
    const FramePtrs& getKeyFrames() const{
        return map.getKFs();
    }

    // Get the last KF
    FramePtr getLastFrame(){
        return map.getCurrentFrame();
    }

    // Get all the landmarks as markers for rviz
    const visualization_msgs::Marker getMapMarkers(){
        return map.getLandmarkMarkers();
    }
    const visualization_msgs::Marker getMapObservations(){
        return map.getLandmarkObservations();
    }

    const geometry_msgs::PoseArray& getTrackPoseMarker(){
        trackPoses.header.frame_id = WORLD_FRAME;
        trackPoses.header.stamp = ros::Time::now();
        ++trackPoses.header.seq;
        return trackPoses;
    }
    const visualization_msgs::Marker& getTrackLineMarker(){
        trackLines.header.stamp = ros::Time::now();
        ++trackLines.header.seq;
        return trackLines;
    }




    // Set paramters with dynamic reconfigure
    void setParameter(ollieRosTools::VoNode_paramsConfig &config, uint32_t level){
        ROS_INFO("ODO > [U] SETTING PARAMS");

        // Settings
        voRelRansacIter   = config.vo_relRansacIter;


        switch (config.vo_relPoseMethod){
            case 0: voRelPoseMethod = opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem::STEWENIUS; break;
            case 1: voRelPoseMethod = opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem::NISTER; break;
            case 2: voRelPoseMethod = opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem::SEVENPT; break;
            case 3: voRelPoseMethod = opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem::EIGHTPT; break;
            case 4: voRelPoseMethod = 4; break;
            default: ROS_ASSERT(0); break;
        }

        switch (config.vo_absPoseMethod){
            case 0: voAbsPoseMethod = opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::TWOPT; break; // central, with rotation prior
            case 1: voAbsPoseMethod = opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::KNEIP; break;
            case 2: voAbsPoseMethod = opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::GAO; break;
            case 3: voAbsPoseMethod = opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::EPNP; break;
            default: ROS_ASSERT(0); break;
        }





        voTriangulationMethod = config.vo_triMethod;
        voRelNLO          = config.vo_relNLO;        
        voBaselineMethod  = static_cast<BaselineMethod>(config.vo_relBaselineMethod);
        voBaseline        = config.vo_relBaseline;
        voAbsRansacIter   = config.vo_absRansacIter;        
        voAbsNLO          = config.vo_absNLO;


        voRelRansacThresh = OVO::px2error(config.vo_relRansacThresh); //1.0 - cos(atan(config.vo_relRansacThresh*sqrt(2.0)*0.5/720.0));
        voAbsRansacThresh = OVO::px2error(config.vo_absRansacThresh);// 1.0 - cos(atan(config.vo_absRansacThresh*sqrt(2.0)*0.5/720.0));
        voInitDisparity   = OVO::px2error(config.vo_initDisparity); //now in angles!
        voKfDisparity     = OVO::px2error(config.vo_kfDisparity); // now in angles!

        ROS_INFO("ODO [U] = VO Init Intialisation Disparity theshold: [%.1f] Pixels = [%.2f] Degrees = [%.6f] error", config.vo_initDisparity, OVO::px2degrees(config.vo_initDisparity), voInitDisparity );
        ROS_INFO("ODO [U] = VO Init New Keyframe Disparity theshold: [%.1f] Pixels = [%.2f] Degrees = [%.6f] error", config.vo_kfDisparity, OVO::px2degrees(config.vo_kfDisparity), voKfDisparity );
        ROS_INFO("ODO [U] = VO Absolute Disparity theshold: [%.1f] Pixels = [%.2f] Degrees = [%.6f] error", config.vo_absRansacThresh, OVO::px2degrees(config.vo_absRansacThresh), voAbsRansacThresh );
        ROS_INFO("ODO [U] = VO Relative Disparity theshold: [%.1f] Pixels = [%.2f] Degrees = [%.6f] error", config.vo_relRansacThresh, OVO::px2degrees(config.vo_relRansacThresh), voRelRansacThresh );




        // User controls
//        if (config.vo_doInitialisation){
//            control = CTR_DO_INIT;
//            config.vo_doInitialisation = false;
//        } else
        if (config.vo_setKeyFrame){
            control = CTR_DO_ADDKF;
            config.vo_setKeyFrame = false;
        } else if (config.vo_doReset){
            control = CTR_DO_RESET;
            config.vo_doReset = false;
        }

        if (config.writeCSV){
            ROS_ERROR("ODO [U] = NOT IMPLEMENTED WRITE CSV FUNCTIONALITY - NON CRITICAL");
            config.writeCSV = false;
        }



        map.setParameter(config, level);

        ROS_INFO("ODO [U] < PARAMS SET");
    }
};
#endif // ODOMETRY_HPP
