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
    // Keeps track of the internal state of the VO pipeline
    enum State {WAIT_FIRST_FRAME, // waiting for the first ever keyframe
                WAIT_INIT, // waiting for initial initialisation
                INITIALISED,      // Initialised and triangulated points
                LOST       // Initialised by no reference to keyframe
               };
    // Allows the user to manually initiate steps in the pipeline
    enum Control {DO_NOTHING, // No user input
                  DO_INIT,    // Force initialisation
                  DO_ADDKF,   // Force adding KF (if initialised, this adds one, if not, it replaces the current)
                  DO_RESET
               };
    enum BaselineMethod {UNCHANGED=-1,       // Baseline is not altered
                         FIXED,           // Baseline fixed to one meter
                         MANUAL_BASELINE, // Baseline specified by the user
                         MANUAL_AVGDEPTH, // Baseline adjusted so the specified avg depth is reached
                         AUTO_MARKER      // Baseline is estimated absoluted, eg by known markers
               };

    /// ////////////////////////////////////////////////////////////////////////////////////// PRIVATE MEMBERS
    // Map
    OdoMap map;
    // state - init means triangulated
    State state;
    // control state set by user
    Control control;
    // VO matches between last frame and key frame
    DMatches matchesVO;

    bool nextAddKf;

    /// SETTINGS
    // RELATIVE POSE - used by initialisation
    int voTriangulationMethod;
    bool voRelNLO;
    double voRelRansacThresh;
    int voRelRansacIter;
    int voRelPoseMethod;    
    int voAbsPoseMethod;
    int voAbsRansacIter;
    double voAbsRansacThresh;
    bool voAbsNLO;
    float voInitDisparity;   // disparity required to trigger initialisation
    float voKfDisparity;     // disparity required to trigger new KF
    float frameQualityThreshold;
    float keyFrameQualityThreshold;
    BaselineMethod voBaselineMethod;
    double voBaseline;

    // just for drawing
    double baselineInitial;
    double baselineCorrected;
    double timeVO;




    /// ////////////////////////////////////////////////////////////////////////////////////// PRIVATE METHODS

    /// Triangulate points points3d using bearing vectors bv1 and bv2 and the estimated pose between them trans1to2.
    void triangulate(const Pose& trans1to2, const Bearings& bv1, const Bearings& bv2, Points3d& points3d) const{
        /// The 3D points are expressed in the frame of the first viewpoint.
        ros::WallTime t0 = ros::WallTime::now();
        ROS_INFO("OVO > Triangulating");


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
        ROS_INFO("OVO < Triangulated in [%f.1ms]",1000*(ros::WallTime::now()-t0).toSec() );
    }



    /// Attempts to initialise VO between two frames
    /// data associsation through matches
    /// Sets the pose of frame f
    bool initialise(FramePtr& f, FramePtr& kf, const DMatches& matches){
        ROS_INFO("ODO > Doing VO Initialisation");


        Points3d points3d;
        DMatches matchesVO;

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
        if (voRelPoseMethod==5){
            /// Use IMU for rotation, compute translation
            ROS_INFO("Using IMU for relative rotation");
            // compute relative rotation
            const Eigen::Matrix3d& imu2cam = f->getImu2Cam(); /// TODO: getImuRotation cam is actually imu2cam*imuRot
            adapter.setR12(imu2cam * kf->getImuRotationCam().transpose() * f->getImuRotationCam() * imu2cam.transpose());

            //adapter.setR12(kf->getImuRotation().transpose() * f->getImuRotation());
            //adapter.setR12(f->getImuRotation() * kf->getImuRotation().transpose());
            //adapter.setR12(f->getImuRotation().transpose() * kf->getImuRotation());
//            adapter.setR12(Eigen::Matrix3d::Identity());
//            ROS_INFO_STREAM("kf' * f\n" << kf->getImuRotation().transpose() * f->getImuRotation());
//            ROS_INFO_STREAM("f * kf'\n" << f->getImuRotation() * kf->getImuRotation().transpose());
//            ROS_INFO_STREAM("f' * kf\n" << f->getImuRotation().transpose() * kf->getImuRotation());

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
            ROS_INFO_STREAM("ODO = NLO Difference:\n  " << (before.inverse()*transKFtoF).matrix());
            ROS_INFO("ODO < NLO Finished in [%.1fms]", (ros::WallTime::now()-tNLO).toSec()*1000);
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

        /// TRIANGULATE points in the keyframe frame
        ROS_INFO("ODO = Generating new point cloud. Relative Pose baseline [%f]", baselineInitial);
        double meanDepth=0.0;
        switch (voBaselineMethod){
            case UNCHANGED:
                ROS_INFO("ODO = Baseline left unchanged [%f]", baselineInitial);
                triangulate(transKFtoF, bvKFinlier, bvFinlier, points3d);
                break;
            case FIXED:
                ROS_INFO("ODO = Baseline scaled [1.0]");
                transKFtoF.translation() *= 1.0/baselineInitial;
                triangulate(transKFtoF, bvKFinlier, bvFinlier, points3d);
                break;
            case MANUAL_BASELINE:
                ROS_INFO("ODO = Baseline selected scaled [%f]", voBaseline);
                transKFtoF.translation() *= voBaseline/baselineInitial;
                triangulate(transKFtoF, bvKFinlier, bvFinlier, points3d);
                break;
            case MANUAL_AVGDEPTH:
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
            case AUTO_MARKER:
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
        reprojectFilter(bvKFinlier, points3d);

        /// put points from KF->Points frame to World->Points frame
        OVO::transformPoints(kf->getPose(), points3d);

        /// Update map
        map.initialise(f, points3d, matchesVO);

        ROS_INFO("ODO < Initialised");
        return true;
    }





    /// Attempts to estiamte the pose between a frame and a keyframe that has world points within it
    /// data associsation through matches
    /// Sets the pose of frame f
    bool poseEstimate(FramePtr& f, FramePtr& kf, const DMatches& matches){
        ROS_INFO("ODO > Doing VO Pose Estimate Frame [%d|%d] vs KeyFrame [%d|%d] with [%lu] matches", f->getId(), f->getKfId(), kf->getId(), kf->getKfId(), matches.size());

        matchesVO.clear();

        if (matches.size()==0){
            ROS_ERROR("ODO < CANNOT DO Pose Estiamte without matches");
            return false;
        }

        ROS_ERROR("NOT IMPLEMENTED poseEstimate()");
        /*
        const Points3d& worldPtsKF = kf->getWorldPoints3d();
        const Eigen::MatrixXd& bvf         = f->getBearings();

        ROS_INFO("ODO = Before Alignment [%lu Matches] [%ld bearings] [%lu world points] [%lu kf vo inliers]", matches.size(), bvf.rows(), worldPtsKF.size(), kf->getVoInliers().size());

        /// Get bearing vectors from current frame aligned with 3d land marks from keyframe aligned with matches
        // Indicies
        Ints fInd, kfInd;
        OVO::match2ind(matches, fInd, kfInd);

        // Bearings
        Bearings bvFMatched;
        OVO::matReduceInd(bvf, bvFMatched, fInd);

        // 3d points
        Points3d worldPtsKFMatched;
        OVO::vecReduceInd<Points3d>(worldPtsKF, worldPtsKFMatched, kfInd);

        ROS_INFO("ODO = AFTER Alignment [%lu Matches] [%lu bearings] [%lu world points]", matches.size(), bvFMatched.size(), worldPtsKFMatched.size());



        /// DO ransac stuff
        // ransac output
        opengv::absolute_pose::CentralAbsoluteAdapter adapter(bvFMatched, worldPtsKFMatched);
        Ints inliers;




        ros::WallTime t0 = ros::WallTime::now();
        if (voAbsPoseMethod==0){
            ROS_INFO("ODO = Using Relative Rotation Prior");
            /// Compute relative rotation using current and previous imu data
            ///TODO: dont we need to apply the relative rotation to the previous pose?
            //const tf::Transform absRotPrior = map.getKF(0).getImu().inverseTimes(frame.getImu()); //map[0] holds the previous frame (which holds previous imu)
            // prev pose * imu_pose_differene

            const opengv::rotation_t& imuF = f->getImuRotationCam();
            const opengv::rotation_t& imuKF = kf->getImuRotationCam();
            const Pose&    poseKF = kf->getPose();
            // set absolute rotation
            Eigen::Matrix3d imu2cam;
            imu2cam << 0, 0, 1,
                    -1, 0 ,0,
                    0,-1, 0;
            adapter.setR(poseKF * imu2cam * imuKF.transpose() * imuF * imu2cam.transpose());
        }


        /// DO RANSAC

        boost::shared_ptr<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>absposeproblem_ptr(
                    new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(adapter,
                        static_cast<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::Algorithm>(voAbsPoseMethod)) );
        opengv::sac::Ransac<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
        ransac.sac_model_ = absposeproblem_ptr;
        ransac.threshold_ = voAbsRansacThresh;
        ransac.max_iterations_ = voAbsRansacIter;

        ROS_INFO("ODO > Computing RANSAC pose estimate over [%lu matches] with [threshold = %f] ", matches.size(), ransac.threshold_);
        ransac.computeModel(1);


        // Set as output
        Pose transWtoF;
        transWtoF = ransac.model_coefficients_;
        std::swap(inliers, ransac.inliers_);


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
            //Compute the pose of a viewpoint using nonlinear optimization. Using all available correspondences. Works for central and non-central case.
            //in:  adapter holding bearing vector to world point correspondences, the multi-camera configuration, plus the initial values.
            //out: Pose of viewpoint (position seen from world frame and orientation from viewpoint to world frame, transforms points from viewpoint to world frame).
            ROS_INFO_STREAM("ODO = NLO Before:\n  " << transWtoF.matrix());
            transWtoF = opengv::absolute_pose::optimize_nonlinear(adapter, inliers) ;
            ROS_INFO_STREAM("ODO < NLO Before:\n  " << transWtoF.matrix());
        }

        f->setPose(transWtoF);

        /// JUST FOR VIS!
        /// Align inliers

        Bearings bvFVO;
        Points3d worldPtsKFVO;

        OVO::vecReduceInd<Bearings>(bvFMatched, bvFVO, inliers);
        OVO::vecReduceInd<DMatches>(matches, matchesVO, inliers);
        OVO::vecReduceInd<Points3d>(worldPtsKFMatched, worldPtsKFVO, inliers);

        const Eigen::VectorXd repjerr = OVO::reprojectErrPointsVsBV(f->getPose(), worldPtsKFVO, bvFVO );
        ROS_INFO("ODO = Min / Avg / Max RePrjErr  F = [%f, %f, %f]", repjerr.minCoeff(), repjerr.mean(), repjerr.maxCoeff());

        Ints FVOInd, KFVOInd;
        OVO::match2ind(matchesVO, FVOInd, KFVOInd);
        f->setWorldPoints(FVOInd, worldPtsKFVO, false); // No need for VO only mode

    */
        ROS_INFO("ODO < Pose Estimated");
        return true;
    }



    /// Adds the first KF
    bool setInitialKFVO(FramePtr& frame){
        ROS_INFO("ODO > SETTING INITIAL KEYFRAME");
        reset();

        const float quality = frame->getQuality();
        if (quality < keyFrameQualityThreshold && quality>=0){
            state=WAIT_FIRST_FRAME;
            ROS_WARN("ODO < FAILED TO ADD INITIAL KEYFRAME, quality too bad [%f]", quality);
            return false;
        }
        // add first keyframe
        map.pushKF(frame, true);
        state = WAIT_INIT;
        ROS_INFO("ODO < INITIAL KEYFRAME ADDED ");
        return true;



    }



    /// resets all internal structures
    void reset(){
        ROS_INFO("ODO > RESETTING");
        state=WAIT_FIRST_FRAME;
        baselineInitial = -1;
        baselineCorrected = -1;
        timeVO = -1;
        map.reset();
        matchesVO.clear();
        ROS_INFO("ODO < RESET");
    }



    /// Initialises the whole VO pipeline
    bool initialiseVO(){
        ROS_INFO("ODO > DOING INITIALISATION");

        bool okay = initialise(map.getCurrentFrame());
        //addKFVO();

        if (okay){
            ROS_INFO("ODO < INITIALISATION SUCCESS");
            state = INITIALISED;
        } else {
            ROS_WARN("ODO < INITIALISATION FAIL");
        }
        return okay;
    }



    /// adds a keyframe to the map
    bool addKFVO(){
        /// Start timer, show messages, check quality
        ros::WallTime t0 = ros::WallTime::now();

        ROS_ERROR("NOT IMPLEMENTED addKFVO()");


        FramePtr& f = map.getCurrentFrame();
        FramePtr& kf = map.getLatestKF();
        /*
        ROS_WARN("ODO > ATTEMPTING TO MAKE FRAME [%d] KEYFRAME", f->getId());

        const float quality = f->getQuality();
        if (quality < keyFrameQualityThreshold && quality>=0){
            ROS_WARN("ODO < FAILED TO ADD KEYFRAME, quality too bad [%f]", quality);
            return false;
        }

//        // only keep points that worked with vo
//        f->reducePointsToWorldPoints();

        // Triangulate all matches, hopefully adding more points


        /// Force redection between both frames from all detections (ie not filtered by vo)
        const DMatches& ms = map.getF2KFMatches(true);
        ROS_INFO_STREAM("ODO = FRAME F \n" << *f);
        ROS_INFO_STREAM("ODO = FRAME KF\n" << *kf);

        // Lets do everything in KF frame
        // get kf->f transform
        Pose kf2f = kf->getPose().inverse() * f->getPose();

        ROS_WARN("ODO = Triangulating all [%lu] matches", ms.size());
        // triangulate points
        Bearings bv_f, bv_kf;
        OVO::alignedBV(f->getBearings(), kf->getBearings(), ms, bv_f, bv_kf);
        Points3d points_tri;
        triangulate(kf2f, bv_kf, bv_f, points_tri);

        // keep those within reprojection error
        Ints inliers;
        inliers = reprojectFilter(bv_kf, points_tri); // points are in kf frame here
        OVO::vecReduceInd<Points3d>(points_tri, inliers);
        OVO::vecReduceInd<DMatches>(ms, matchesVO, inliers);

        // Put points in world frame (was in kf frame)
        OVO::transformPoints(kf->getPose(), points_tri);

        // remove outliers from f
        Ints fIn, kfIn;
        OVO::match2ind(matchesVO, fIn, kfIn);
        f->setWorldPoints(fIn, points_tri, true);

        // add f to map
        map.pushKF(f);

*/
        double time = (ros::WallTime::now()-t0).toSec();
        timeVO += time;
        bool okay = true;
        if (okay){
            ROS_INFO("ODO < KEYFRAME ADDED [ID: %d, KF: %d] in [%1.fms]", f->getId(), f->getKfId(), time*1000);
        } else {
            ROS_WARN("ODO < FAILED TO ADD KEYFRAME in [%1.fms]", time*1000.);
        }
        return okay;

    }

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


    /// estiamte pose vs using worldPoints from latest map KF
    void estimatePoseVO(bool newkf=false){
        ros::WallTime t0 = ros::WallTime::now();
        ROS_INFO("ODO > DOING POSE ESTIMATION");        

        bool okay = false;

        // Estiamte pose between current frame and last KF
        okay = poseEstimate(map.getCurrentFrame(), map.getLatestKF(), map.getF2KFMatches(newkf));


        timeVO = (ros::WallTime::now()-t0).toSec();
        if (okay){
            ROS_INFO("ODO < POSE ESTIMATION SUCCESS [%.1fms]", timeVO*1000.);
            state = INITIALISED;
        } else {
            ROS_WARN("ODO < POSE ESTIMATION FAIL [%.1fms]", timeVO*1000.);
            state = LOST;
        }

    }






public:
    /// ////////////////////////////////////////////////////////////////////////////////////// PUBLIC METHODS
    Odometry(){
        init_g2o_types();
        voRelRansacIter = 300;
        voRelRansacThresh = 1;
        voRelPoseMethod = 0;
        voTriangulationMethod = 1;
        voInitDisparity = 100;
        nextAddKf = false;

        control = DO_NOTHING;
        state   = WAIT_FIRST_FRAME;

        frameQualityThreshold = 0.2;
        keyFrameQualityThreshold = 0.6;
        baselineInitial = -1;
        baselineCorrected = -1;
        timeVO = -1;

    }



    /// a step in the VO pipeline
    void update(FramePtr& frame){
        ROS_INFO("ODO > PROCESSING FRAME [%d]", frame->getId());

        /// Skip frame immediatly if quality is too bad (should ust be used for extremely bad frames)
        const float quality = frame->getQuality();
        if (quality < frameQualityThreshold && quality>=0){
            ROS_WARN("ODO < SKIPPING FRAME [%d], Quality too bad [%f]", frame->getId(), quality );
            cv::imshow("FAILED_QUALITY", frame->getVisualImage());
            cv::waitKey(20);
            return;
        }

        float disparity = 0.f;


        if (control==DO_RESET){
            control = DO_NOTHING;
            reset();
        }


        if (state==WAIT_FIRST_FRAME) {
            // first time we received a keyframe ever
           setInitialKFVO(frame);
           track(frame);

        } else if (state==WAIT_INIT) {
            // waiting to initialise

            /// Reset initial KF
            if (control==DO_ADDKF){
                control = DO_NOTHING;
                setInitialKFVO(frame);
                track(frame); // I think?
            } else {
                /// Wait for keyframe as usual
                disparity = track(frame);

                /// Check if we can initialise
                if (control==DO_INIT || disparity>voInitDisparity){                    
                    control = DO_NOTHING;
                    initialiseVO();

                }
            }

        } else if (state==INITIALISED || state==LOST) {
            // initialised, estimate pose vs keyframe


            if (nextAddKf){
                nextAddKf = false;
                disparity = track(frame, true);
                estimatePoseVO(true);
                addKFVO();

            } else {
                disparity = track(frame);
                estimatePoseVO();
            }


            if (control==DO_ADDKF || disparity>voKfDisparity){
                control = DO_NOTHING;
                nextAddKf = true;
                ROS_WARN("ODO = SETTING NEXT FRAME AS KEYFRAME");
            }



        }

        if (state==LOST) {
            ROS_WARN("ODO = LOST");
        }

        ROS_INFO("ODO < PROCESSED FRAME [%d]", frame->getId());

    }


    /// Gets a visual image of the current state
    cv::Mat getVisualImage(){
        /// Get matching image
        cv::Mat image = map.getVisualImage();

        // Overlay baseline
        if (baselineInitial>=0){
            OVO::putInt(image, baselineInitial, cv::Point(10,9*25), CV_RGB(0,96*2,0), false , "BLD:");
        }
        if (baselineCorrected>=0){
            OVO::putInt(image, baselineCorrected, cv::Point(10,10*25), CV_RGB(0,96*2,0), false , "BLC:");
        }

        // Draw VO flow
        // only for drawing!
        if (matchesVO.size()>0){
            Points2f drawVoKFPts, drawVoFPts;
            OVO::vecAlignMatch<Points2f>(map.getCurrentFrame()->getPoints(true), map.getLatestKF()->getPoints(true), drawVoFPts, drawVoKFPts, matchesVO);
            for (uint i=0; i<drawVoFPts.size(); ++i){
                cv::line(image, drawVoFPts[i], drawVoKFPts[i], CV_RGB(255,0,255), 1, CV_AA);
            }
            OVO::putInt(image, matchesVO.size(), cv::Point(10,3*25), CV_RGB(200,0,200),  true,"VO:");
        }







        // show timings
        if (timeVO>0){
            OVO::putInt(image, timeVO*1000., cv::Point(10,image.rows-2*25), CV_RGB(200,0,200), false, "O:");
        }


        // show state
        // show user controls
        // show imu
        // show flow
        // show tracker matches
        // show vo matches
        // show map statistics
        return image;

    }

    const FramePtrs& getKeyFrames() const{
        return map.getKFs();
    }
    FramePtr& getLastFrame(){
        return map.getCurrentFrame();
    }

    void setParameter(ollieRosTools::VoNode_paramsConfig &config, uint32_t level){
        ROS_INFO("ODO > SETTING PARAMS");

        // Settings
        voRelRansacIter   = config.vo_relRansacIter;
        voRelRansacThresh = 1.0 - cos(atan(config.vo_relRansacThresh*sqrt(2.0)*0.5/720.0));
        voRelPoseMethod   = config.vo_relPoseMethod;
        voTriangulationMethod = config.vo_triMethod;
        voRelNLO          = config.vo_relNLO;
        voInitDisparity   = config.vo_initDisparity;
        voKfDisparity     = config.vo_kfDisparity;
        voBaselineMethod  = static_cast<BaselineMethod>(config.vo_relBaselineMethod);
        voBaseline        = config.vo_relBaseline;
        voAbsPoseMethod   = config.vo_absPoseMethod;
        voAbsRansacIter   = config.vo_absRansacIter;
        voAbsRansacThresh = 1.0 - cos(atan(config.vo_absRansacThresh*sqrt(2.0)*0.5/720.0));
        voAbsNLO          = config.vo_absNLO;


        // User controls
        if (config.vo_doInitialisation){
            control = DO_INIT;
            config.vo_doInitialisation = false;
        } else if (config.vo_setKeyFrame){
            control = DO_ADDKF;
            config.vo_setKeyFrame = false;
        } else if (config.vo_doReset){
            control = DO_RESET;
            config.vo_doReset = false;
        }

        if (config.writeCSV){
            ROS_ERROR("NOT IMPLEMENTED WRITE CSV FUNCTIONALITY - NON CRITICAL");
            config.writeCSV = false;
        }



        map.setParameter(config, level);

        ROS_INFO("ODO < PARAMS SET");
    }
};

#endif // ODOMETRY_HPP
