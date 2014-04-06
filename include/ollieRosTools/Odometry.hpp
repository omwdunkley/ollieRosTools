#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP
#include <Eigen/Core>

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

    // ABSOLUTE POSE - used when computing pose vs keyframe


    // just for drawing
    double baselineInitial;
    double baselineCorrected;
    double timeVO;




    /// ////////////////////////////////////////////////////////////////////////////////////// PRIVATE METHODS

    /// Triangulate points points3d using bearing vectors bv1 and bv2 and the estimated pose between them trans1to2.
    void triangulate(const opengv::transformation_t& trans1to2, const opengv::bearingVectors_t& bv1, const opengv::bearingVectors_t& bv2, opengv::points_t& points3d) const{
        /// The 3D points are expressed in the frame of the first viewpoint.

        ROS_INFO("OVO > Triangulating");


        // Triangulate point pairs of each match
        opengv::relative_pose::CentralRelativeAdapter adapter(bv1, bv2, trans1to2.col(3), trans1to2.block<3,3>(0,0) );

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
        ROS_INFO("OVO < Triangulated");
    }



    /// Attempts to initialise VO between two frames
    /// data associsation through matches
    /// Sets the pose of frame f
    bool initialise(FramePtr& f, FramePtr& kf, const DMatches& matches){
        ROS_INFO("ODO > Doing VO Initialisation");


        opengv::points_t worldPoints;
        DMatches matchesVO;

        /// Get unit features aligned
        opengv::bearingVectors_t bvF;
        opengv::bearingVectors_t bvKF;
        OVO::alignedBV(f->getBearings(), kf->getBearings(), matches, bvF, bvKF);
        opengv::relative_pose::CentralRelativeAdapter adapter(bvKF, bvF);
        Ints inliers;
        int iterations=-1;

        /// Compute relative transformation from KeyFrame KF to Frame F using ransac
        opengv::transformation_t transKFtoF;

        ros::WallTime t0 = ros::WallTime::now();
        if (voRelPoseMethod==5){
            /// Use IMU for rotation, compute translation
            // compute relative rotation
            Eigen::Matrix3d imu2cam;
            imu2cam << 0, 0, 1,
                      -1, 0 ,0,
                       0,-1, 0;
            adapter.setR12(imu2cam * kf->getImuRotation().transpose() * f->getImuRotation() * imu2cam.transpose());

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
            ROS_INFO("ODO = Relative Ransac Done with %lu/%lu inliers [%d iterations]in [%.1fms", inliers.size(), matches.size(), iterations,(ros::WallTime::now()-t0).toSec()*1000.);
        }

        /// Experimental: NLO
        /// TODO: estimate yaw bias
        if (voRelNLO){
            // check order
            ROS_INFO("ODO > Doing NLO on Relative Pose");
            adapter.sett12(transKFtoF.col(3));
            adapter.setR12(transKFtoF.block<3,3>(0,0));
            ROS_INFO_STREAM("ODO = NLO Before:\n  " << transKFtoF);
            transKFtoF = opengv::relative_pose::optimize_nonlinear(adapter, inliers);
            ROS_INFO_STREAM("ODO < NLO AFTER:\n  " << transKFtoF);
        }

        /// g2o BUNDLE ADJUST
        /// TODO


        //printRPYXYZ(poseTF, "Ransac Output: ");

        /// Align inliers
        opengv::bearingVectors_t bvFinlier;
        opengv::bearingVectors_t bvKFinlier;
        OVO::vecReduceInd<opengv::bearingVectors_t>(bvF, bvFinlier, inliers);
        OVO::vecReduceInd<opengv::bearingVectors_t>(bvKF, bvKFinlier, inliers);
        OVO::vecReduceInd<DMatches>(matches, matchesVO, inliers);

        /// Scale Pose
        baselineInitial = transKFtoF.col(3).norm();

        /// TRIANGULATE points in the keyframe frame
        ROS_INFO("ODO = Relative Pose baseline [%f]", baselineInitial);
        double meanDepth=0.0;
        switch (voBaselineMethod){
            case UNCHANGED:
                ROS_INFO("ODO = Baseline left unchanged [%f]", baselineInitial);
                triangulate(transKFtoF, bvKFinlier, bvFinlier, worldPoints);
                break;
            case FIXED:
                ROS_INFO("ODO = Baseline scaled [1.0]");
                transKFtoF.col(3) *= 1.0/baselineInitial;
                triangulate(transKFtoF, bvKFinlier, bvFinlier, worldPoints);
                break;
            case MANUAL_BASELINE:
                ROS_INFO("ODO = Baseline selected scaled [%f]", voBaseline);
                transKFtoF.col(3) *= voBaseline/baselineInitial;
                triangulate(transKFtoF, bvKFinlier, bvFinlier, worldPoints);
                break;
            case MANUAL_AVGDEPTH:
                ROS_INFO("ODO > Scaling Baseline so avg depth is [%f]", voBaseline);
                transKFtoF.col(3) /= baselineInitial;

                // Triangulate with baseline = 1
                triangulate(transKFtoF, bvKFinlier, bvFinlier, worldPoints);

                // calculate mean depth
                for (uint i=0; i<worldPoints.size(); ++i){
                   meanDepth += worldPoints[i].norm();
                }
                meanDepth /=static_cast<double>(worldPoints.size());
                ROS_INFO("ODO = Baseline is [%f] with mean depth [%f]", baselineInitial, meanDepth);

                // modify points
                for (uint i=0; i<worldPoints.size(); ++i){
                    worldPoints[i] /= meanDepth/voBaseline;
                }
                // modify baseline
                transKFtoF.col(3) /= meanDepth/voBaseline;
                ROS_INFO("ODO < Baseline updated to [%f] with mean depth [%f]", transKFtoF.col(3).norm(), voBaseline);
                break;
            case AUTO_MARKER:
                ROS_ERROR("ODO = NOT IMPLEMENTED: Baseline being scaled using Markers");
                transKFtoF.col(3) *= 1.0/baselineInitial;
                triangulate(transKFtoF, bvKFinlier, bvFinlier, worldPoints);
                break;

        }

        // just for drawing really
        baselineCorrected = transKFtoF.col(3).norm();

        meanDepth = 0;
        for (uint i=0; i<worldPoints.size(); ++i){
           meanDepth += worldPoints[i].norm();
        }
        meanDepth /=static_cast<double>(worldPoints.size());
        ROS_INFO("ODO = Mean depth [%f]", meanDepth);




        /// Set pose of F from KF->F to world->F
        //f2->setPose(); //
        f->setPose(kf->getPose() * transKFtoF);

        /// put points from KF->Points frame to World->Points frame
        OVO::transformPoints(kf->getPose(), worldPoints);

        /// just to print!
        Eigen::VectorXd repjerr;
        repjerr = OVO::reprojectErrPointsVsBV(kf->getPose(), worldPoints, bvKFinlier );
        ROS_INFO("ODO = Min / Avg / Max RePrjErr KF = [%f, %f, %f]", repjerr.minCoeff(), repjerr.mean(), repjerr.maxCoeff());
        repjerr = OVO::reprojectErrPointsVsBV(f->getPose(), worldPoints, bvFinlier );
        ROS_INFO("ODO = Min / Avg / Max RePrjErr  F = [%f, %f, %f]", repjerr.minCoeff(), repjerr.mean(), repjerr.maxCoeff());

        /// add points to track against to keyframes
//        Points2f fWPts3d, kfWPts3d;
//        OVO::vecAlignMatch(f->getPoints(true), kf->getPoints(true), fWPts3d, kfWPts3d ,matchesVO);
//        // kf should already be added
//        f->setWorldPoints(fWPts3d, worldPoints);
//        kf->setWorldPoints(kfWPts3d, worldPoints);

        // remove points / keypoints / descriptors / etc not used
        Ints indF, indKF;
        OVO::match2ind(matchesVO, indF, indKF);
        f->setWorldPoints(indF, worldPoints);
        kf->setWorldPoints(indKF, worldPoints);

        map.addKeyFrame(f);

        ROS_INFO("ODO < Initialised");

        return true;
    }




    /// Attempts to estiamte the pose between a frame and a keyframe that has world points within it
    /// data associsation through matches
    /// Sets the pose of frame f
    bool poseEstimate(FramePtr& f, FramePtr& kf, const DMatches& matches){
        ROS_INFO("ODO > Doing VO Pose Estimate");


        const opengv::points_t& worldPtsKF = kf->getWorldPoints3d();


        /// Get bearing vectors from current frame aligned with 3d land marks from keyframe aligned with matches

        Ints bvFindMatched, wpKFindMatched;
        OVO::match2ind(matches, bvFindMatched, wpKFindMatched);

        // Bearing vectors
        opengv::bearingVectors_t bvFMatched;
        OVO::matReduceInd(f->getBearings(), bvFMatched, bvFindMatched);

        // 3d points
        opengv::points_t worldPtsKFMatched;
        OVO::vecReduceInd<opengv::points_t>(worldPtsKF, worldPtsKFMatched, wpKFindMatched);




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

            const opengv::rotation_t& imuF = f->getImuRotation();
            const opengv::rotation_t& imuKF = kf->getImuRotation();
            const Eigen::Affine3d&    poseKF = kf->getPose();
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
        opengv::transformation_t transWtoF = ransac.model_coefficients_;
        std::swap(inliers, ransac.inliers_);


        if (inliers.size()<10){
            ROS_WARN("ODO < RANSAC failed on Absolute Pose Estimation with %lu/%lu inliers [%d iterations] [%.1fms]", inliers.size(), matches.size(),  ransac.iterations_, (ros::WallTime::now()-t0).toSec()*1000.);
            return false;
        } else {
            ROS_INFO("ODO < Absolute Ransac Done with %lu/%lu inliers [%d iterations] [%.1fms]", inliers.size(), matches.size(), ransac.iterations_,  (ros::WallTime::now()-t0).toSec()*1000.);


        }


        if (voAbsNLO){
            ROS_INFO("ODO > Doing NLO on Absolute Pose");
            adapter.sett(transWtoF.col(3));
            adapter.setR(transWtoF.block<3,3>(0,0));
            //Compute the pose of a viewpoint using nonlinear optimization. Using all available correspondences. Works for central and non-central case.
            //in:  adapter holding bearing vector to world point correspondences, the multi-camera configuration, plus the initial values.
            //out: Pose of viewpoint (position seen from world frame and orientation from viewpoint to world frame, transforms points from viewpoint to world frame).
            ROS_INFO_STREAM("ODO = NLO Before:\n  " << transWtoF);
            transWtoF = opengv::absolute_pose::optimize_nonlinear(adapter, inliers) ;
            ROS_INFO_STREAM("ODO < NLO Before:\n  " << transWtoF);
        }

        f->setPose(transWtoF);

        /// JUST FOR VIS!
        /// Align inliers

        opengv::bearingVectors_t bvFVO;
        opengv::points_t worldPtsKFVO;

        OVO::vecReduceInd<opengv::bearingVectors_t>(bvFMatched, bvFVO, inliers);
        OVO::vecReduceInd<DMatches>(matches, matchesVO, inliers);
        OVO::vecReduceInd<opengv::points_t>(worldPtsKFMatched, worldPtsKFVO, inliers);

        const Eigen::VectorXd repjerr = OVO::reprojectErrPointsVsBV(f->getPose(), worldPtsKFVO, bvFVO );
        ROS_INFO("ODO = Min / Avg / Max RePrjErr  F = [%f, %f, %f]", repjerr.minCoeff(), repjerr.mean(), repjerr.maxCoeff());

        Ints FVOInd, KFVOInd;
        OVO::match2ind(matchesVO, FVOInd, KFVOInd);
        f->setWorldPointsInliers(FVOInd, worldPtsKFVO); // not really needed i guess, but good for vis



        ROS_INFO("ODO < Pose Estimated");
        return true;
    }




    /// Tracks frame against keyframe
    float track(FramePtr& frame){
        ROS_INFO("ODO > TRACKING");
        matchesVO.clear();
        const float disparity = map.showFrame(frame);


        ROS_INFO("ODO < TRACKED");

        bool okay = true;
        if (okay){
            ROS_INFO("ODO < TRACKED ");
        } else {
            ROS_INFO("ODO < TRACKING FAILED");
        }


        return disparity;
    }



    /// Adds the first KF
    void setInitialKFVO(FramePtr& frame){
        ROS_INFO("ODO > SETTING INITIAL KEYFRAME");
        reset();

        const float quality = frame->getQuality();
        if (quality < keyFrameQualityThreshold && quality>=0){
            state=WAIT_FIRST_FRAME;
            ROS_WARN("ODO < FAILED TO ADD INITIAL KEYFRAME, quality too bad [%f]", quality);
        }
        // add first keyframe
        map.addKeyFrame(frame, true);
        state = WAIT_INIT;
        ROS_INFO("ODO < INITIAL KEYFRAME ADDED ");



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
    void initialiseVO(){
        ROS_INFO("ODO > DOING INITIALISATION");


        bool okay = initialise(map.getCurrentFrame(), map.getLatestKF(), map.getF2KFMatches());
        //addKFVO();

        if (okay){
            ROS_INFO("ODO < INITIALISATION SUCCESS");


            //map.initialise(worldPoints, voMatches);

            state = INITIALISED;
        } else {
            ROS_WARN("ODO < INITIALISATION FAIL");
        }

    }



    /// adds a keyframe to the map
    void addKFVO(){
        ROS_INFO("ODO > ADDING KEYFRAME");

        bool okay = true;
        if (okay){
            ROS_INFO("ODO < KEYFRAME ADDED");
        } else {
            ROS_WARN("ODO < FAILED TO ADD KEYFRAME");
        }

    }


    /// estiamte pose vs using worldPoints from latest map KF
    void estimatePoseVO(){
        ros::WallTime t0 = ros::WallTime::now();
        ROS_INFO("ODO > DOING POSE ESTIMATION");        

        bool okay = poseEstimate(map.getCurrentFrame(), map.getLatestKF(), map.getF2KFMatches());

        timeVO = (ros::WallTime::now()-t0).toSec();
        if (okay){
            ROS_INFO("ODO < POSE ESTIMATION SUCCESS [%fms]", timeVO*1000.);
            state = INITIALISED;
        } else {
            ROS_WARN("ODO < POSE ESTIMATION FAIL [%fms]", timeVO*1000.);
            state = LOST;
        }

    }






public:
    /// ////////////////////////////////////////////////////////////////////////////////////// PUBLIC METHODS
    Odometry(){
        voRelRansacIter = 300;
        voRelRansacThresh = 1;
        voRelPoseMethod = 0;
        voTriangulationMethod = 1;
        voInitDisparity = 100;

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

        /// Skip frame immediatly of quality is too bad (should ust be used for extremely bad frames)
        const float quality = frame->getQuality();
        if (quality < frameQualityThreshold && quality>=0){
            ROS_INFO("ODO < SKIPPING FRAME [%d], Quality too bad [%f]", frame->getId(), quality );
        }

        float disparity = 0.f;


        if (control==DO_RESET){
            control = DO_NOTHING;
            reset();
        }


        if (state==WAIT_FIRST_FRAME) {
            // first time we receiver a keyframe ever
           setInitialKFVO(frame);
           track(frame);



        } else if (state==WAIT_INIT) {
            // waiting to initialise

            /// Reset initial KF
            if (control==DO_ADDKF){
                control = DO_NOTHING;
                setInitialKFVO(frame);

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

            disparity = track(frame);

            estimatePoseVO();

            if (control==DO_ADDKF || disparity>voKfDisparity){
                control = DO_NOTHING;
                addKFVO();
            }



        }

        if (state==LOST) {
            ROS_WARN_THROTTLE(0.1, "ODO = LOST");
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



        map.setParameter(config, level);

        ROS_INFO("ODO < PARAMS SET");
    }
};

#endif // ODOMETRY_HPP
