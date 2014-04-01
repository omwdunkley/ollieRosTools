#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP
#include <Eigen/Core>

#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/TranslationOnlySacProblem.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/triangulation/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>

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

    /// SETTINGS
    // RELATIVE POSE - used by initialisation
    int voTriangulationMethod;
    bool voRelNLO;
    double voRelRansacThresh;
    int voRelRansacIter;
    int voRelPoseMethod;    
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
    /// Outputs triangulated world points and the corresponding inlier matches
    bool initialise(FramePtr& f, FramePtr& kf, const DMatches& matches, opengv::points_t& worldPoints, DMatches& matchesVO){
        ROS_INFO(" ODO > Initialising");

        /// Get unit features aligned
        opengv::bearingVectors_t bvF;
        opengv::bearingVectors_t bvKF;
        OVO::alignedBV(f->getBearings(), kf->getBearings(), matches, bvF, bvKF);
        opengv::relative_pose::CentralRelativeAdapter adapter(bvKF, bvF);
        Ints inliers;

        /// Compute relative transformation from KeyFrame KF to Frame F using ransac
        opengv::transformation_t transKFtoF;

        if (voRelPoseMethod==5){
            /// Use IMU for rotation, compute translation
            // compute relative rotation

            adapter.setR12(kf->getImuRotation().transpose() * f->getImuRotation());

            ///DO RANSAC for translation only
            boost::shared_ptr<RP::TranslationOnlySacProblem>relposeproblem_ptr(new RP::TranslationOnlySacProblem(adapter) );
            opengv::sac::Ransac<RP::TranslationOnlySacProblem> ransac;
            ransac.sac_model_ = relposeproblem_ptr;
            ransac.threshold_ = voRelRansacThresh;
            ransac.max_iterations_ = voRelRansacIter;
            ransac.computeModel(1);

            // Set as output
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
            transKFtoF = ransac.model_coefficients_;
            std::swap(inliers, ransac.inliers_);  //Get inliers
        }

        if (inliers.size()<10){
            ROS_WARN("ODO = RANSAC failed on Relative Pose Estimation with %lu/%lu inliers", inliers.size(), matches.size());
            return false;
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

        ROS_INFO("   Relative Ransac Done with %lu/%lu inliers", inliers.size(), matches.size());
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
        ROS_INFO(" ODO < Initialised");
        return true;
    }



    /// Tracks frame against keyframe
    float track(FramePtr& frame){
        ROS_INFO("ODO > TRACKING");
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
        map.reset();
        ROS_INFO("ODO < RESET");
    }



    /// Initialises the whole VO pipeline
    void initialiseVO(){
        ROS_INFO("ODO > DOING INITIALISATION");

        opengv::points_t worldPoints;
        DMatches voMatches;



        bool okay = initialise(map.getCurrentFrame(), map.getLatestKF(), map.getF2KFMatches(), worldPoints, voMatches );
        //addKFVO();

        if (okay){
            ROS_INFO("ODO < INITIALISATION SUCCESS");


            map.initialise(worldPoints, voMatches);

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


    /// estiamte pose vs MAP
    void estimatePoseVO(){
        ROS_INFO("ODO > DOING POSE ESTIMATION");



        bool okay = true;
        if (okay){
            ROS_INFO("ODO < POSE ESTIMATION SUCCESS");
            state = INITIALISED;
        } else {
            ROS_WARN("ODO < POSE ESTIMATION FAIL");
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
        /// TODO
        cv::Mat image = map.getVisualImage();

        if (baselineInitial>=0){
        OVO::putInt(image, baselineInitial, cv::Point(10,9*25), CV_RGB(0,96*2,0), false , "BLD:");
        }
        if (baselineCorrected>=0){
        OVO::putInt(image, baselineCorrected, cv::Point(10,10*25), CV_RGB(0,96*2,0), false , "BLC:");
        }


        // show timings
        // show state
        // show user controls
        // show imu
        // show flow
        // show tracker matches
        // show vo matches
        // show map statistics
        return image;

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
