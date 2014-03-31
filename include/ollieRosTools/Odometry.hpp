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
    float frameQualityThreshold;
    float keyFrameQualityThreshold;

    // ABSOLUTE POSE - used when computing pose vs keyframe



    /// ////////////////////////////////////////////////////////////////////////////////////// PRIVATE METHODS

    /// Triangulate points points3d using bearing vectors bv1 and bv2 and the estimated pose between them trans1to2.
    void triangulate(const opengv::transformation_t& trans1to2, const opengv::bearingVectors_t& bv1, const opengv::bearingVectors_t& bv2, opengv::points_t& points3d) const{
        /// The 3D points are expressed in the frame of the first viewpoint.

        ROS_INFO("      Triangulating with baseline %f", trans1to2.col(3).norm());


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
    }



    /// Attempts to initialise VO between two frames
    /// data associsation through matches
    /// Sets the pose of frame f
    /// Outputs triangulated world points and the corresponding inlier matches
    bool initialise(FramePtr& f, FramePtr& kf, const DMatches& matches, opengv::points_t& worldPoints, DMatches& matchesVO, double scale=1.0) const{
        ROS_INFO("   Initialising");

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
            ROS_WARN("RANSAC failed init with %lu/%lu inliers", inliers.size(), matches.size());
            return false;
        }

        /// Experimental: NLO
        /// TODO: estimate yaw bias
        if (voRelNLO){
            // check order
            ROS_INFO("Doing NLO");

            adapter.sett12(transKFtoF.col(3));
            adapter.setR12(transKFtoF.block<3,3>(0,0));
            ROS_INFO_STREAM("   Before:\n  " << transKFtoF);
            transKFtoF = opengv::relative_pose::optimize_nonlinear(adapter, inliers);
            ROS_INFO_STREAM("   After :\n  " << transKFtoF);
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
        double length = transKFtoF.col(3).norm();

        ROS_INFO("   Relative Pose baseline %f", length);
        transKFtoF.col(3) *= scale/length;

        /// set poses
        //f2->setPose(); //
        f->setPose(kf->getPose() * transKFtoF);

        /// triangulate
        // get points in f2 frame
        triangulate(transKFtoF, bvKFinlier, bvFinlier, worldPoints);
        // put them in world frame
        OVO::transformPoints(kf->getPose(), worldPoints);
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
                    setInitialKFVO(frame); // JUST FOR TESTING
                    //initialiseVO();

                }
            }




        } else if (state==INITIALISED || state==LOST) {
            // initialised, estimate pose vs keyframe

            disparity = track(frame);

            estimatePoseVO();

            if (control==DO_ADDKF || disparity>voInitDisparity){
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
