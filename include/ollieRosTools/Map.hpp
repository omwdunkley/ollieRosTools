#ifndef MAP_HPP
#define MAP_HPP



#include <deque>


#include <Eigen/StdVector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/structure_only/structure_only_solver.h>

#include <ollieRosTools/custom_types/edge_pose_landmark_reprojectBV.hpp>
#include <ollieRosTools/custom_types/vertex_landmarkxyz.hpp>
#include <ollieRosTools/custom_types/vertex_pose.hpp>
#include <ollieRosTools/custom_types/register_types.hpp>


//#include <ollieRosTools/Frame.hpp>
#include <ollieRosTools/aux.hpp>
#include <ollieRosTools/Landmark.hpp>
#include <ollieRosTools/Matcher.hpp>



static const uint MAX_KF = 1000;

// worldPoint[i] corresponds to feature[idx[i]]
cv::Mat getPointsProjectedImage(FramePtr& f, const opengv::points_t& worldPts, const Ints& idx);



class OdoMap {
    private:
        /// Containers
        // list of keyframes. Latest KF at end, oldest first DEQUEs
        FramePtrs keyframes;        
        // List of map points observed by the key points
        LandMarkPtrs landmarks;
        // Frame last exposed to the map. Might be a keyframe or not
        FramePtr currentFrame;
        // Matcher used to do map-frame and frame-frame matching
        Matcher matcher;

        /// Meta
        // which frames to fix during BA
        enum FixMethod {FIX_FIRST,
                    FIX_LAST,
                    FIX_FIRST_LAST,
                    FIX_ALL,
                    FIX_NONE
                   };


        /// Settings
        uint maxKFNr;
        bool g2oDense;
        int g2oIter;
        bool g2oHuber;
        bool g2oStructure;
        FixMethod g2oFix;


    public:
        OdoMap(){
            maxKFNr = 10;
            g2oDense = false;
            g2oIter = 1000;
            g2oHuber = false;
            g2oStructure = false;
            g2oFix = FIX_FIRST;
        }

        const FramePtrs& getKFs() const {
            return keyframes;
        }
        const LandMarkPtrs& getLMs() const {
            return landmarks;
        }

        FramePtr& getLatestKF(){
            ROS_ASSERT(getKeyframeNr()>0);
            return keyframes.back();
        }

        const FramePtr& getLatestKF() const{
            ROS_ASSERT(getKeyframeNr()>0);
            return keyframes.back();
        }

        // Gets latest frame the
        FramePtr& getCurrentFrame(){
            ROS_ASSERT(!currentFrame.empty()&&currentFrame->isInitialised());
            return currentFrame;
        }

        FramePtrs getClosestKeyframes(FramePtr){
            /// Goes through all keyframes and gets the cloest N frames
            // TODO
            // image descriptor (rotation invarient)
            // sbi ncc/ssd  ( rotated)
            // estimated angle
            ///x,y,z < 2m,
            // gyro angle, yaw only +- 45 degrees
            // optical axis vs optical axis < 45 degrees
            ROS_ERROR("MAP = NOT IMPLEMENTED GET CLOSEST KEYFRAMES: %s}\n For now returning all", __SHORTFILE__);
            return keyframes;
        }


        /// Goes through all map points and remove them if they only have one reference (ie this container holding it)
        void removeNonVisiblePoints(){
            ROS_INFO("MAP > Removing non visible points");
            size_t s = landmarks.size();
            landmarks.erase( std::remove_if( landmarks.begin(), landmarks.end(), noRef), landmarks.end() );
            ROS_INFO("MAP < Removed [%lu/%lu] points left", landmarks.size(), s);
        }


        // Gets some kind of visual image showing the state of the keyframes (and current frame)?)
        cv::Mat getVisualImage(){
            ROS_WARN("MAP = NOT IMPLEMENTED GET VISUAL IMAGE");
            cv::Mat img;
            //cv::Mat img = tracker.getVisualImage();
            return img;
        }

        void reset(){
            ROS_INFO("MAP > RESETING MAP. Clearing [%lu] key frames and [%lu] land marks", keyframes.size(), landmarks.size());
            keyframes.clear();
            landmarks.clear();
            Landmark::reset();
            currentFrame = FramePtr();
            ROS_INFO("MAP < MAP RESET");
        }

        size_t getKeyframeNr() const{
            return keyframes.size();
        }

        size_t getLandmarkNr() const{
            return landmarks.size();
        }



        /// Matches against a keyframe. If voOnly = true, only match against points that have associsated land marks
        void match2KF(FramePtr& f, Points3d points, DMatches& ms, bool voOnly=false){
            /// Match against last N keyframes
            /// Match against closest N keyframes
            ROS_ASSERT(keyframes.size()>0);
            FramePtr& kf = getLatestKF();
            ROS_INFO("MAP > Matching Frame [%d|%d] against KeyFrame [%d|%d]", f->getId(), f->getKfId(), kf->getId(), kf->getKfId() );


            if (voOnly){
                //matcher.match();
            } else {

            }





        }

        /// Matches against the map
        void match2Map(FramePtr& f, Points3d points, DMatches& ms){
            ROS_ASSERT(landmarks.size()>0);

        }

        /// Show the frame to the map, track against latest KF, return disparity
//        float showFrame(FramePtr& frame, bool reset = false){
//            currentFrame = frame;
//            const float disparity = tracker.track(currentFrame, reset);
//            return disparity;
//        }




//        const DMatches& getF2KFMatches(const bool forceVoModeOff = false){
//                return tracker.getF2KFMatches(forceVoModeOff);
//        }


        void pushKF(FramePtr& frame, const bool first=false){
            ROS_INFO("MAP > ADDING KF TO MAP");
            frame->setAsKF(first);
            if (keyframes.size()==0 || first){
                reset();
                keyframes.push_back(frame);
                ROS_INFO("MAP > INITIAL KF ADDED");
            } else {
                keyframes.push_back(frame);
                ROS_INFO("MAP > KF ADDED [KFS = %lu]", getKeyframeNr());
                // Check we dont have too many keyframes
                shirnkKFs();
            }
        }

        // Initialise the map. Must have an initial keyframe already, and this should then
        // add the second one. vomatches associate them and must be aligned to points (in world frame)
        void initialise(FramePtr& f, const Points3d& points, const DMatches& voMatches){
            ROS_INFO("MAP > Initialiseing map with new Frame [%d]", f->getId());

            ROS_ASSERT(points.size() == voMatches.size());
            ROS_ASSERT(keyframes.size()==1);
            ROS_ASSERT(landmarks.size()==0);
            ROS_ASSERT(f->poseEstimated());

            // Short cut to current key frame
            FramePtr kf = getLatestKF();

            // Add frame
            pushKF(f);


            // add points
            for (uint i=0; i<points.size(); ++i){
                LandmarkPtr lm = new Landmark(points[i]);
                // add frames to points
                lm->addObservation( f, voMatches[i].queryIdx);
                lm->addObservation(kf, voMatches[i].trainIdx);
                // add points to frames
                f ->addLandMarkRef(voMatches[i].queryIdx, lm);
                kf->addLandMarkRef(voMatches[i].trainIdx, lm);
                // add point to map
                landmarks.push_back(lm);
            }
            ROS_INFO_STREAM("MAP < Map Initialised. " << *this);
        }

        // Assumes worldPoiints are triangulated from current frame and frame
        /// TODO
//        void initialise(const opengv::points_t worldPoints, const DMatches& voMatches){
//            ROS_INFO("MAP < INITIALISING MAP ");
//            points = worldPoints;
//            kfMatches = voMatches;
//            ROS_INFO("MAP < MAP INITIALISED ");
//        }

        void shirnkKFs(){
            if (keyframes.size()>maxKFNr || keyframes.size()==MAX_KF) {
                ROS_INFO("MAP > TOO MANY KEYFRAMES, REMOVING OLDEST");
                while(keyframes.size()>maxKFNr){
                    popKF();
                }
                ROS_INFO("MAP < CAPPED KFS");
            }
        }


        void popKF(){
            ROS_INFO("MAP > REMOVING OLDEST KF [%d|%d]", keyframes.front()->getId(), keyframes.front()->getKfId() );
            keyframes.pop_front();
            removeNonVisiblePoints();
            ROS_INFO("MAP < REMOVED OLDEST KF");
        }



        void bundleAdjust(){
            ROS_INFO("MAP > Doing G2O Bundle adjustment with [%lu] KeyFrames and [%lu] LandMarks", keyframes.size(), landmarks.size());
            ros::WallTime tStart = ros::WallTime::now();

            /// create solver
            g2o::SparseOptimizer optimizer;
            optimizer.setVerbose(true);
            g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
            if (g2oDense) {
                linearSolver= new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
            } else {
                linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();
            }
            g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
            g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
            optimizer.setAlgorithm(solver);

            // Some meta data to print with
            bool ok;
            int edgeCount = 0;
            int lmCount    = 0;
            int poseCount  = 0;


            /// Setup add frames
            ROS_INFO("g2o = Adding [%lu] Keyframe poses", keyframes.size());
            for (uint i=0; i<keyframes.size();++i){
                FramePtr& kf = keyframes[i];
                // create pose
                VertexPose* v_pose = new VertexPose();
                // set estimate
                v_pose->setEstimate(Eigen::Isometry3d(kf->getPose().matrix()));
                // fix
                if (g2oFix==FIX_NONE){
                    v_pose->setFixed(false);
                } else if (g2oFix==FIX_ALL){
                    v_pose->setFixed(true);
                } else if (i==keyframes.size()-1 && g2oFix==FIX_LAST){
                    ROS_INFO("g2o = Fixing [Last] Keyframe [%d|%d] pose", kf->getId(), kf->getKfId());
                    v_pose->setFixed(true);
                } else if (i==0 && g2oFix==FIX_FIRST){
                    ROS_INFO("g2o = Fixing [First] Keyframe [%d|%d] pose", kf->getId(), kf->getKfId());
                    v_pose->setFixed(true);
                } else if ((i==0 || i==keyframes.size()-1) && g2oFix==FIX_FIRST_LAST){
                    v_pose->setFixed(true);
                    ROS_INFO("g2o = Fixing [%s] Keyframe [%d|%d] pose", i==0 ? "First" : "Last", kf->getId(), kf->getKfId());
                }
                v_pose->setId(kf->getKfId());
                poseCount++;

                ok = optimizer.addVertex(v_pose);
                ROS_ASSERT_MSG(ok, "g2o = Could not add v_pose of frame [%d|%d] to pose graph", kf->getId(), kf->getKfId());
            }
            ROS_INFO("g2o = Added [%d] Keyframe poses", poseCount);

            /// TODO: do we need to add pose-pose edges??


            /// Setup add land marks
            ROS_INFO("g2o = Adding up to [%lu landmarks], adding observations", landmarks.size());
            for (uint i=0; i<landmarks.size(); ++i){
                LandmarkPtr& lm = landmarks[i];
                uint obsNr = lm->getObservationsNr();
                if (obsNr>1){
                    VertexLandmarkXYZ * v_lm = new VertexLandmarkXYZ();
                    v_lm->setId(lm->getId()+MAX_KF);
                    v_lm->setMarginalized(true);
                    v_lm->setFixed(false);
                    v_lm->setEstimate(lm->getPosition());
                    ok = optimizer.addVertex(v_lm);
                    ++lmCount;
                    ROS_ASSERT_MSG(ok, "g2o = Could not add v_landmark [%d] to pose graph", lm->getId());

                    /// Setup add observations
                    for (uint fid=0; fid<obsNr; ++fid){
                        const FramePtr& kf = lm->getObservationFrame(fid);
                        const Bearing&  bv = lm->getObservationBearing(fid);
                        EdgePoseLandmarkReprojectBV * ef = new EdgePoseLandmarkReprojectBV();
                        ef->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(kf->getId())));
                        ef->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_lm));
                        ef->setMeasurement(Eigen::Vector2d(bv[0]/bv[2], bv[1]/bv[2]));
                        ef->setInformation(Eigen::Matrix2d::Identity());
                        if (g2oHuber) {
                            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                            ef->setRobustKernel(rk);
                        }
                        ok = optimizer.addEdge(ef);
                        ++edgeCount;
                        ROS_ASSERT_MSG(ok, "g2o = Could not add edge between Landmark [%d] and frame [%d|%d] to pose graph", lm->getId(), kf->getId(), kf->getKfId());
                    }
                }

            }
            ROS_INFO("g2o = Added [%d/%lu] landmarks", lmCount, landmarks.size());
            ROS_INFO("g2o = Added [%d] pose-landmark edges", edgeCount);
            ros::WallTime tSetup = ros::WallTime::now();
            ROS_INFO("g2o = Finished setting up [%.1fms]", (tSetup-tStart).toSec()*1000.);


            /// Run optimisation
            optimizer.initializeOptimization();
            optimizer.setVerbose(true);
            if (g2oStructure){
                g2o::StructureOnlySolver<3> structure_only_ba;
                ROS_INFO("g2o = Doing structure-only BA First");
                g2o::OptimizableGraph::VertexContainer points;
                for (g2o::OptimizableGraph::VertexIDMap::const_iterator it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it) {
                    g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
                    if (v->dimension() == 3){
                        points.push_back(v);
                    }
                }
                structure_only_ba.calc(points, g2oIter);
            }
            optimizer.save("map_ba.g2o");
            ROS_INFO("g2o > Performing full BA:");
            optimizer.optimize(g2oIter);
            ros::WallTime tOpt = ros::WallTime::now();
            ROS_INFO("g2o < Done BA [Setup: %.1fms] [Opti: %.1fms]",  (tSetup-tStart).toSec()*1000., (tOpt-tSetup).toSec()*1000. );


            // Update map using BA results
            /// Update frames
            ROS_INFO("g2o = Updating [%d] Keyframe poses", poseCount);
            for (uint i=0; i<keyframes.size();++i){
                FramePtr& kf = keyframes[i];
                Pose poseEstimate = dynamic_cast<VertexPose*>(optimizer.vertex(kf->getId()))->estimate();
                kf->setPose(poseEstimate);
            }

            /// Update Land Marks
            ROS_INFO("g2o = Updating [%d landmarks]", lmCount);
            for (uint i=0; i<landmarks.size(); ++i){
                LandmarkPtr& lm = landmarks[i];
                Point3d positionEstimate = dynamic_cast<VertexLandmarkXYZ*>(optimizer.vertex(MAX_KF+lm->getId()))->estimate();
                lm->setPosition(positionEstimate);
            }

            optimizer.clear();


            ros::WallTime tEnd = ros::WallTime::now();
            ROS_INFO("MAP < Bundle Adjustment Finished [Setup: %.1fms] [Opti: %.1fms] [Update: %.1fms] [Total: %.1fms]",
                     (tSetup-tStart).toSec()*1000., (tOpt-tSetup).toSec()*1000., (tEnd-tOpt).toSec()*1000., (tEnd-tStart).toSec()*1000. );

        }


        // print id, xyz, nr of ovservations, and observations
        friend std::ostream& operator<< (std::ostream& stream, const OdoMap& map) {
            stream << "Map "
                   << "[KFs:" << std::setw(3) << std::setfill(' ') << map.keyframes.size() << "]"
                   << "[LMs:" << std::setw(5) << std::setfill(' ') << map.landmarks.size() << "]"
                   << "[Latest KeyFrame: "
                   << std::setw(5) << std::setfill(' ') << map.getLatestKF()->getId() << "|"
                   << std::setw(3) << std::setfill(' ') << map.getLatestKF()->getKfId() << "]"
                   << "[Current Frame:"
                   << std::setw(5) << std::setfill(' ') << map.currentFrame->getId() << "|"
                   << std::setw(3) << std::setfill(' ') << map.currentFrame->getKfId() << "]" ;
            return stream;
        }



        void setParameter(ollieRosTools::VoNode_paramsConfig &config, uint32_t level){
            ROS_INFO("MAP > SETTING PARAMS");
            maxKFNr = config.map_maxKF;
            shirnkKFs();

            g2oDense     = config.g2o_dense;
            g2oIter      = config.g2o_iterations;
            g2oHuber     = config.g2o_huber;
            g2oStructure = config.g2o_structureOnly;
            g2oFix       = static_cast<FixMethod>(config.g2o_fix);

            matcher.setParameter(config, level);
            ROS_INFO("MAP < PARAMS SET");
        }


};

#endif // MAP_HPP
