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
#include <ollieRosTools/Matcher.hpp>
#include <ollieRosTools/aux.hpp>

static const uint MAX_KF = 1000;


// worldPoint[i] corresponds to feature[idx[i]]
cv::Mat getPointsProjectedImage(FramePtr& f, const opengv::points_t& worldPts, const Ints& idx);

bool noRef(const LandmarkPtr& p);

class Landmark {
private:
    /// Containers
    // XYZ position in world frame
    Point3d xyz;
    // Frame that observed this landmark
    FramePtrs seenFrom;
    // Descriptor / Keypoint / image point Id. id = descriptor row of seenFrom
    Ints pointIds;


    /// Meta
    static int pIdCounter;
    int id;


    /// Visibility settings
    // threshold angle between a keyframe and the direction from which this point was observed in
    static double angleConeThresh;
    static double angleFOVThresh;
    static double distThreshRatio; //ratio from 0,1. Eg 0.3 = error should be within 30% of original distance
    static double distThresh;      //distance error should be within this distance, in meters

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // create a new point
    Landmark(const Eigen::Vector3d& point){
        xyz = point;
        id = ++pIdCounter;
    }

    // returns the unique landmark ID
    int getId() const {
        return id;
    }

    void setPosition(const Point3d& pos){
        xyz = pos;
    }

    // Returns the number of observations, ie the nr of frames that view this point
    uint getObservationsNr() const {
        ROS_ASSERT(check());
        return seenFrom.size();
    }

    // get the frame at observation i
    const FramePtr& getObservationFrame(const int i) const {
        ROS_ASSERT(i>0 && i<static_cast<int>(seenFrom.size()));
        return seenFrom[i];
    }

    // get the bearing vector to this point from observation i
    const Bearing getObservationBearing(const int i) const {
        ROS_ASSERT(i>0 && i<<static_cast<int>(seenFrom.size()));
        return seenFrom[i]->getBearing(pointIds[i]);
    }

    // Returns all frames that saw this point
    const FramePtrs& getObservationIds() const {
        return seenFrom;
    }

    // return descriptor of observation i
    const cv::Mat getObservationDesc(const int i) const{
        ROS_ASSERT(i>0 && i<<static_cast<int>(seenFrom.size()));
        return seenFrom[i]->getDescriptor(pointIds[i]);
    }

    // return all descriptors
    const cv::Mat getObservationDescs() const{
        cv::Mat descs;
        for (uint i=0; i<pointIds.size(); ++i){
            descs.push_back(seenFrom[i]->getDescriptor(pointIds[i]));
        }
        return descs;
    }

    // return the position this point is at
    const Point3d& getPosition(){
        return xyz;
    }

    // add an observation. Seen by frame f with f.keypoint[i]
    void addObservation(FramePtr& f, const int id){
        seenFrom.push_back(f);
        pointIds.push_back(id);
    }

    // returns true if the given frame might provide a similar observation.
    // Frame must have an estiamted position in its pose member
    bool visibleFrom(FramePtr& f) const{
        for (uint i=0; i<pointIds.size(); ++i){
            // Check Distance
            const Eigen::Vector3d xyz_kf = seenFrom[i]->getOpticalCenter();
            const Eigen::Vector3d xyz_f  = f->getOpticalCenter();
            Eigen::Vector3d bvKF2P = (xyz-xyz_kf);   // normalised later
            Eigen::Vector3d bvF2P  = (xyz-xyz_f);    // normalised later
            const double distP2KF  = bvKF2P.norm();
            const double distP2F   = bvF2P.norm();
            const double dist_diff = abs(distP2KF-distP2F);


            // Check P similar distance to F as from where it was observered
            if (dist_diff<distThreshRatio*distP2KF || dist_diff<distThresh){
                // Check F in FOV of P from which it was seen
                bvKF2P.normalize();
                bvF2P.normalize();
                const double angleCone = 1.0 - bvF2P.dot(bvKF2P);
                if (angleCone<angleConeThresh){
                    // check P in FOV of F
                    const Eigen::Vector3d bvFOptAxis = f->getOpticalAxisBearing();
                    const double angleFOV = 1.0 - bvF2P.dot(bvFOptAxis);
                    if (angleFOV<angleFOVThresh){
                        return true;
                        // Point is in FOV of camera,
                        // being seen from a similar distance,
                        // from a similar side
                    }
                }
            }
        }

        // Looked at all possible observations, none are similar
        return false;
    }

    const cv::Mat getClosestDescriptor(){
        ROS_ASSERT_MSG(false, "NOT IMPLEMENTED");
        return cv::Mat();
    }

    // simply reset the id counter
    static void reset(){
        ROS_INFO("Resetting Landmark IDs");
        pIdCounter = 0;
    }

    // print id, xyz, nr of ovservations, and observations
    friend std::ostream& operator<< (std::ostream& stream, const Landmark& point) {
        stream << "LANDMARK [ID:" << std::setw(5) << std::setfill(' ') << point.id << "]"
               << "[XYZ:"
               << std::setw(4) << std::setfill(' ') << std::setprecision(1) << point.xyz[0] << ","
               << std::setw(4) << std::setfill(' ') << std::setprecision(1) << point.xyz[1] << ","
               << std::setw(4) << std::setfill(' ') << std::setprecision(1) << point.xyz[1] << "]"
               << "[Obs: " << std::setw(3) << std::setfill(' ') << point.seenFrom.size() << " = ";

        for (uint i=0; i<point.pointIds.size(); ++i){
            stream << "(" << std::setw(5) << std::setfill(' ') << point.seenFrom[i]->getId() << "|" << std::setw(3) << std::setfill(' ') << point.seenFrom[i]->getKfId() << ")";
        }

        stream << " ]";
        return stream;
    }

    // Checks if the landmark is in a consistence state
    bool check() const {
        ROS_ASSERT(seenFrom.size()>0);
        ROS_ASSERT(pointIds.size()==seenFrom.size());
        return true;

    }



};




















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
            g2oFix = FIX_LAST;
        }

        const FramePtrs& getKFs() const {
            return keyframes;
        }
        const LandMarkPtrs& getLMs() const {
            return landmarks;
        }

        FramePtrs getClosestKeyframes(FramePtr){
            /// Goes through all keyframes and gets the cloest N frames

            /// image descriptor (rotation invarient)
            /// sbi ncc/ssd  ( rotated)
            /// estimated angle
            /// x,y,z < 2m,
            /// gyro angle, yaw only +- 45 degrees
            /// optical axis vs optical axis < 45 degrees
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
            /// TODO clear points too
            ROS_INFO("MAP < MAP RESET");
        }

        size_t getKeyframeNr() const{
            return keyframes.size();
        }

        FramePtr& getLatestKF(){
            ROS_ASSERT(getKeyframeNr()>0);
            return keyframes.back();
        }

        /// Show the frame to the map, track against latest KF, return disparity
//        float showFrame(FramePtr& frame, bool reset = false){
//            currentFrame = frame;
//            const float disparity = tracker.track(currentFrame, reset);
//            return disparity;
//        }


        FramePtr& getCurrentFrame(){
            ROS_ASSERT(!currentFrame.empty()&&currentFrame->isInitialised());
            return currentFrame;
        }

//        const DMatches& getF2KFMatches(const bool forceVoModeOff = false){
//                return tracker.getF2KFMatches(forceVoModeOff);
//        }


        void addKeyFrame(FramePtr& frame, const bool first=false){
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
                    removeKF();
                }
                ROS_INFO("MAP < CAPPED KFS");
            }
        }

        void removeKF(){
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
                        const FramePtr& f = lm->getObservationFrame(fid);
                        const Bearing& bv = lm->getObservationBearing(fid);
                        EdgePoseLandmarkReprojectBV * ef = new EdgePoseLandmarkReprojectBV();
                        ef->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(f->getId())));
                        ef->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_lm));
                        ef->setMeasurement(Eigen::Vector2d(bv[0]/bv[2], bv[1]/bv[2]););
                        ef->information() = Eigen::Matrix2d::Identity();
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
                kf->setPose(optimizer.vertex(kf->getId())->estimate());
            }

            /// Update Land Marks
            ROS_INFO("g2o = Updating [%d landmarks]", lmCount);
            for (uint i=0; i<landmarks.size(); ++i){
                LandmarkPtr& lm = landmarks[i];
                lm->setPosition(optimizer.vertex(MAX_KF+lm->getId())->estimate());
            }

            optimizer.clear();


            ros::WallTime tEnd = ros::WallTime::now();
            ROS_INFO("MAP < Bundle Adjustment Finished [Setup: %.1fms] [Opti: %.1fms] [Update: %.1fms] [Total: %.1fms]",
                     (tSetup-tStart).toSec()*1000., (tOpt-tSetup).toSec()*1000., (tEnd-tOpt).toSec()*1000., (tEnd-tStart).toSec()*1000. );

        }


        void setParameter(ollieRosTools::VoNode_paramsConfig &config, uint32_t level){
            ROS_INFO("MAP > SETTING PARAMS");
            maxKFNr = config.map_maxKF;
            shirnkKFs();

            g2oDense     = config.g2o_dense;
            g2oIter      = config.g2o_iterations;
            g2oHuber     = config.g2o_huber;
            g2oStructure = config.g2o_structureOnly;
            g2oFix       = config.g2o_fix;

            matcher.setParameter(config, level);
            ROS_INFO("MAP < PARAMS SET");
        }


};

#endif // MAP_HPP
