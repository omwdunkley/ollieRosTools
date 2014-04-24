#ifndef MAP_HPP
#define MAP_HPP



#include <deque>
#include <map>
#include <stdio.h>


#include <Eigen/StdVector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/package.h>

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
cv::Mat getPointsProjectedImage(const FramePtr& f, const opengv::points_t& worldPts, const Ints& idx);



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

        /// /////////////////////////////////////////////////////////////////////////////////
        /// GETTER / SETTER

        // Get
        const FramePtrs& getKFs() const {
            return keyframes;
        }

        // Get All Landmarks
        const LandMarkPtrs& getLMs() const {
            return landmarks;
        }

        // Get last keyframe If recent = 1, gets last-1, etc
        FramePtr& getLatestKF(uint last=0){
            ROS_ASSERT(getKeyframeNr()>last);
            if (last==0){
                return keyframes.back();
            } else {
                return keyframes[keyframes.size()-1-last];
            }
        }

        // Get last keyframe CONST version. If recent = 1, gets last-1, etc
        /// TODO: should this return the latest or the most recently used??
        const FramePtr& getLatestKF(uint last=0) const{
            ROS_ASSERT(getKeyframeNr()>last);
            if (last==0){
            return keyframes.back();
            } else {
                return keyframes[keyframes.size()-1-last];
            }
        }

        // Gets Latest frame matches vs map
        FramePtr& getCurrentFrame(){
            ROS_INFO("MAP > Getting current frame");
            ROS_ASSERT(!currentFrame.empty());
            ROS_ASSERT(currentFrame->isInitialised());
            return currentFrame;
        }

        // Get nr of keyframes
        size_t getKeyframeNr() const{
            return keyframes.size();
        }

        // Get nr of landmarks
        size_t getLandmarkNr() const{
            return landmarks.size();
        }

        // Gets all the descriptors most likely to be observable from the given frame
        uint getAllPossibleObservations(const FramePtr& f, cv::Mat& desc, LandMarkPtrs& lms){
            ROS_INFO("MAP > Getting all possible observations of [%lu] landmarks from frame [%d|%d]", landmarks.size(), f->getId(), f->getKfId());
            ros::WallTime t0 = ros::WallTime::now();
            desc = cv::Mat();

            lms.clear();

            // add points that are visible. Also sets within the LM from which frame it was visible
            for (uint i=0; i<landmarks.size(); ++i){
                LandmarkPtr& lm = landmarks[i];
                if (lm->visibleFrom(f)){
                    lms.push_back(lm);
                    desc.push_back(lm->getObservationDesc());
                }
            }
            Landmark::printStats();
            ROS_INFO(OVO::colorise("MAP < Found [%lu/%lu] possible observations from frame [%d|%d] in [%.1fms]", OVO::FG_MAGNETA).c_str(),lms.size(), landmarks.size(), f->getId(), f->getKfId(), (ros::WallTime::now()-t0).toSec()*1000.);
            return lms.size();
        }







        /// /////////////////////////////////////////////////////////////////////////////////
        /// Landmark Related Functions

        // Goes through all map points and remove them if they only have one reference (ie this container holding it)
        /// NOT TESTED
        void removeNonVisiblePoints(){
            ROS_INFO("MAP > Removing non visible points");
            size_t s = landmarks.size();
            landmarks.erase( std::remove_if( landmarks.begin(), landmarks.end(), noRef), landmarks.end() );
            ROS_INFO("MAP < Removed [%lu/%lu] points left", landmarks.size(), s);
        }




        /// /////////////////////////////////////////////////////////////////////////////////
        /// Matching Functions frame-frame points, frame-map points, frame->closest frames,


        // Matches against the map. Returns matches, corresponding points, and the identifiers
        double match2Map(FramePtr& f, DMatches& matches, LandMarkPtrs& lms, double& time/*, Points3d points, DMatches& ms*/){
            ROS_INFO("MAP > Matching Frame [%d|%d] against MAP with [%lu] landmarks", f->getId(), f->getKfId(), landmarks.size() );
            ROS_ASSERT(landmarks.size()>0);
            ROS_ASSERT(keyframes.size()>0);
            matches.clear();

            //TODO: shouldnt we also time this?
            /// Get landmarks that are potentially visible. descs and LMS are now aligned
            cv::Mat descs;
            getAllPossibleObservations(f, descs, lms);

            /// Do Matching vs Map. Aligns lms with matches
            const double disparity =  matcher.matchMap(descs, lms, f, matches, time, Ints() );
            ROS_INFO("MAP < Found [%lu/%lu] matches for Frame [%d|%d] vs MAP with disparity [%f]", matches.size(), lms.size(), f->getId(), f->getKfId(), disparity);
            return disparity;
        }


        // Matches against a keyframe. If voOnly = true, only match against points that have associsated land marks. Returns disparity
        double match2KF(FramePtr& f, DMatches& matches, double& time, bool voOnly=false){
            /// TODO Match against last N keyframes
            /// TODO Match against closest N keyframes
            ROS_ASSERT(keyframes.size()>0);
            FramePtr& kf = getLatestKF();
            currentFrame = f;
            ROS_INFO("MAP = Matching Frame [%d|%d] against KeyFrame [%d|%d]", f->getId(), f->getKfId(), kf->getId(), kf->getKfId() );
            if (voOnly){
                // Create a mask where we only match against KF points that have associated landmarks
                return matcher.matchFrame(f, kf, matches, time, Ints(0), kf->getIndLM());
            } else {
                return matcher.matchFrame(f, kf, matches, time);
            }

        }





        /// /////////////////////////////////////////////////////////////////////////////////
        /// Keyframe Functions frame->closest frames, add kf, remove kf, etc


        void pushKF(FramePtr& frame, const bool first=false){
            ROS_INFO("MAP > ADDING%s KF TO MAP", first?" FIRST":"");
            frame->setAsKF(first);
            if (keyframes.size()==0 || first){
                reset();
                currentFrame = frame;
                keyframes.push_back(frame);
                ROS_INFO("MAP < INITIAL KF PUSHED");
            } else {
                keyframes.push_back(frame);
                currentFrame = frame;
                ROS_INFO("MAP < KF PUSHED [KFS = %lu]", getKeyframeNr());

                // optimise
                bundleAdjust();

                // Check we dont have too many keyframes
                shirnkKFs();
            }
        }

        // Removes KFs if needed
        void shirnkKFs(){
            if (keyframes.size()>maxKFNr || keyframes.size()==MAX_KF) {
                ROS_INFO("MAP > TOO MANY KEYFRAMES, REMOVING OLDEST");
                while(keyframes.size()>maxKFNr){
                    popKF();
                }
                ROS_INFO("MAP < CAPPED KFS");
            }
        }

        // Removes oldest keyframe
        void popKF(){
            ROS_INFO("MAP > POPPING OLDEST KF FIFO [%d|%d]", keyframes.front()->getId(), keyframes.front()->getKfId() );
            keyframes.pop_front();
            removeNonVisiblePoints();
            ROS_INFO("MAP < OLDEST KF POPPED");
        }


        // Goes through all keyframes and gets the cloest N frames
        FramePtrs getClosestKeyframes(const FramePtr& f){
            ROS_ERROR("MAP = NOT IMPLEMENTED GET CLOSEST KEYFRAMES: %s}\n For now returning all", __SHORTFILE__);
            // TODO
            // image descriptor (rotation invarient)
            // sbi ncc/ssd  ( rotated)
            // estimated angle
            ///x,y,z < 2m,
            // gyro angle, yaw only +- 45 degrees
            // optical axis vs optical axis < 45 degrees

            return keyframes;
        }





        /// /////////////////////////////////////////////////////////////////////////////////
        /// High Level Map Functions reset, initialise, bundle adjust


        // Resets the map
        void reset(){
            ROS_INFO("MAP > RESETING MAP. Clearing [%lu] key frames and [%lu] land marks", keyframes.size(), landmarks.size());
            keyframes.clear();
            landmarks.clear();
            Landmark::reset();
            currentFrame = FramePtr();
            ROS_INFO("MAP < MAP RESET");
        }


        // Initialise the map. Must have an initial keyframe already, and this should then
        // add the second one. vomatches associate them and must be aligned to points (in world frame)
        void initialiseMap(FramePtr& f, const Points3d& points, const DMatches& voMatches){
            ROS_INFO("MAP > Initialiseing map with new Frame [%d]", f->getId());

            ROS_ASSERT(points.size() == voMatches.size());
            ROS_ASSERT(keyframes.size()==1);
            ROS_ASSERT(landmarks.size()==0);
            ROS_ASSERT(f->poseEstimated());


            // Short cut to current key frame
            FramePtr& kf = getLatestKF();
            ROS_ASSERT(kf->poseEstimated());

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



        // DO bundle adjustment over all points and observations
        void bundleAdjust(){
            ROS_INFO("MAP > Doing G2O Bundle adjustment with [%lu] KeyFrames and [%lu] LandMarks", keyframes.size(), landmarks.size());
            ROS_INFO_STREAM(*this);
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
            int obsCount  = 0;
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
                v_pose->setId(kf->getKfId()); //set id using unique statid id of keyframe class
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

                // Make sure we can still triangulate, need at least two observations
                if (obsNr>1){
                    VertexLandmarkXYZ * v_lm = new VertexLandmarkXYZ();
                    v_lm->setId(lm->getId()+MAX_KF); //set id using unique static id of landmark class + offset
                    v_lm->setMarginalized(true);
                    v_lm->setFixed(false);
                    v_lm->setEstimate(lm->getPosition());
                    ok = optimizer.addVertex(v_lm);
                    ++lmCount;
                    ROS_ASSERT_MSG(ok, "g2o = Could not add v_landmark [%d] to pose graph", lm->getId());

                    /// Setup add observations
                    for (uint fid=0; fid<obsNr; ++fid){
                        // Add each observation of this landmark. The landmark knows from which KFs it is visible and we use these kf IDS to associate the edge to the correct KF
                        const FramePtr& kf = lm->getObservationFrame(fid); //reference to the frame that observes this land mark
                        const Bearing&  bv = lm->getObservationBearing(fid); //reference to observation
                        EdgePoseLandmarkReprojectBV * ef = new EdgePoseLandmarkReprojectBV();
                        ef->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(kf->getKfId()))); //get association via KF id
                        ef->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_lm));
                        ef->setMeasurement(Eigen::Vector2d(bv[0]/bv[2], bv[1]/bv[2])); //bearing vector -> image plane (from norm==1 to depth==1)
                        ef->setInformation(Eigen::Matrix2d::Identity());
                        if (g2oHuber) {
                            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                            ef->setRobustKernel(rk);
                        }
                        ok = optimizer.addEdge(ef);
                        ++obsCount;
                        ROS_ASSERT_MSG(ok, "g2o = Could not add edge between Landmark [%d] and frame [%d|%d] to pose graph", lm->getId(), kf->getId(), kf->getKfId());
                    }
                }

            }
            ROS_INFO("g2o = Added [%d/%lu] landmarks", lmCount, landmarks.size());
            ROS_INFO("g2o = Added [%d] pose-landmark edges", obsCount);
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
            optimizer.save((ros::package::getPath("ollieRosTools")+"/data/map_ba.g2o").c_str());
            ROS_INFO("g2o > Performing full BA:");
            optimizer.optimize(g2oIter);
            ros::WallTime tOpt = ros::WallTime::now();
            ROS_INFO("g2o < Done BA [Setup: %.1fms] [Opti: %.1fms]",  (tSetup-tStart).toSec()*1000., (tOpt-tSetup).toSec()*1000. );


            // Update map using BA results
            /// Update frames
            ROS_INFO("g2o = Updating [%d] Keyframe poses", poseCount);
            for (uint i=0; i<keyframes.size();++i){
                FramePtr& kf = keyframes[i];
                Pose poseEstimate = dynamic_cast<VertexPose*>(optimizer.vertex(kf->getKfId()))->estimate();
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






        /// /////////////////////////////////////////////////////////////////////////////////
        /// META Functions time keeping, drawing, outputting, statistics, etc

        // Gets some kind of visual image showing the state of the keyframes (and current frame)?)
        cv::Mat getVisualImage(){
            ROS_WARN("MAP = NOT IMPLEMENTED GET VISUAL IMAGE");
            cv::Mat img;
            //cv::Mat img = tracker.getVisualImage();
            return img;
        }




        const visualization_msgs::Marker getLandmarkMarkers(int id=0, const std::string& name="Landmarks", const std::string frame ="/world", double size=1.0, const CvScalar RGB = CV_RGB(100,0,100)) const{
            visualization_msgs::Marker markers;
            markers.header.stamp = ros::Time::now();
            markers.ns = name;
            markers.id = id;
            markers.header.frame_id = frame;
            markers.type = visualization_msgs::Marker::SPHERE_LIST;
            markers.action = visualization_msgs::Marker::ADD;
            markers.scale.x = 0.1*size;
            markers.scale.y = 0.1*size;
            markers.scale.z = 0.1*size;
            markers.frame_locked = true;

            std_msgs::ColorRGBA col;
            col.a = 0.7;
            col.r = RGB.val[2]/255.;
            col.g = RGB.val[1]/255.;
            col.b = RGB.val[0]/255.;
            markers.color = col;
            for (uint i=0; i< landmarks.size(); ++i){
                markers.points.push_back(landmarks[i]->getMarker());
            }
            return markers;
        }

        // draws lines from the landmark to the place it was seen from
        const visualization_msgs::Marker getLandmarkObservations(int id=0, const std::string& name="Observations", const std::string frame ="/world", double size=1.0, const CvScalar RGB = CV_RGB(0,0,255)) const{
            visualization_msgs::Marker markers;
            markers.header.stamp = ros::Time::now();
            markers.ns = name;
            markers.id = id;
            markers.header.frame_id = frame;
            markers.type = visualization_msgs::Marker::LINE_LIST;
            markers.action = visualization_msgs::Marker::ADD;
            markers.scale.x = 0.005*size;
            markers.frame_locked = true;

            std_msgs::ColorRGBA col;
            col.a = 0.55;
            col.r = RGB.val[2]/255.;
            col.g = RGB.val[1]/255.;
            col.b = RGB.val[0]/255.;
            markers.color = col;
            for (uint i=0; i< landmarks.size(); ++i){
                const LandmarkPtr& lm = landmarks[i];
                for (uint o=0; o< lm->getObservationsNr(); ++o){
                    geometry_msgs::Point p;

                    tf::pointEigenToMsg(lm->getObservationFrame(o)->getPose().translation(), p);
                    markers.points.push_back(p);


                    tf::pointEigenToMsg(lm->getPosition(),p);
                    markers.points.push_back(p);
                }
            }
            return markers;
        }



        // print id, xyz, nr of ovservations, and observations
        friend std::ostream& operator<< (std::ostream& stream, const OdoMap& map) {
            stream << "\nMap Statistics"
                   << "\n    Keyframes:       [" << std::setw(5) << std::setfill(' ') << map.keyframes.size() << "]"
                   << "\n    Landmarks:       [" << std::setw(5) << std::setfill(' ') << map.landmarks.size() << "]"
                   << "\n    Latest KeyFrame: ["
                   << std::setw(5) << std::setfill(' ') << map.getLatestKF()->getId() << "|"
                   << std::setw(3) << std::setfill(' ') << map.getLatestKF()->getKfId() << "]"
                   << "\n    Current Frame:   ["
                   << std::setw(5) << std::setfill(' ') << map.currentFrame->getId() << "|"
                   << std::setw(3) << std::setfill(' ') << map.currentFrame->getKfId() << "]" ;
             stream << "\nMAP KF Details";
             for (uint i=0; i<map.keyframes.size(); ++i){
                 char buffer [128]; //yeah yeah i know
                 snprintf(buffer, 128, "\n    %3u: KF [%4d|%3d] -> %4d/%4lu observations", i, map.keyframes[i]->getId(),map.keyframes[i]->getKfId(), map.keyframes[i]->getLandmarkRefNr(), map.getLandmarkNr() );
                 stream << buffer;
             }




            return stream;
        }


        // Dynamic reconfigure
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
