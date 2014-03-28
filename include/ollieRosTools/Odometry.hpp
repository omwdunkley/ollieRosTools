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

namespace RP = opengv::sac_problems::relative_pose;

class Odometry
{

private:

    int voTriangulationMethod;
    bool voRelNLO;
    double voRelRansacThresh;
    int voRelRansacIter;
    int voRelPoseMethod;
    //bool voDoIntitialise;

    void triangulate(const opengv::transformation_t& trans1to2, const opengv::bearingVectors_t& bv1, const opengv::bearingVectors_t& bv2, opengv::points_t& points3d){
        /// DO TRIANGULATIION
        /// The 3D point expressed in the first viewpoint.

        ROS_INFO("Triangulating");


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

public:
    Odometry(){
        voRelRansacIter = 300;
        voRelRansacThresh = 1;
        voRelPoseMethod = 0;
        voTriangulationMethod = 1;
    }


    // removed outliers from matches
    bool initialise(FramePtr& f1, FramePtr& f2, DMatches& matches, opengv::points_t& worldPoints ){
        ROS_INFO("Initialising");

        /// Get unit features aligned
        opengv::bearingVectors_t bv1;
        opengv::bearingVectors_t bv2;
        Ints ms1to1;
        OVO::alignedBV(f1->getBearings(), f2->getBearings(), matches, bv1, bv2, ms1to1);
        opengv::relative_pose::CentralRelativeAdapter adapter(bv2, bv1);

        /// Compute relative transformation from KeyFrame KF to Frame F using ransac
        Ints inliers;
        opengv::transformation_t trans2to1;

        if (voRelPoseMethod==5){
            /// Use IMU for rotation, compute translation
            // compute relative rotation

            adapter.setR12(f2->getImuRotation().transpose() * f1->getImuRotation());

            ///DO RANSAC for translation only
            boost::shared_ptr<RP::TranslationOnlySacProblem>relposeproblem_ptr(new RP::TranslationOnlySacProblem(adapter) );


            opengv::sac::Ransac<RP::TranslationOnlySacProblem> ransac;
            ransac.sac_model_ = relposeproblem_ptr;
            ransac.threshold_ = voRelRansacThresh;
            ransac.max_iterations_ = voRelRansacIter;
            ransac.computeModel(1);

            // Set as output
            trans2to1 = ransac.model_coefficients_;
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
            trans2to1 = ransac.model_coefficients_;
            std::swap(inliers, ransac.inliers_);  //Get inliers
        }

        if (inliers.size()<5){
            ROS_WARN("RANSAC failed init with %lu/%lu inliers", inliers.size(), matches.size());
            return false;
        }

        /// Experimental: NLO
        /// TODO: estimate yaw bias
        if (voRelNLO){
            // check order
            ROS_INFO("Doing NLO");

            adapter.sett12(trans2to1.col(3));
            adapter.setR12(trans2to1.block<3,3>(0,0));
            ROS_INFO_STREAM("   Before:\n  " << trans2to1);
            trans2to1 = opengv::relative_pose::optimize_nonlinear(adapter, inliers);
            ROS_INFO_STREAM("   After :\n  " << trans2to1);
        }

        /// g2o BUNDLE ADJUST

        ROS_INFO("Relative Ransac Done with %lu/%lu inliers", inliers.size(), matches.size());
        //printRPYXYZ(poseTF, "Ransac Output: ");

        /// Align inliers
        opengv::bearingVectors_t bv1inlier;
        opengv::bearingVectors_t bv2inlier;
        DMatches msTemp;
        OVO::vecReduceInd<opengv::bearingVectors_t>(bv1, bv1inlier, inliers);
        OVO::vecReduceInd<opengv::bearingVectors_t>(bv2, bv2inlier, inliers);
        OVO::vecReduceInd<DMatches>(matches, msTemp, inliers);
        std::swap(matches, msTemp);


        /// Scale Pose
        double length = trans2to1.col(3).norm();

        ROS_INFO("Triangulation baseline %f", length);
        trans2to1.col(3) *= 1./length;

        /// set poses
        //f2->setPose(); //
        f1->setPose(f2->getPose() * trans2to1);

        /// triangulate
        // get points in f2 frame
        triangulate(trans2to1, bv2inlier, bv1inlier, worldPoints);
        // put them in world frame
        OVO::transformPoints(f2->getPose(), worldPoints);
        return true;

    }


    void estimatePose(FramePtr& f1, FramePtr& f2, const DMatches& matches){

    }

    void setParameter(ollieRosTools::VoNode_paramsConfig &config, uint32_t level){
        voRelRansacIter   = config.vo_relRansacIter;
        voRelRansacThresh = 1.0 - cos(atan(config.vo_relRansacThresh*sqrt(2.0)*0.5/720.0));
        voRelPoseMethod   = config.vo_relPoseMethod;
        voTriangulationMethod = config.vo_triMethod;
        voRelNLO          = config.vo_relNLO;
        //voDoIntitialise = config.vo_doInitialisation;
        //config.vo_doInitialisation = false;
        ROS_INFO("Reprojection error threshold: %f", voRelRansacThresh);

    }
};

#endif // ODOMETRY_HPP
