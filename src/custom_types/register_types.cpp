
#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"
#include <ollieRosTools/custom_types/edge_pose_landmark_reprojectBV.hpp>
#include <ollieRosTools/custom_types/edge_pose_pose.hpp>
#include <ollieRosTools/custom_types/vertex_landmarkxyz.hpp>
#include <ollieRosTools/custom_types/vertex_pose.hpp>

#include <iostream>


bool init_g2o_types() {
    g2o::Factory* factory = g2o::Factory::instance();
    factory->registerType("EDGE_POSE_LANDMARKBV", new g2o::HyperGraphElementCreator<EdgePoseLandmarkReprojectBV>);
    factory->registerType("EDGE_POSE_POSE", new g2o::HyperGraphElementCreator<EdgePosePose>);
    factory->registerType("VERTEX_LANDMARK_XYZ", new g2o::HyperGraphElementCreator<VertexLandmarkXYZ>);
    factory->registerType("VERTEX_POSE", new g2o::HyperGraphElementCreator<VertexPose>);

    return true;
}

//G2O_REGISTER_ACTION(EdgePoseLandmarkReprojectDrawAction);
G2O_REGISTER_ACTION(EdgePoseLandmarkReprojectBVDrawAction);
G2O_REGISTER_ACTION(EdgePosePoseDrawAction);
G2O_REGISTER_ACTION(VertexLandmarkXYZDrawAction);
G2O_REGISTER_ACTION(VertexPoseDrawAction);
