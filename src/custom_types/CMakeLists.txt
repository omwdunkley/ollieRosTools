
SET(CUSTOM_G2O_TYPES_SOURCES 
              vertex_landmarkxyz.cpp
              vertex_pose.cpp
              edge_pose_landmark_reprojectBV.cpp
              edge_pose_pose.cpp
              register_types.cpp)

ROSBUILD_ADD_LIBRARY(g2o_custom_types ${CUSTOM_G2O_TYPES_SOURCES})
target_link_libraries(g2o_custom_types GL GLU glut)
