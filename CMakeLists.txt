cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)


#LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/home/ollie/Dropbox/Code/ROS/ollieRosTools/thirdparty/eigen/cmake)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


FIND_PACKAGE(Cholmod)
FIND_PACKAGE(BLAS)
FIND_PACKAGE(LAPACK)
IF(CHOLMOD_FOUND AND BLAS_FOUND AND LAPACK_FOUND)
  MESSAGE(STATUS "Found CHOLMOD and its dependencies")
  SET(CHOLMOD_FOUND TRUE)
ELSE()
  SET(CHOLMOD_FOUND FALSE)
ENDIF()
FIND_PACKAGE(CSparse)
INCLUDE_DIRECTORIES(${g2o_DIR} )

INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR})


FIND_PACKAGE(OpenCV REQUIRED)

FIND_PACKAGE(Eigen REQUIRED 3.2.1) #https://launchpad.net/~kalakris/+archive/eigen
include_directories(${Eigen_INCLUDE_DIRS})


#ind_package(libg2o REQUIRED)
#ET(G2O_LIBS ${libg2o_LIBRARIES} cholmod cxsparse)
#ADD_DEFINITIONS(-DDO_FEATURE_OPTIMIZATION)

#find_package(OpenMP REQUIRED)

#Boost
rosbuild_add_boost_directories()

#### BUILD TYPE
#set(ROS_BUILD_TYPE Release)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -Ofast   -Wall -Wextra -msse2 -msse3 -msse4 -march=native -ffast-math")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -Ofast  -Wall -Wextra -msse2 -msse3 -msse4 -march=native -ffast-math")

set(ROS_BUILD_TYPE RelWithDebInfo)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -Wall -Wextra -msse2 -msse3 -msse4 -march=native -g") #-Wdouble-promotion
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -Wall -Wextra -msse2 -msse3 -msse4 -march=native -g") #-Wdouble-promotion

#set(ROS_BUILD_TYPE Debug)
#set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -Wall -Wextra -Wdouble-promotion -g")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}  -Wall -Wextra -Wdouble-promotion -g")

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -fpermissive")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fpermissive")
endif()


rosbuild_init()
set(CMAKE_VERBOSE_MAKEFILE TRUE)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


# Dynamic Reconfigure
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()


#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()



include_directories(
  ${PROJECT_SOURCE_DIR}/thirdparty/agast/include
)
link_directories(
  ${PROJECT_SOURCE_DIR}/thirdparty/agast/lib
)

include_directories(
  ${PROJECT_SOURCE_DIR}/thirdparty/akaze-opencv/src
)
link_directories(
  ${PROJECT_SOURCE_DIR}/thirdparty/akaze-opencv/lib
)




SET(G2O_LIBS cholmod cxsparse -lg2o_cli -lg2o_core
-lg2o_csparse_extension -lg2o_ext_freeglut_minimal -lg2o_incremental
-lg2o_interactive -lg2o_interface -lg2o_opengl_helper -lg2o_parser
-lg2o_simulator -lg2o_solver_cholmod -lg2o_solver_csparse
-lg2o_solver_dense -lg2o_solver_pcg -lg2o_solver_slam2d_linear
-lg2o_solver_structure_only -lg2o_stuff -lg2o_types_data -lg2o_types_icp
-lg2o_types_sba -lg2o_types_sclam2d -lg2o_types_sim3 -lg2o_types_slam2d
-lg2o_types_slam3d)

####### g2o custom types ###############################
ADD_SUBDIRECTORY(src/custom_types)



set(LIBRARIES
    agast
    akaze
    boost_system
    rbrief
    #opengsurf
    #opengv #included via ros    
    #cvd
    blas
    lapack
) # GL glut cvd GVars3_headless blas lapack



SET(PREPROC_FILES     
    src/mainPreProc.cpp     
    src/PreProc.cpp 
    src/PreProcNode.cpp
    src/aux.cpp
    src/CameraATAN.cpp
)

SET(VO_FILES
    src/mainVO.cpp
    src/PreProc.cpp
    src/VoNode.cpp
    src/aux.cpp
    src/CameraATAN.cpp
    src/Frame.cpp
    src/Detector.cpp
    src/Tracker.cpp
    src/Matcher.cpp
    src/Odometry.cpp
    src/Map.cpp
)


#rosbuild_add_executable(preproc ${PREPROC_FILES} )
#target_link_libraries(preproc ${LIBRARIES} ${OpenCV_LIBS})

rosbuild_add_executable(vo ${VO_FILES} )
target_link_libraries(vo ${G2O_LIBS} ${LIBRARIES} ${OpenCV_LIBS} g2o_custom_types)

SET(BA_FILES
    src/ba_demo.cpp
)
rosbuild_add_executable(ba_demo ${BA_FILES} )
target_link_libraries(ba_demo ${G2O_LIBS} ${LIBRARIES} ${OpenCV_LIBS})

SET(SBA_FILES
    src/sba_demo.cpp
)
rosbuild_add_executable(sba_demo ${SBA_FILES} )
target_link_libraries(sba_demo ${G2O_LIBS} ${LIBRARIES} ${OpenCV_LIBS})
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
