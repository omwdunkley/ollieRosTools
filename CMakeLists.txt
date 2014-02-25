cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})


find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -O3  -Wall -Wextra -Wunused-variable  -g -msse2 -msse3 -msse4 -ffast-math")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -O3 -Wall -Wextra -Wunused-variable -g -msse2 -msse3 -msse4 -ffast-math")

#Boost
rosbuild_add_boost_directories()


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

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


set(PREPROC_LIBRARIES 
    #agast
    #akaze
    boost_system
    #rbrief
    #opengsurf
    #opengv #included via ros    
    #cvd
    blas
    lapack
) # GL glut cvd GVars3_headless blas lapack



SET(PREPROC_FILES     
    src/mainPreProc.cpp     
    src/PreProc.cpp 
    src/OCam.cpp
    src/PreProcNode.cpp
    src/Aux.cpp
    src/CameraModel.cpp

)	

rosbuild_add_executable(preproc ${PREPROC_FILES} )
target_link_libraries(preproc ${PREPROC_LIBRARIES} ${OpenCV_LIBS})




#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
