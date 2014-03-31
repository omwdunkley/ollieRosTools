cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen REQUIRED 3.1.2) #https://launchpad.net/~kalakris/+archive/eigen
include_directories(${Eigen_INCLUDE_DIRS})


#find_package(OpenMP REQUIRED)

#Boost
rosbuild_add_boost_directories()

#### BUILD TYPE
#set(ROS_BUILD_TYPE Release)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -Ofast   -Wall -Wextra -msse2 -msse3 -msse4 -march=native -ffast-math")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -Ofast  -Wall -Wextra -msse2 -msse3 -msse4 -march=native -ffast-math")

#set(ROS_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -Wall -Wextra -msse2 -msse3 -msse4 -march=native -g") #-Wdouble-promotion
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -Wall -Wextra -msse2 -msse3 -msse4 -march=native -g") #-Wdouble-promotion

set(ROS_BUILD_TYPE Debug)
#set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -Wall -Wextra -Wdouble-promotion -g")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}  -Wall -Wextra -Wdouble-promotion -g")

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
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
)


#rosbuild_add_executable(preproc ${PREPROC_FILES} )
#target_link_libraries(preproc ${LIBRARIES} ${OpenCV_LIBS})

rosbuild_add_executable(vo ${VO_FILES} )
target_link_libraries(vo ${LIBRARIES} ${OpenCV_LIBS})



#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
