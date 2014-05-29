cmake_minimum_required(VERSION 2.8)
    
add_definitions(-O3 -Wno-unknown-pragmas -Wunused-variable -g -msse2 -msse3 -msse4 -ffast-math -march=native)


find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/installfiles)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

file(GLOB AKAZE_SOURCE_FILES  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
file(GLOB AKAZE_HEADER_FILES  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/src)

#build the library dynamic and static versions
add_library(akaze ${AKAZE_SOURCE_FILES} ${AKAZE_HEADER_FILES})

find_package(OpenCV REQUIRED)
target_link_libraries(akaze ${OpenCV_LIBRARIES})


