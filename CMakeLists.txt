cmake_minimum_required(VERSION 4.2)
project(stereo_vslam)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++23 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++23 -O3 -pthread")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
############### dependencies ######################
# Eigen
include_directories("/usr/include/eigen3")

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# gflags
find_package(GFlags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

# fbow
find_package(fbow REQUIRED)
include_directories(${fbow_INCLUDE_DIRS})
link_directories(${fbow_LIB_DIR})

# yaml-cpp
find_package(yaml-cpp REQUIRED)

# ONNX Runtime (SuperPoint and LightGlue). The published CMake package looks
# for lib64 and include/onnxruntime, which this GPU build does not use.
set(ONNXRUNTIME_ROOT
    "/home/nk/workspace/libraries/onnxruntime-linux-x64-gpu_cuda13-1.28.3"
    CACHE PATH "ONNX Runtime install prefix")
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
          HINTS ${ONNXRUNTIME_ROOT}/include)
find_library(ONNXRUNTIME_LIBRARY onnxruntime
             HINTS ${ONNXRUNTIME_ROOT}/lib)
if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
    message(FATAL_ERROR
        "ONNX Runtime not found. Set ONNXRUNTIME_ROOT to the install prefix.")
endif()
include_directories(${ONNXRUNTIME_INCLUDE_DIR})
get_filename_component(ONNXRUNTIME_LIB_DIR ${ONNXRUNTIME_LIBRARY} DIRECTORY)

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${G2O_CORE_LIBRARY}
        ${G2O_STUFF_LIBRARY}
        ${G2O_SOLVER_CSPARSE}
        ${G2O_SOLVER_CSPARSE_EXTENSION}
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        pthread
        ${CSPARSE_LIBRARY}
        fbow
        yaml-cpp::yaml-cpp
        ${ONNXRUNTIME_LIBRARY}
        )

############### source and test ######################
set(STEREO_VSLAM_SOURCES
        src/core/camera.cpp
        src/core/config.cpp
        src/core/frame.cpp
        src/core/map.cpp
        src/core/landmark.cpp
        src/frontend/feature_extractor.cpp
        src/frontend/superpoint.cpp
        src/frontend/light_glue.cpp
        src/frontend/stereo_matcher.cpp
        src/frontend/tracker.cpp
        src/backend/local_mapper.cpp
        src/backend/optimizer.cpp
        src/loop/loop_closure.cpp
        src/loop/place_recognizer.cpp
        src/io/dataset.cpp
        src/viz/viewer.cpp
        src/visual_slam.cpp)

add_library(stereo_vslam_objects OBJECT ${STEREO_VSLAM_SOURCES})
target_include_directories(stereo_vslam_objects PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(stereo_vslam_objects PUBLIC ${THIRD_PARTY_LIBS})

add_executable(vslam src/main.cpp)
target_link_libraries(vslam PRIVATE
        stereo_vslam_objects
        -Wl,--no-as-needed ${G2O_SOLVER_CSPARSE_EXTENSION} -Wl,--as-needed)
set_property(TARGET vslam APPEND PROPERTY BUILD_RPATH "${ONNXRUNTIME_LIB_DIR}")
