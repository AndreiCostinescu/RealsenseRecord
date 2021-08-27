cmake_minimum_required(VERSION 3.17)
project(RealsenseRecord)

set(CMAKE_CXX_STANDARD 14)

if (UNIX)
    set(EXTERNAL_LIBS pthread ${EXTERNAL_LIBS})
endif ()

find_package(realsense2 REQUIRED)
set(EXTERNAL_LIBS ${realsense2_LIBRARY} ${EXTERNAL_LIBS})

find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
set(EXTERNAL_LIBS Eigen3::Eigen ${EXTERNAL_LIBS})

if (PERCEPTRON_BUILD)
    find_package(OpenCV 3.2 REQUIRED)
else (PERCEPTRON_BUILD)
    find_package(OpenCV 4 REQUIRED HINTS "/usr/local/src/opencv/build-release")
endif (PERCEPTRON_BUILD)
include_directories(${OpenCV_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${OpenCV_LIBS} ${EXTERNAL_LIBS})

SET(USE_EIGEN ON)
SET(USE_JSON ON)
SET(USE_OPENCV ON)
SET(USE_REALSENSE ON)

SET(ANDREI_UTILS_INCLUDE_DIRS utils/include utils/json)
SET(ANDREI_UTILS_LIB AndreiUtils)
add_subdirectory(utils)
include_directories(${ANDREI_UTILS_INCLUDE_DIRS})

include_directories("include")

add_library(RealsenseRecording src/recording/RecordingParameters.cpp src/recording/Recording.cpp src/recording/ReadRecording.cpp src/recording/WriteRecording.cpp src/utils.cpp)
target_link_libraries(RealsenseRecording ${EXTERNAL_LIBS} ${ANDREI_UTILS_LIB})

add_executable(RealsenseRecord src/main.cpp src/RealsenseCapture.cpp)
target_link_libraries(RealsenseRecord RealsenseRecording)
