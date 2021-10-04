cmake_minimum_required(VERSION 3.17)
project(RealsenseRecord)

set(CMAKE_CXX_STANDARD 14)

set(OPENCV_VERSION "" CACHE STRING "The opencv version to use in the project")

if (UNIX)
    set(EXTERNAL_LIBS pthread ${EXTERNAL_LIBS})
endif ()

find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
set(EXTERNAL_LIBS Eigen3::Eigen ${EXTERNAL_LIBS})

if ("${OPENCV_VERSION}" STREQUAL "")
    find_package(OpenCV REQUIRED)
else ()
    find_package(OpenCV ${OPENCV_VERSION} REQUIRED)
endif ()
include_directories(${OpenCV_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${OpenCV_LIBS} ${EXTERNAL_LIBS})

find_package(realsense2 REQUIRED)
set(EXTERNAL_LIBS ${realsense2_LIBRARY} ${EXTERNAL_LIBS})

set(ANDREI_UTILS_PATH "" CACHE PATH "The (absolute!) path to the AndreiUtils repository directory")
if ("${ANDREI_UTILS_PATH}" STREQUAL "")
    message(FATAL_ERROR "No path to AndreiUtils directory provided")
endif ()
message("AndreiUtils path set to ${ANDREI_UTILS_PATH}")
set(ANDREI_UTILS_DIR "${ANDREI_UTILS_PATH}")
if (UNIX)
    set(ANDREI_UTILS_LIB_FORMAT ".a")
else (UNIX)
    set(ANDREI_UTILS_LIB_FORMAT ".so")
endif ()
set(ANDREI_UTILS_INCLUDE "${ANDREI_UTILS_DIR}/include" "${ANDREI_UTILS_DIR}/json")
set(ANDREI_UTILS_LIB "${ANDREI_UTILS_DIR}/build-release/libAndreiUtils${ANDREI_UTILS_LIB_FORMAT}")
message("AndreiUtils include dirs set to ${ANDREI_UTILS_INCLUDE}")
message("AndreiUtils library path set to ${ANDREI_UTILS_LIB}")
include_directories(${ANDREI_UTILS_INCLUDE})
set(EXTERNAL_LIBS ${ANDREI_UTILS_LIB} ${EXTERNAL_LIBS})

include_directories("include" "private_include")

add_library(RealsenseRecording src/RealsenseCapture.cpp src/recording/RecordingParameters.cpp src/recording/Recording.cpp src/recording/ReadRecording.cpp src/recording/WriteRecording.cpp src/configDirectoryLocation.cpp src/utils.cpp)

add_executable(RealsenseRecord src/main.cpp)
target_link_libraries(RealsenseRecord RealsenseRecording ${EXTERNAL_LIBS})
