//
// Created by andrei on 26.11.20.
//

#ifndef DARKNETOPENPOSEREALSENSE_RECORDING_H
#define DARKNETOPENPOSEREALSENSE_RECORDING_H

#include <AndreiUtils/enums/RotationType.h>
#include <fstream>
#include <librealsense2/rs.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <RealsenseRecording/recording/RecordingParameters.h>
#include <thread>

namespace RealsenseRecording {
    class Recording {
    public:
        static std::string outputDirectory;
        static bool outputDirectoryInitialized;

        static std::string getOutputDirectory();

        static std::string format(int number, const char *type, const std::string &format);

        explicit Recording(std::string imageFormat = "avi", std::string depthFormat = "bin",
                           std::string parameterFormat = "json",
                           AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);

        Recording(std::string imageFormat, std::string depthFormat, std::string parameterFormat,
                  const RecordingParameters *recordingParameters = nullptr,
                  rs2::video_stream_profile *videoStreamProfile = nullptr,
                  cv::VideoCapture *videoCapture = nullptr,
                  AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);

        Recording(double fps, int width, int height, float fx, float fy, float ppx, float ppy, rs2_distortion model,
                  const float coefficients[5], std::string imageFormat = "avi", std::string depthFormat = "bin",
                  std::string parameterFormat = "xml",
                  AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);

        Recording(double fps, rs2_intrinsics intrinsics, std::string imageFormat = "avi",
                  std::string depthFormat = "bin",
                  std::string parameterFormat = "xml",
                  AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);

        virtual ~Recording();

        void setFiles(bool read, int fileNumber = -1);

        rs2_intrinsics getIntrinsics();

        const RecordingParameters *getParameters();

    protected:
        RecordingParameters parameters;
        std::string imageFile, depthFile, parameterFile;
    };
}

#endif //DARKNETOPENPOSEREALSENSE_RECORDING_H
