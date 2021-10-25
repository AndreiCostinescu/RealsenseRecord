//
// Created by andrei on 26.11.20.
//

#ifndef DARKNETOPENPOSEREALSENSE_RECORDING_H
#define DARKNETOPENPOSEREALSENSE_RECORDING_H

#include <AndreiUtils/enums/RotationType.h>
#include <fstream>
#include <librealsense2/rs.hpp>
#include <mutex>
#include <RealsenseRecording/recording/RecordingParameters.h>
#include <thread>

namespace RealsenseRecording {
    class Recording {
    public:
        static std::string getOutputDirectory();

        static std::string format(int number, const char *type, const std::string &format);

        explicit Recording(const std::string &imageFormat = "avi", const std::string &depthFormat = "bin",
                           const std::string &parameterFormat = "json",
                           AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);

        Recording(const std::string &imageFormat, const std::string &depthFormat, const std::string &parameterFormat,
                  const void *parameters, RecordingParametersType parametersType,
                  AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);

        Recording(double fps, int width, int height, float fx, float fy, float ppx, float ppy, rs2_distortion model,
                  const float coefficients[5], const std::string &imageFormat = "avi",
                  const std::string &depthFormat = "bin", const std::string &parameterFormat = "xml",
                  AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);

        Recording(double fps, rs2_intrinsics intrinsics, const std::string &imageFormat = "avi",
                  const std::string &depthFormat = "bin", const std::string &parameterFormat = "xml",
                  AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);

        virtual ~Recording();

        void setFiles(bool read, int fileNumber = -1);

        rs2_intrinsics getIntrinsics();

        const RecordingParameters *getParameters();

    protected:
        static std::string outputDirectory;
        static bool outputDirectoryInitialized;

        RecordingParameters parameters;
        std::string imageFile, depthFile, parameterFile;
    };
}

#endif //DARKNETOPENPOSEREALSENSE_RECORDING_H
