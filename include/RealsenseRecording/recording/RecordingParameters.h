//
// Created by andrei on 27.11.20.
//

#ifndef DARKNETOPENPOSEREALSENSE_RECORDINGPARAMETERS_H
#define DARKNETOPENPOSEREALSENSE_RECORDINGPARAMETERS_H

#include <AndreiUtils/enums/RotationType.h>
#include <AndreiUtils/json.hpp>
#include <librealsense2/rs.hpp>
#include <string>
#include <vector>
#include "RecordingParametersType.h"

#ifdef OPENCV
#include <opencv2/opencv.hpp>
#endif

namespace RealsenseRecording {
    class RecordingParameters {
    public:
        const static std::map<std::string, rs2_distortion> DISTORTION_MODELS;
        const static std::vector<std::string> PARAMETER_FORMATS;

        RecordingParameters();

        RecordingParameters(std::string imageFormat, std::string depthFormat, std::string parametersFormat,
                            AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);

        RecordingParameters(std::string imageFormat, std::string depthFormat, std::string parametersFormat,
                            const void *parameters, RecordingParametersType parameterType,
                            AndreiUtils::RotationType rotationType = AndreiUtils::NO_ROTATION);

        #ifdef OPENCV
        RecordingParameters(std::string imageFormat, std::string depthFormat, std::string parametersFormat,
                            const RecordingParameters *recordingParameters = nullptr,
                            rs2::video_stream_profile *videoStreamProfile = nullptr,
                            cv::VideoCapture *videoCapture = nullptr,
                            AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);
        #endif

        RecordingParameters(double fps, int width, int height, float fx, float fy, float ppx, float ppy,
                            rs2_distortion model, const float coefficients[5], std::string imageFormat,
                            std::string depthFormat, std::string parametersFormat,
                            AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);

        RecordingParameters(double fps, rs2_intrinsics intrinsics, std::string imageFormat, std::string depthFormat,
                            std::string parametersFormat,
                            AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);

        ~RecordingParameters();

        void clear();

        void setParameters(const rs2::video_stream_profile *videoStreamProfile,
                           AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);

        #ifdef OPENCV
        void setParameters(const cv::VideoCapture *videoCapture,
                           AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);
        #endif

        void setParameters(const RecordingParameters *recordingParameters,
                           AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);

        void serialize(const std::string &parametersFile) const;

        void deserialize(const std::string &parametersFile, const std::string &_parametersFormat);

        #ifdef OPENCV
        void writeParameters(cv::FileStorage &fs) const;

        void readParameters(const cv::FileNode &node);
        #endif

        void to_json(nlohmann::json &j) const;

        void from_json(const nlohmann::json &j);

        rs2_intrinsics getIntrinsics();

        bool isInitialized() const;

        double fps;
        int width{}, height{};
        float ppx{}, ppy{}, fx{}, fy{}, coefficients[5];
        rs2_distortion model;
        std::string imageFormat, depthFormat, parametersFormat;
        AndreiUtils::RotationType rotation;

    private:
        void setRotationDependentParameters(AndreiUtils::RotationType rotation, int _width, int _height);

        void setRotationDependentParameters(AndreiUtils::RotationType rotation, int _width, int _height,
                                            float _fx, float _fy, float _ppx, float _ppy);

        bool initialized;
    };
}

#ifdef OPENCV
namespace cv {
    void write(cv::FileStorage &fs, const std::string &, const RealsenseRecording::RecordingParameters &x);

    void read(const cv::FileNode &node, RealsenseRecording::RecordingParameters &x,
              const RealsenseRecording::RecordingParameters &default_value = RealsenseRecording::RecordingParameters());
}
#endif

void to_json(nlohmann::json &j, const RealsenseRecording::RecordingParameters &p);

void from_json(const nlohmann::json &j, RealsenseRecording::RecordingParameters &p);

#endif //DARKNETOPENPOSEREALSENSE_RECORDINGPARAMETERS_H
