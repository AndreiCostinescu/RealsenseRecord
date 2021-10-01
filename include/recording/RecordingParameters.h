//
// Created by andrei on 27.11.20.
//

#ifndef DARKNETOPENPOSEREALSENSE_RECORDINGPARAMETERS_H
#define DARKNETOPENPOSEREALSENSE_RECORDINGPARAMETERS_H

#include <AndreiUtils/enums/RotationType.h>
#include <AndreiUtils/json.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class RecordingParameters {
public:
    const static std::map<std::string, rs2_distortion> DISTORTION_MODELS;
    const static std::vector<std::string> PARAMETER_FORMATS;

    RecordingParameters();

    RecordingParameters(std::string imageFormat, std::string depthFormat, std::string parametersFormat,
                        AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);

    RecordingParameters(std::string imageFormat, std::string depthFormat, std::string parametersFormat,
                        const RecordingParameters *recordingParameters = nullptr,
                        rs2::video_stream_profile *videoStreamProfile = nullptr,
                        cv::VideoCapture *videoCapture = nullptr,
                        AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);

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

    void setParameters(const cv::VideoCapture *videoCapture,
                       AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);

    void setParameters(const RecordingParameters *recordingParameters,
                       AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);

    void serialize(const std::string &parametersFile) const;

    void deserialize(const std::string &parametersFile, const std::string &_parametersFormat);

    // Write serialization for this class
    void writeParameters(cv::FileStorage &fs) const;

    // Read serialization for this class
    void readParameters(const cv::FileNode &node);

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

void write(cv::FileStorage &fs, const std::string &, const RecordingParameters &x);

void read(const cv::FileNode &node, RecordingParameters &x,
          const RecordingParameters &default_value = RecordingParameters());

// void write(cv::FileStorage &fs, const std::string &, const RecordingParameters *&x);

// void read(const cv::FileNode &node, RecordingParameters *&x, RecordingParameters *default_value = nullptr);

void to_json(nlohmann::json &j, const RecordingParameters &p);

void from_json(const nlohmann::json &j, RecordingParameters &p);

#endif //DARKNETOPENPOSEREALSENSE_RECORDINGPARAMETERS_H
