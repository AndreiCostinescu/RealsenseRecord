//
// Created by andrei on 25.08.21.
//

#ifndef REALSENSERECORD_WRITERECORDING_H
#define REALSENSERECORD_WRITERECORDING_H

#include <RealsenseRecording/recording/Recording.h>

namespace RealsenseRecording {
    class WriteRecording : public Recording {
    public:
        explicit WriteRecording(const std::string &imageWriteFormat = "avi",
                                const std::string &depthWriteFormat = "bin",
                                const std::string &parametersWriteFormat = "xml",
                                const RecordingParameters *recordingParameters = nullptr,
                                rs2::video_stream_profile *videoStreamProfile = nullptr,
                                cv::VideoCapture *videoCapture = nullptr,
                                AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);

        WriteRecording(double fps, int width, int height, float fx, float fy, float ppx, float ppy,
                       rs2_distortion model,
                       const float coefficients[5], const std::string &imageWriteFormat = "avi",
                       const std::string &depthWriteFormat = "bin", const std::string &parametersWriteFormat = "xml",
                       AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);

        WriteRecording(double fps, rs2_intrinsics intrinsics, const std::string &imageWriteFormat = "avi",
                       const std::string &depthWriteFormat = "bin", const std::string &parametersWriteFormat = "xml",
                       AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);

        ~WriteRecording() override;

        void setParameters(const rs2::video_stream_profile *_videoStreamProfile);

        void setParameters(const cv::VideoCapture *_videoCapture);

        void setParameters(const RecordingParameters *_recordingParameters);

        bool writeData(cv::Mat *image, rs2::depth_frame *depth, unsigned long long counter = -1);

        bool writeData(cv::Mat *image, cv::Mat *depth, unsigned long long counter = -1);

        bool writeData(cv::Mat &image, cv::Mat &depth, unsigned long long counter = -1);

    private:
        static int dataBufferSize;

        void bufferThreadWrite();

        void initializeThreadAndBuffers();

        void writeImage(cv::Mat *image);

        void writeDepth(cv::Mat *depth);

        bool initializeImageWriter();

        bool initializeDepthWriter();

        void releaseImageWriter();

        void releaseDepthWriter();

        cv::VideoWriter *imageWriter{};
        std::ofstream *imageWriterBinary{}, *depthWriterBinary{};
        bool imageWriterInitialized, depthWriterInitialized;

        std::thread writerThread;
        std::mutex lock;

        AndreiUtils::RotationType writeRotation;

        std::vector<cv::Mat *> imageBuffer, depthBuffer;
        std::vector<unsigned long long> countBuffer;
        bool writeFlag;
        int bufferStartIndex, bufferEndIndex, bufferSize;
    };
}

#endif //REALSENSERECORD_WRITERECORDING_H
