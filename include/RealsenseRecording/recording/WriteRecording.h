//
// Created by andrei on 25.08.21.
//

#ifndef REALSENSERECORD_WRITERECORDING_H
#define REALSENSERECORD_WRITERECORDING_H

#include <RealsenseRecording/recording/Recording.h>

namespace RealsenseRecording {
    class WriteRecording : public Recording {
    public:
        static WriteRecording *createEmptyPtr(const std::string &imageWriteFormat = "avi",
                                              const std::string &depthWriteFormat = "bin",
                                              const std::string &parametersWriteFormat = "xml", bool withOpenCV = false,
                                              AndreiUtils::RotationType rotationType = AndreiUtils::NO_ROTATION);

        WriteRecording(const std::string &imageFormat, const std::string &depthFormat,
                       const std::string &parameterFormat, const void *parameters,
                       RecordingParametersType parametersType, bool withOpenCV = false,
                       AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);

        WriteRecording(double fps, int width, int height, float fx, float fy, float ppx, float ppy,
                       rs2_distortion model, const float coefficients[5], const std::string &imageWriteFormat = "avi",
                       const std::string &depthWriteFormat = "bin", const std::string &parametersWriteFormat = "xml",
                       bool withOpenCV = false,
                       AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);

        WriteRecording(double fps, rs2_intrinsics intrinsics, const std::string &imageWriteFormat = "avi",
                       const std::string &depthWriteFormat = "bin", const std::string &parametersWriteFormat = "xml",
                       bool withOpenCV = false,
                       AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);

        ~WriteRecording() override;

        void setParameters(const rs2::video_stream_profile *_videoStreamProfile);

        #ifdef OPENCV

        void setParameters(const cv::VideoCapture *_videoCapture);

        #endif

        void setParameters(const RecordingParameters *_recordingParameters);

        #ifdef OPENCV

        bool writeData(cv::Mat *image, rs2::depth_frame *depth, unsigned long long counter = -1);

        bool writeData(cv::Mat *image, cv::Mat *depth, unsigned long long counter = -1);

        bool writeData(cv::Mat &image, cv::Mat &depth, unsigned long long counter = -1);

        #endif

        bool writeData(rs2::video_frame *image, rs2::depth_frame *depth, unsigned long long counter = -1);

        bool writeData(uint8_t *image, int nrImageElements, uint16_t *depth, int nrDepthElements,
                       unsigned long long counter = -1);

        bool writeData(uint8_t *image, int nrImageElements, const double *depth, int nrDepthElements,
                       unsigned long long counter = -1);

    private:
        explicit WriteRecording(bool iWillSetParametersLater, const std::string &imageWriteFormat = "avi",
                                const std::string &depthWriteFormat = "bin",
                                const std::string &parametersWriteFormat = "xml", bool withOpenCV = false,
                                AndreiUtils::RotationType rotationType = AndreiUtils::RotationType::NO_ROTATION);

        static int dataBufferSize;

        void bufferThreadWrite(bool useOpenCV);

        void initializeThreadAndBuffers(bool useOpenCV = false);

        #ifdef OPENCV

        void writeImage(cv::Mat *image);

        void writeDepth(cv::Mat *depth);

        #endif

        void writeImage(uint8_t *imageData);

        void writeDepth(uint16_t *depthData);

        bool initializeImageWriter();

        bool initializeDepthWriter();

        void releaseImageWriter();

        void releaseDepthWriter();

        #ifdef OPENCV
        cv::VideoWriter *imageWriter{};
        #endif
        std::ofstream *imageWriterBinary{}, *depthWriterBinary{};
        bool imageWriterInitialized, depthWriterInitialized;

        std::thread writerThread;
        std::mutex lock;

        AndreiUtils::RotationType writeRotation;

        #ifdef OPENCV
        std::vector<cv::Mat *> imageBuffer{}, depthBuffer{};
        #endif
        std::vector<uint8_t *> imageBytesBuffer;
        std::vector<uint16_t *> depthBytesBuffer;
        std::vector<unsigned long long> countBuffer;
        bool writeFlag, parametersSet;
        int bufferStartIndex, bufferEndIndex, bufferSize;
    };
}

#endif //REALSENSERECORD_WRITERECORDING_H
