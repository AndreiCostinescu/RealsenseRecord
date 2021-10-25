//
// Created by andrei on 25.08.21.
//

#ifndef REALSENSERECORD_REALSENSECAPTURE_H
#define REALSENSERECORD_REALSENSECAPTURE_H

#include <AndreiUtils/classes/Timer.hpp>
#include <RealsenseRecording/recording/ReadRecording.h>
#include <RealsenseRecording/recording/WriteRecording.h>
#include <string>

namespace RealsenseRecording {
    class RealsenseCapture {
    public:
        explicit RealsenseCapture(int fps, bool withRecord = false, int recordedFileNumber = -1,
                                  const std::string &recordedBagFile = "", int colorWidth = 1280, int colorHeight = 720,
                                  int depthWidth = 640, int depthHeight = 480,
                                  const std::string &recordImageFormat = "avi",
                                  const std::string &recordDepthFormat = "bin",
                                  const std::string &recordParametersFormat = "xml", bool withOpenCV = false,
                                  bool writeFPSOnImage = false);

        ~RealsenseCapture();

        void run();

        #ifdef OPENCV
        cv::Mat &getImage();

        cv::Mat getImage() const;

        void setImage(const cv::Mat &_image);

        cv::Mat &getDepth();

        cv::Mat getDepth() const;

        void setDepth(const cv::Mat &_depth);
        #endif

        rs2::frame &getImageFrame();

        rs2::video_frame getImageFrame() const;

        rs2::frame &getDepthFrame();

        rs2::depth_frame getDepthFrame() const;

        rs2_intrinsics &getDepthIntrinsics();

        rs2_intrinsics getDepthIntrinsics() const;

        void setDepthIntrinsics(const rs2_intrinsics &_depthIntrinsics);

        bool saveData();

    private:
        bool updateFrame();

        void computeAndDisplayFps();

        int IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_FPS, DEPTH_WIDTH, DEPTH_HEIGHT, DEPTH_FPS;

        rs2::pipeline pipeline;
        rs2::config startConfig;
        rs2::frameset frames;

        rs2::frame imageFrame, depthFrame;

        #ifdef OPENCV
        cv::Mat image{}, depth{};
        #endif
        uint8_t *imageData{};
        double *depthData{};
        rs2_intrinsics depthIntrinsics;

        ReadRecording *inputRecording;
        WriteRecording *outputRecording;

        #ifdef OPENCV
        const cv::Scalar SCALAR_BLACK = cv::Scalar(0.0, 0.0, 0.0);
        const cv::Scalar SCALAR_WHITE = cv::Scalar(255.0, 255.0, 255.0);
        const cv::Scalar SCALAR_BLUE = cv::Scalar(255.0, 0.0, 0.0);
        const cv::Scalar SCALAR_GREEN = cv::Scalar(0.0, 200.0, 0.0);
        const cv::Scalar SCALAR_RED = cv::Scalar(0.0, 0.0, 255.0);
        #endif

        AndreiUtils::Timer fpsTimer;
        int fps = 0, sleepTime = 0;
        bool writeFPSOnImage, withOpenCV;
    };
}

#endif //REALSENSERECORD_REALSENSECAPTURE_H
