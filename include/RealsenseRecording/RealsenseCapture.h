//
// Created by andrei on 25.08.21.
//

#ifndef REALSENSERECORD_REALSENSECAPTURE_H
#define REALSENSERECORD_REALSENSECAPTURE_H

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
                                  const std::string &recordParametersFormat = "xml", bool writeFPSOnImage = false);

        ~RealsenseCapture();

        void run();

        cv::Mat &getImage();

        cv::Mat getImage() const;

        void setImage(const cv::Mat &_image);

        cv::Mat &getDepth();

        cv::Mat getDepth() const;

        void setDepth(const cv::Mat &_depth);

        rs2_intrinsics &getDepthIntrinsics();

        rs2_intrinsics getDepthIntrinsics() const;

        void setDepthIntrinsics(const rs2_intrinsics &_depthIntrinsics);

        bool saveData();

    private:
        bool updateFrame();

        void computeAndDisplayFps();

        const int IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_FPS, DEPTH_WIDTH, DEPTH_HEIGHT, DEPTH_FPS;

        rs2::pipeline pipeline;
        rs2::config startConfig;
        rs2::frameset frames;

        rs2::frame imageFrame, depthFrame;

        cv::Mat image, depth;
        rs2_intrinsics depthIntrinsics;

        ReadRecording *inputRecording;
        WriteRecording *outputRecording;

        const cv::Scalar SCALAR_BLACK = cv::Scalar(0.0, 0.0, 0.0);
        const cv::Scalar SCALAR_WHITE = cv::Scalar(255.0, 255.0, 255.0);
        const cv::Scalar SCALAR_BLUE = cv::Scalar(255.0, 0.0, 0.0);
        const cv::Scalar SCALAR_GREEN = cv::Scalar(0.0, 200.0, 0.0);
        const cv::Scalar SCALAR_RED = cv::Scalar(0.0, 0.0, 255.0);

        std::chrono::system_clock::time_point start, end;
        int fps = 0, sleepTime = 0;
        bool writeFPSOnImage;
    };
}

#endif //REALSENSERECORD_REALSENSECAPTURE_H
