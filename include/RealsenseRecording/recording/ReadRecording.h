//
// Created by andrei on 25.08.21.
//

#ifndef REALSENSERECORD_READRECORDING_H
#define REALSENSERECORD_READRECORDING_H

#include <RealsenseRecording/recording/Recording.h>

namespace RealsenseRecording {
    class ReadRecording : public Recording {
    public:
        explicit ReadRecording(int fileNumber);

        ~ReadRecording() override;

        bool readData(cv::Mat **image, cv::Mat **depth);

        bool readData(cv::Mat *image, cv::Mat *depth);

        bool readData(cv::Mat &image, cv::Mat &depth);

    private:
        bool readImage(cv::Mat **image);

        bool readDepth(cv::Mat **depth);

        void initializeImageReader();

        void initializeDepthReader();

        void releaseImageReader();

        void releaseDepthReader();

        cv::VideoCapture *imageReader{};
        std::ifstream *imageReaderBinary{}, *depthReaderBinary{};
        bool imageReaderInitialized, depthReaderInitialized;
    };
}

#endif //REALSENSERECORD_READRECORDING_H
