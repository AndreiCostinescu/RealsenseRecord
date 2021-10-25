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

        #ifdef OPENCV
        bool readData(cv::Mat **image, cv::Mat **depth);

        bool readData(cv::Mat *image, cv::Mat *depth);

        bool readData(cv::Mat &image, cv::Mat &depth);
        #endif

        bool readData(uint8_t **image, uint16_t **depth);

        bool readData(uint8_t **image, double **depth);

        bool readData(uint8_t *image, uint16_t *depth);

        bool readData(uint8_t *image, double *depth);

        bool readData(uint8_t **image, int imageSize, uint16_t **depth, int depthSize);

        bool readData(uint8_t **image, int imageSize, double **depth, int depthSize);

        bool readData(uint8_t *image, int imageSize, uint16_t *depth, int depthSize);

        bool readData(uint8_t *image, int imageSize, double *depth, int depthSize);

    private:
        #ifdef OPENCV
        bool readImage(cv::Mat **image);

        bool readDepth(cv::Mat **depth);
        #endif

        bool readImage(uint8_t **image);

        bool readDepth(uint16_t **depth);

        bool readDepth(double **depth);

        bool readImage(uint8_t **image, int imageSize);

        bool readDepth(uint16_t **depth, int depthSize);

        bool readDepth(double **depth, int depthSize);

        void initializeImageReader();

        void initializeDepthReader();

        void releaseImageReader();

        void releaseDepthReader();

        #ifdef OPENCV
        cv::VideoCapture *imageReader{};
        #endif
        std::ifstream *imageReaderBinary{}, *depthReaderBinary{};
        bool imageReaderInitialized, depthReaderInitialized;
    };
}

#endif //REALSENSERECORD_READRECORDING_H
