//
// Created by andrei on 25.08.21.
//

#include <recording/ReadRecording.h>
#include <andrei_utils/utilsOpenCV.h>

using namespace cv;
using namespace std;

ReadRecording::ReadRecording(int fileNumber) : Recording(), imageReaderInitialized(false),
                                               depthReaderInitialized(false) {
    this->setFiles(true, fileNumber);
}

ReadRecording::~ReadRecording() {
    this->releaseImageReader();
    this->releaseDepthReader();
}

bool ReadRecording::readData(Mat **image, Mat **depth) {
    if ((image != nullptr && !this->imageReaderInitialized) || (depth != nullptr && !this->depthReaderInitialized)) {
        if (image != nullptr) {
            this->initializeImageReader();
            this->imageReaderInitialized = true;
        }

        if (depth != nullptr) {
            this->initializeDepthReader();
            this->depthReaderInitialized = true;
        }
    }

    if (image != nullptr) {
        bool imageReadSuccess = this->readImage(image);
        if (!imageReadSuccess) {
            assert(!depth || !this->readDepth(depth));
            return false;
        }
    }
    if (depth != nullptr) {
        bool depthReadSuccess = this->readDepth(depth);
        assert(depthReadSuccess);
        if (!image || !depthReadSuccess) {
            return false;
        }
    }

    return true;
}

bool ReadRecording::readData(Mat *image, Mat *depth) {
    return this->readData(&image, &depth);
}

bool ReadRecording::readData(Mat &image, Mat &depth) {
    return this->readData(&image, &depth);
}

bool ReadRecording::readImage(Mat **image) {
    if (this->parameters.imageFormat == "avi") {
        *this->imageReader >> **image;
        return !(**image).empty();
    } else if (this->parameters.imageFormat == "bin") {
        return matReadBinary(this->imageReaderBinary, *image);
    }
    throw runtime_error("Unknown image format: \"" + this->parameters.imageFormat + "\"");
}

bool ReadRecording::readDepth(cv::Mat **depth) {
    if (this->parameters.depthFormat == "bin") {
        return matReadBinary(this->depthReaderBinary, *depth);
    }
    throw runtime_error("Unknown depth format: \"" + this->parameters.depthFormat + "\"");
}

void ReadRecording::initializeImageReader() {
    if (this->parameters.imageFormat == "avi") {
        this->imageReader = new VideoCapture(this->imageFile);
        return;
    } else if (this->parameters.imageFormat == "bin") {
        this->imageReaderBinary = new ifstream(this->imageFile, fstream::binary);
        return;
    }
    throw runtime_error("Unknown image format: \"" + this->parameters.imageFormat + "\"");
}

void ReadRecording::initializeDepthReader() {
    if (this->parameters.depthFormat == "bin") {
        this->depthReaderBinary = new ifstream(this->depthFile, fstream::binary);
        return;
    }
    throw runtime_error("Unknown depth format: \"" + this->parameters.depthFormat + "\"");
}

void ReadRecording::releaseImageReader() {
    if (this->parameters.imageFormat.empty()) {
        return;
    } else if (this->parameters.imageFormat == "avi") {
        this->imageReader->release();
        delete this->imageReader;
        return;
    } else if (this->parameters.imageFormat == "bin") {
        this->imageReaderBinary->close();
        delete this->imageReaderBinary;
        return;
    }
    throw runtime_error("Unknown image format: \"" + this->parameters.imageFormat + "\"");
}

void ReadRecording::releaseDepthReader() {
    if (this->parameters.depthFormat.empty()) {
        return;
    } else if (this->parameters.depthFormat == "bin") {
        this->depthReaderBinary->close();
        delete this->depthReaderBinary;
        return;
    }
    throw runtime_error("Unknown depth format: \"" + this->parameters.depthFormat + "\"");
}
