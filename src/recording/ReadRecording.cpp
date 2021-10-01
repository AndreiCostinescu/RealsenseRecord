//
// Created by andrei on 25.08.21.
//

#include <RealsenseRecording/recording/ReadRecording.h>
#include <AndreiUtils/utilsOpenCV.h>

using namespace AndreiUtils;
using namespace cv;
using namespace RealsenseRecording;
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
            if (depth && this->readDepth(depth)) {
                cout << "Something is wrong with the serialization... "
                     << "There are no more video frames left but there still are depth frames!" << endl;
            }
            // assert(!depth || !this->readDepth(depth));
            return false;
        }
    }
    if (depth != nullptr) {
        bool depthReadSuccess = this->readDepth(depth);
        if (image && !depthReadSuccess) {
            cout << "Something is wrong with the serialization... "
                 << "There are no more depth frames left but there still are color frames!" << endl;
        }
        // assert(depthReadSuccess);
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
        bool readSuccess = matReadBinary(this->depthReaderBinary, *depth);
        if (!readSuccess) {
            return false;
        }
        if ((**depth).type() == CV_16U) {
            convertDepthToMetersDouble64(*depth);
        }
        return true;
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
