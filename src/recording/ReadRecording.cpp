//
// Created by andrei on 25.08.21.
//

#include <RealsenseRecording/recording/ReadRecording.h>
#include <AndreiUtils/utilsImages.h>
#include <iostream>

#ifdef OPENCV
#include <AndreiUtils/utilsOpenCV.h>
#include <AndreiUtils/utilsOpenMP.hpp>
#endif

using namespace AndreiUtils;
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

#ifdef OPENCV
bool ReadRecording::readData(cv::Mat **image, cv::Mat **depth) {
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

bool ReadRecording::readData(cv::Mat *image, cv::Mat *depth) {
    return this->readData(&image, &depth);
}

bool ReadRecording::readData(cv::Mat &image, cv::Mat &depth) {
    return this->readData(&image, &depth);
}
#endif

bool ReadRecording::readData(uint8_t **image, uint16_t **depth) {
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

bool ReadRecording::readData(uint8_t **image, double **depth) {
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

bool ReadRecording::readData(uint8_t *image, uint16_t *depth) {
    return this->readData(&image, &depth);
}

bool ReadRecording::readData(uint8_t *image, double *depth) {
    return this->readData(&image, &depth);
}

bool ReadRecording::readData(uint8_t **image, int imageSize, uint16_t **depth, int depthSize) {
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
        bool imageReadSuccess = this->readImage(image, imageSize);
        if (!imageReadSuccess) {
            if (depth && this->readDepth(depth, depthSize)) {
                cout << "Something is wrong with the serialization... "
                     << "There are no more video frames left but there still are depth frames!" << endl;
            }
            // assert(!depth || !this->readDepth(depth));
            return false;
        }
    }
    if (depth != nullptr) {
        bool depthReadSuccess = this->readDepth(depth, depthSize);
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

bool ReadRecording::readData(uint8_t **image, int imageSize, double **depth, int depthSize) {
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
        bool imageReadSuccess = this->readImage(image, imageSize);
        if (!imageReadSuccess) {
            if (depth && this->readDepth(depth, depthSize)) {
                cout << "Something is wrong with the serialization... "
                     << "There are no more video frames left but there still are depth frames!" << endl;
            }
            // assert(!depth || !this->readDepth(depth));
            return false;
        }
    }
    if (depth != nullptr) {
        bool depthReadSuccess = this->readDepth(depth, depthSize);
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

bool ReadRecording::readData(uint8_t *image, int imageSize, uint16_t *depth, int depthSize) {
    return this->readData(&image, imageSize, &depth, depthSize);
}

bool ReadRecording::readData(uint8_t *image, int imageSize, double *depth, int depthSize) {
    return this->readData(&image, imageSize, &depth, depthSize);
}

#ifdef OPENCV
bool ReadRecording::readImage(cv::Mat **image) {
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
#endif

bool ReadRecording::readImage(uint8_t **image) {
    if (this->parameters.imageFormat == "avi") {
        #ifdef OPENCV
        cv::Mat mat;
        *this->imageReader >> mat;
        if (mat.empty()) {
            return false;
        }
        size_t imageSize = matByteSize(mat);
        delete[] *image;
        *image = new uint8_t[imageSize];
        fastMemCopy(*image, mat.data, imageSize);
        return true;
        #else
        throw runtime_error("Can not read image in avi format when opencv is not enabled");
        #endif
    } else if (this->parameters.imageFormat == "bin") {
        StandardTypes imageType;
        bool readSuccess = readColorImageBinary(this->imageReaderBinary, *image, this->parameters.height,
                                                this->parameters.width, imageType);
        assert (imageType == AndreiUtils::StandardTypes::TYPE_UINT_8);
        return readSuccess;
    }
    throw runtime_error("Unknown image format: \"" + this->parameters.imageFormat + "\"");
}

bool ReadRecording::readDepth(uint16_t **depth) {
    if (this->parameters.depthFormat == "bin") {
        bool readSuccess = readDepthImageBinary(this->depthReaderBinary, *depth, this->parameters.height,
                                                this->parameters.width);
        if (!readSuccess) {
            return false;
        }
        return true;
    }
    throw runtime_error("Unknown depth format: \"" + this->parameters.depthFormat + "\"");
}

bool ReadRecording::readDepth(double **depth) {
    if (this->parameters.depthFormat == "bin") {
        bool readSuccess = readDepthImageBinary(this->depthReaderBinary, *depth, this->parameters.height,
                                                this->parameters.width);
        if (!readSuccess) {
            return false;
        }
        return true;
    }
    throw runtime_error("Unknown depth format: \"" + this->parameters.depthFormat + "\"");
}

bool ReadRecording::readImage(uint8_t **image, int imageSize) {
    if (this->parameters.imageFormat == "avi") {
        #ifdef OPENCV
        cv::Mat mat;
        *this->imageReader >> mat;
        if (mat.empty()) {
            return false;
        }
        assert (matByteSize(mat) == (size_t) imageSize);
        fastMemCopy(*image, mat.data, imageSize);
        return true;
        #else
        throw runtime_error("Can not read image in avi format when opencv is not enabled");
        #endif
    } else if (this->parameters.imageFormat == "bin") {
        StandardTypes imageType;
        bool readSuccess = readColorImageBinary(this->imageReaderBinary, *image, this->parameters.height,
                                                this->parameters.width, imageType, imageSize);
        assert (imageType == AndreiUtils::StandardTypes::TYPE_UINT_8);
        return readSuccess;
    }
    throw runtime_error("Unknown image format: \"" + this->parameters.imageFormat + "\"");
}

bool ReadRecording::readDepth(uint16_t **depth, int depthSize) {
    if (this->parameters.depthFormat == "bin") {
        bool readSuccess = readDepthImageBinary(this->depthReaderBinary, *depth, this->parameters.height,
                                                this->parameters.width, depthSize);
        if (!readSuccess) {
            return false;
        }
        return true;
    }
    throw runtime_error("Unknown depth format: \"" + this->parameters.depthFormat + "\"");
}

bool ReadRecording::readDepth(double **depth, int depthSize) {
    if (this->parameters.depthFormat == "bin") {
        bool readSuccess = readDepthImageBinary(this->depthReaderBinary, *depth, this->parameters.height,
                                                this->parameters.width, depthSize);
        if (!readSuccess) {
            return false;
        }
        return true;
    }
    throw runtime_error("Unknown depth format: \"" + this->parameters.depthFormat + "\"");
}

void ReadRecording::initializeImageReader() {
    if (this->parameters.imageFormat == "avi") {
        #ifdef OPENCV
        this->imageReader = new cv::VideoCapture(this->imageFile);
        return;
        #else
        throw runtime_error("Can not initialize the image reader in \"avi\" format without opencv enabled");
        #endif
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
        #ifdef OPENCV
        this->imageReader->release();
        delete this->imageReader;
        return;
        #endif
    } else if (this->parameters.imageFormat == "bin") {
        if (this->imageReaderBinary != nullptr) {
            this->imageReaderBinary->close();
        }
        delete this->imageReaderBinary;
        return;
    }
    throw runtime_error("Unknown image format: \"" + this->parameters.imageFormat + "\"");
}

void ReadRecording::releaseDepthReader() {
    if (this->parameters.depthFormat.empty()) {
        return;
    } else if (this->parameters.depthFormat == "bin") {
        if (this->depthReaderBinary != nullptr) {
            this->depthReaderBinary->close();
        }
        delete this->depthReaderBinary;
        return;
    }
    throw runtime_error("Unknown depth format: \"" + this->parameters.depthFormat + "\"");
}
