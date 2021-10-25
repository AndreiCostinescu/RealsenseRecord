//
// Created by andrei on 25.08.21.
//

#include <RealsenseRecording/recording/WriteRecording.h>
#include <AndreiUtils/utilsImages.h>
#include <AndreiUtils/utilsJson.h>
#include <AndreiUtils/utilsOpenMP.hpp>
#include <AndreiUtils/utilsRealsense.h>
#include <configDirectoryLocation.h>
#include <iostream>

#ifdef OPENCV
#include <AndreiUtils/utilsOpenCV.h>
#include <AndreiUtils/utilsOpenCVRealsense.h>
#endif

using namespace AndreiUtils;
using namespace RealsenseRecording;
using namespace rs2;
using namespace std;

int WriteRecording::dataBufferSize = 0;

WriteRecording *WriteRecording::createEmptyPtr(const string &imageWriteFormat, const string &depthWriteFormat,
                                               const string &parametersWriteFormat,
                                               AndreiUtils::RotationType rotationType) {
    return new WriteRecording(true, imageWriteFormat, depthWriteFormat, parametersWriteFormat, rotationType);
}

WriteRecording::WriteRecording(const std::string &imageFormat, const std::string &depthFormat,
                               const std::string &parameterFormat, const void *parameters,
                               RecordingParametersType parametersType, AndreiUtils::RotationType rotationType) :
        Recording(imageFormat, depthFormat, parameterFormat, parameters, parametersType, rotationType),
        imageBytesBuffer(), depthBytesBuffer(), countBuffer(), writeFlag(true), bufferStartIndex(0), bufferEndIndex(0),
        bufferSize(0), writeRotation(rotationType), imageWriterInitialized(false), depthWriterInitialized(false),
        parametersSet(true) {
    this->initializeThreadAndBuffers();
}

WriteRecording::WriteRecording(double fps, int width, int height, float fx, float fy, float ppx, float ppy,
                               rs2_distortion model, const float *coefficients, const string &imageWriteFormat,
                               const string &depthWriteFormat, const string &parametersWriteFormat,
                               RotationType rotationType) :
        Recording(fps, width, height, fx, fy, ppx, ppy, model, coefficients, imageWriteFormat, depthWriteFormat,
                  parametersWriteFormat, rotationType), imageBytesBuffer(), depthBytesBuffer(), countBuffer(),
        writeFlag(true), bufferStartIndex(0), bufferEndIndex(0), bufferSize(0), writeRotation(rotationType),
        imageWriterInitialized(false), depthWriterInitialized(false), parametersSet(true) {
    this->initializeThreadAndBuffers();
}

WriteRecording::WriteRecording(double fps, rs2_intrinsics intrinsics, const string &imageWriteFormat,
                               const string &depthWriteFormat, const string &parametersWriteFormat,
                               RotationType rotationType) :
        Recording(fps, intrinsics, imageWriteFormat, depthWriteFormat, parametersWriteFormat, rotationType),
        imageBytesBuffer(), depthBytesBuffer(), countBuffer(), writeFlag(true), bufferStartIndex(0), bufferEndIndex(0),
        bufferSize(0), writeRotation(rotationType), imageWriterInitialized(false), depthWriterInitialized(false),
        parametersSet(true) {
    this->initializeThreadAndBuffers();
}

WriteRecording::~WriteRecording() {
    cout << "Entering WriteRecording destructor!" << endl;
    this->writeFlag = false;
    cout << "Wait until all remaining frames have been written!" << endl;
    this->writerThread.join();
    cout << "Finished writing!" << endl;
    this->parametersSet = false;

    this->releaseImageWriter();
    this->releaseDepthWriter();
}

void WriteRecording::setParameters(const rs2::video_stream_profile *_videoStreamProfile) {
    this->parameters.setParameters(_videoStreamProfile, this->writeRotation);
    this->parametersSet = true;
}

#ifdef OPENCV
void WriteRecording::setParameters(const cv::VideoCapture *_videoCapture) {
    this->parameters.setParameters(_videoCapture, this->writeRotation);
    this->parametersSet = true;
}
#endif

void WriteRecording::setParameters(const RecordingParameters *_recordingParameters) {
    this->parameters.setParameters(_recordingParameters, this->writeRotation);
    this->parametersSet = true;
}

#ifdef OPENCV
bool WriteRecording::writeData(cv::Mat *image, rs2::depth_frame *depth, unsigned long long counter) {
    if (depth == nullptr) {
        return this->writeData(image, (cv::Mat *) nullptr, counter);
    }
    cv::Mat depthMat = depth_frame_to_meters(*depth);
    return this->writeData(image, &depthMat, counter);
}

bool WriteRecording::writeData(cv::Mat *image, cv::Mat *depth, unsigned long long counter) {
    if ((image != nullptr && !this->imageWriterInitialized) || (depth != nullptr && !this->depthWriterInitialized)) {
        if (!this->imageWriterInitialized && !this->depthWriterInitialized) {
            if (!this->parametersSet) {
                throw runtime_error("Parameters have not been set but tried to write data!");
            }
            // Write parameter data
            this->parameters.serialize(this->parameterFile);
            cout << "Wrote outputRecording data to file!" << endl;
        }

        if (image != nullptr) {
            if (!this->initializeImageWriter()) {
                return false;
            }
            this->imageWriterInitialized = true;
        }

        if (depth != nullptr) {
            if (!this->initializeDepthWriter()) {
                return false;
            }
            this->depthWriterInitialized = true;
        }
    }

    while (this->bufferSize == WriteRecording::dataBufferSize) {
        // this_thread::yield();  // ???
    }

    this->countBuffer[this->bufferEndIndex] = counter;

    if (this->imageBuffer[this->bufferEndIndex] != nullptr) {
        delete this->imageBuffer[this->bufferEndIndex];
        this->imageBuffer[this->bufferEndIndex] = nullptr;
    }
    if (image != nullptr) {
        this->imageBuffer[this->bufferEndIndex] = new cv::Mat(*image);
    }

    if (this->depthBuffer[this->bufferEndIndex] != nullptr) {
        delete this->depthBuffer[this->bufferEndIndex];
        this->depthBuffer[this->bufferEndIndex] = nullptr;
    }
    if (depth != nullptr) {
        this->depthBuffer[this->bufferEndIndex] = new cv::Mat(*depth);
    }

    this->bufferEndIndex = (this->bufferEndIndex + 1) % WriteRecording::dataBufferSize;

    this->lock.lock();
    this->bufferSize++;
    this->lock.unlock();

    return true;
}

bool WriteRecording::writeData(cv::Mat &image, cv::Mat &depth, unsigned long long counter) {
    return this->writeData(&image, &depth, counter);
}
#endif

bool WriteRecording::writeData(rs2::video_frame *image, rs2::depth_frame *depth, unsigned long long counter) {
    int imageDataType;
    int nrImageElements = image->get_height() * image->get_width() * image->get_bytes_per_pixel();
    auto *imageData = new uint8_t[nrImageElements];
    frameToBytes(*image, imageData, imageDataType, nrImageElements);
    assert (imageDataType == StandardTypes::TYPE_UINT_8);
    int nrDepthElements = depth->get_height() * depth->get_width();
    auto *depthData = new double[nrDepthElements];
    depthFrameToMeters(*depth, depthData, nrDepthElements);
    return this->writeData(imageData, nrImageElements, depthData, nrDepthElements, counter);
}

bool WriteRecording::writeData(uint8_t *image, int nrImageElements, uint16_t *depth, int nrDepthElements,
                               unsigned long long counter) {
    if ((image != nullptr && !this->imageWriterInitialized) || (depth != nullptr && !this->depthWriterInitialized)) {
        if (!this->imageWriterInitialized && !this->depthWriterInitialized) {
            if (!this->parametersSet) {
                throw runtime_error("Parameters have not been set but tried to write data!");
            }
            // Write parameter data
            this->parameters.serialize(this->parameterFile);
            cout << "Wrote outputRecording data to file!" << endl;
        }

        if (image != nullptr) {
            if (!this->initializeImageWriter()) {
                return false;
            }
            this->imageWriterInitialized = true;
        }

        if (depth != nullptr) {
            if (!this->initializeDepthWriter()) {
                return false;
            }
            this->depthWriterInitialized = true;
        }
    }

    while (this->bufferSize == WriteRecording::dataBufferSize) {
        // this_thread::yield();  // ???
    }

    this->countBuffer[this->bufferEndIndex] = counter;

    if (this->imageBytesBuffer[this->bufferEndIndex] != nullptr) {
        delete[] this->imageBytesBuffer[this->bufferEndIndex];
        this->imageBytesBuffer[this->bufferEndIndex] = nullptr;
    }
    if (image != nullptr) {
        this->imageBytesBuffer[this->bufferEndIndex] = new uint8_t[nrImageElements];
        fastMemCopy(this->imageBytesBuffer[this->bufferEndIndex], image, nrImageElements);
    }

    if (this->depthBytesBuffer[this->bufferEndIndex] != nullptr) {
        delete[] this->depthBytesBuffer[this->bufferEndIndex];
        this->depthBytesBuffer[this->bufferEndIndex] = nullptr;
    }
    if (depth != nullptr) {
        this->depthBytesBuffer[this->bufferEndIndex] = new uint16_t[nrDepthElements];
        fastMemCopy(this->depthBytesBuffer[this->bufferEndIndex], depth, nrDepthElements);
    }

    this->bufferEndIndex = (this->bufferEndIndex + 1) % WriteRecording::dataBufferSize;

    this->lock.lock();
    this->bufferSize++;
    this->lock.unlock();

    return true;
}

bool WriteRecording::writeData(uint8_t *image, int nrImageElements, double *depth, int nrDepthElements,
                               unsigned long long counter) {
    if ((image != nullptr && !this->imageWriterInitialized) || (depth != nullptr && !this->depthWriterInitialized)) {
        if (!this->imageWriterInitialized && !this->depthWriterInitialized) {
            if (!this->parametersSet) {
                throw runtime_error("Parameters have not been set but tried to write data!");
            }
            // Write parameter data
            this->parameters.serialize(this->parameterFile);
            cout << "Wrote outputRecording data to file!" << endl;
        }

        if (image != nullptr) {
            if (!this->initializeImageWriter()) {
                return false;
            }
            this->imageWriterInitialized = true;
        }

        if (depth != nullptr) {
            if (!this->initializeDepthWriter()) {
                return false;
            }
            this->depthWriterInitialized = true;
        }
    }

    while (this->bufferSize == WriteRecording::dataBufferSize) {
        // this_thread::yield();  // ???
    }

    this->countBuffer[this->bufferEndIndex] = counter;

    if (this->imageBytesBuffer[this->bufferEndIndex] != nullptr) {
        delete[] this->imageBytesBuffer[this->bufferEndIndex];
        this->imageBytesBuffer[this->bufferEndIndex] = nullptr;
    }
    if (image != nullptr) {
        this->imageBytesBuffer[this->bufferEndIndex] = new uint8_t[nrImageElements];
        fastMemCopy(this->imageBytesBuffer[this->bufferEndIndex], image, nrImageElements);
    }

    if (this->depthBytesBuffer[this->bufferEndIndex] != nullptr) {
        delete[] this->depthBytesBuffer[this->bufferEndIndex];
        this->depthBytesBuffer[this->bufferEndIndex] = nullptr;
    }
    if (depth != nullptr) {
        this->depthBytesBuffer[this->bufferEndIndex] = new uint16_t[nrDepthElements];
        #pragma omp parallel for shared(depth, nrDepthElements) default(none)
        for (size_t i = 0; i < nrDepthElements; i++) {
            this->depthBytesBuffer[this->bufferEndIndex][i] = (uint16_t) (depth[i] * 1000);
        }
    }

    this->bufferEndIndex = (this->bufferEndIndex + 1) % WriteRecording::dataBufferSize;

    this->lock.lock();
    this->bufferSize++;
    this->lock.unlock();

    return true;
}

WriteRecording::WriteRecording(bool iWillSetParametersLater, const std::string &imageWriteFormat,
                               const std::string &depthWriteFormat, const std::string &parametersWriteFormat,
                               AndreiUtils::RotationType rotationType) :
        Recording(imageWriteFormat, depthWriteFormat, parametersWriteFormat, rotationType), imageBytesBuffer(),
        depthBytesBuffer(), countBuffer(), writeFlag(true), bufferStartIndex(0), bufferEndIndex(0), bufferSize(0),
        writeRotation(rotationType), imageWriterInitialized(false), depthWriterInitialized(false),
        parametersSet(false) {
    if (!iWillSetParametersLater) {
        throw runtime_error("When creating an empty WriteRecording, you must agree to set the parameters later!");
    }
    this->initializeThreadAndBuffers();
}

void WriteRecording::bufferThreadWrite(bool useOpenCV) {
    while (this->writeFlag || this->bufferSize > 0) {
        while (this->writeFlag && this->bufferSize == 0) {
            this_thread::yield();
        }
        if (this->bufferSize == 0) {
            break;
        }

        if (useOpenCV) {
            #ifdef OPENCV
            cv::Mat *data;
            if (this->imageBuffer[this->bufferStartIndex] != nullptr) {
                data = this->imageBuffer[this->bufferStartIndex];
                this->writeImage(data);
                delete data;
                this->imageBuffer[this->bufferStartIndex] = nullptr;
            }
            if (this->depthBuffer[this->bufferStartIndex] != nullptr) {
                data = this->depthBuffer[this->bufferStartIndex];
                this->writeDepth(data);
                delete data;
                this->depthBuffer[this->bufferStartIndex] = nullptr;
            }
            #else
            cout << "Can use opencv when writing images when opencv is not enabled!" << endl;
            #endif
        } else {
            if (this->imageBytesBuffer[this->bufferStartIndex] != nullptr) {
                this->writeImage(this->imageBytesBuffer[this->bufferStartIndex]);
                delete[] this->imageBytesBuffer[this->bufferStartIndex];
                this->imageBytesBuffer[this->bufferStartIndex] = nullptr;
            }
            if (this->depthBytesBuffer[this->bufferStartIndex] != nullptr) {
                this->writeDepth(this->depthBytesBuffer[this->bufferStartIndex]);
                delete[] this->depthBytesBuffer[this->bufferStartIndex];
                this->depthBytesBuffer[this->bufferStartIndex] = nullptr;
            }
        }

        this->bufferStartIndex = (this->bufferStartIndex + 1) % WriteRecording::dataBufferSize;

        this->lock.lock();
        this->bufferSize--;
        this->lock.unlock();
    }
}

void WriteRecording::initializeThreadAndBuffers(bool withOpenCV) {
    this->writerThread = thread(&WriteRecording::bufferThreadWrite, this, withOpenCV);
    if (WriteRecording::dataBufferSize == 0) {
        if (RealsenseRecording::configDirectoryLocation.empty()) {
            throw runtime_error("RealsenseRecording: configDirectoryLocation is not set...");
        }
        auto config = readJsonFile(RealsenseRecording::configDirectoryLocation + "recordingOutputDirectory.cfg");
        WriteRecording::dataBufferSize = config["writeBufferSize"].get<int>();
        if (WriteRecording::dataBufferSize< 1) {
            throw runtime_error(
                    "Can not work with a buffer size < 1! Was " + to_string(WriteRecording::dataBufferSize));
        }
    }

    #ifdef OPENCV
    this->imageBuffer.resize(WriteRecording::dataBufferSize);
    this->depthBuffer.resize(WriteRecording::dataBufferSize);
    #endif
    this->imageBytesBuffer.resize(WriteRecording::dataBufferSize);
    this->depthBytesBuffer.resize(WriteRecording::dataBufferSize);
    this->countBuffer.resize(WriteRecording::dataBufferSize);
}

#ifdef OPENCV
void WriteRecording::writeImage(cv::Mat *image) {
    imageRotation(image, this->writeRotation);
    if (this->parameters.imageFormat == "avi") {
        this->imageWriter->write(*image);
        return;
    } else if (this->parameters.imageFormat == "bin") {
        matWriteBinary(this->imageWriterBinary, *image);
        return;
    }
    throw runtime_error("Unknown image format: \"" + this->parameters.imageFormat + "\"");
}

void WriteRecording::writeDepth(cv::Mat *depth) {
    imageRotation(depth, this->writeRotation);
    if (this->parameters.depthFormat == "bin") {
        cv::Mat convertedData;
        convertDepthToMillimetersUInt16(depth, convertedData);
        matWriteBinary(this->depthWriterBinary, convertedData);
        return;
    }
    throw runtime_error("Unknown depth format: \"" + this->parameters.depthFormat + "\"");
}
#endif

void WriteRecording::writeImage(uint8_t *imageData) {
    imageDataRotation(imageData, this->writeRotation, AndreiUtils::TYPE_UINT_8, this->parameters.height,
                      this->parameters.width, 3);
    if (this->parameters.imageFormat == "bin") {
        writeColorImageBinary(this->imageWriterBinary, imageData, this->parameters.height, this->parameters.width,
                              AndreiUtils::TYPE_UINT_8);
        return;
    }
    throw runtime_error("Unknown image format: \"" + this->parameters.imageFormat + "\"");
}

void WriteRecording::writeDepth(uint16_t *depthData) {
    imageDataRotation((uint8_t *) depthData, this->writeRotation, AndreiUtils::TYPE_UINT_16, this->parameters.height,
                      this->parameters.width, 1);
    if (this->parameters.depthFormat == "bin") {
        writeDepthImageBinary(this->depthWriterBinary, depthData, this->parameters.height, this->parameters.width);
        return;
    }
    throw runtime_error("Unknown depth format: \"" + this->parameters.depthFormat + "\"");
}

bool WriteRecording::initializeImageWriter() {
    if (this->parameters.imageFormat == "avi") {
        #ifdef OPENCV
        auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
        auto size = cv::Size(this->parameters.width, this->parameters.height);

        this->imageWriter = new cv::VideoWriter(this->imageFile, fourcc, this->parameters.fps, size, true);
        if (!this->imageWriter->isOpened()) {
            delete this->imageWriter;
            this->imageWriter = nullptr;
            cerr << "Cannot save video to file" << endl;
            return false;
        }
        return true;
        #else
        throw runtime_error("Can not initialize image writer in avi format when opencv is not enabled");
        #endif
    } else if (this->parameters.imageFormat == "bin") {
        this->imageWriterBinary = new ofstream(this->imageFile, fstream::binary);
        return true;
    }
    throw runtime_error("Unknown image format: \"" + this->parameters.imageFormat + "\"");
}

bool WriteRecording::initializeDepthWriter() {
    if (this->parameters.depthFormat == "bin") {
        this->depthWriterBinary = new ofstream(this->depthFile, fstream::binary);
        return true;
    }
    throw runtime_error("Unknown depth format: \"" + this->parameters.depthFormat + "\"");
}

void WriteRecording::releaseImageWriter() {
    if (this->parameters.imageFormat.empty()) {
        return;
    } else if (this->parameters.imageFormat == "avi") {
        #ifdef OPENCV
        this->imageWriter->release();
        delete this->imageWriter;
        #else
        cout << "Warning: releasing image writer in imageFormat avi when OPENCV is not enabled" << endl;
        #endif
        return;
    } else if (this->parameters.imageFormat == "bin") {
        if (this->imageWriterBinary != nullptr) {
            this->imageWriterBinary->close();
        }
        delete this->imageWriterBinary;
        return;
    }
    throw runtime_error("Unknown image format: \"" + this->parameters.imageFormat + "\"");
}

void WriteRecording::releaseDepthWriter() {
    if (this->parameters.depthFormat.empty()) {
        return;
    } else if (this->parameters.depthFormat == "bin") {
        if (this->depthWriterBinary != nullptr) {
            this->depthWriterBinary->close();
        }
        delete this->depthWriterBinary;
        return;
    }
    throw runtime_error("Unknown depth format: \"" + this->parameters.depthFormat + "\"");
}
