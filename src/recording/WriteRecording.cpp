//
// Created by andrei on 25.08.21.
//

#include <recording/WriteRecording.h>
#include <AndreiUtils/utilsJson.h>
#include <AndreiUtils/utilsOpenCV.h>
#include <AndreiUtils/utilsRealsense.h>
#include <utils.h>

using namespace AndreiUtils;
using namespace cv;
using namespace rs2;
using namespace std;

int WriteRecording::dataBufferSize = 0;

WriteRecording::WriteRecording(const string &imageWriteFormat, const string &depthWriteFormat,
                               const string &parametersWriteFormat, const RecordingParameters *recordingParameters,
                               rs2::video_stream_profile *videoStreamProfile, VideoCapture *videoCapture,
                               RotationType rotationType) :
        Recording(imageWriteFormat, depthWriteFormat, parametersWriteFormat, recordingParameters, videoStreamProfile,
                  videoCapture, rotationType), imageBuffer(), depthBuffer(), countBuffer(), writeFlag(true),
        bufferStartIndex(0), bufferEndIndex(0), bufferSize(0), writeRotation(rotationType),
        imageWriterInitialized(false), depthWriterInitialized(false) {
    this->initializeThreadAndBuffers();
}

WriteRecording::WriteRecording(double fps, int width, int height, float fx, float fy, float ppx, float ppy,
                               rs2_distortion model, const float *coefficients, const string &imageWriteFormat,
                               const string &depthWriteFormat, const string &parametersWriteFormat,
                               RotationType rotationType) :
        Recording(fps, width, height, fx, fy, ppx, ppy, model, coefficients, imageWriteFormat, depthWriteFormat,
                  parametersWriteFormat, rotationType), imageBuffer(), depthBuffer(), countBuffer(), writeFlag(true),
        bufferStartIndex(0), bufferEndIndex(0), bufferSize(0), writeRotation(rotationType),
        imageWriterInitialized(false), depthWriterInitialized(false) {
    this->initializeThreadAndBuffers();
}

WriteRecording::WriteRecording(double fps, rs2_intrinsics intrinsics, const string &imageWriteFormat,
                               const string &depthWriteFormat, const string &parametersWriteFormat,
                               RotationType rotationType) :
        Recording(fps, intrinsics, imageWriteFormat, depthWriteFormat, parametersWriteFormat, rotationType),
        imageBuffer(), depthBuffer(), countBuffer(), writeFlag(true), bufferStartIndex(0), bufferEndIndex(0),
        bufferSize(0), writeRotation(rotationType), imageWriterInitialized(false), depthWriterInitialized(false) {
    this->initializeThreadAndBuffers();
}

WriteRecording::~WriteRecording() {
    cout << "Entering WriteRecording destructor!" << endl;
    this->writeFlag = false;
    cout << "Wait until all remaining frames have been written!" << endl;
    this->writerThread.join();
    cout << "Finished writing!" << endl;

    this->releaseImageWriter();
    this->releaseDepthWriter();
}

void WriteRecording::setParameters(const rs2::video_stream_profile *_videoStreamProfile) {
    this->parameters.setParameters(_videoStreamProfile, this->writeRotation);
}

void WriteRecording::setParameters(const VideoCapture *_videoCapture) {
    this->parameters.setParameters(_videoCapture, this->writeRotation);
}

void WriteRecording::setParameters(const RecordingParameters *_recordingParameters) {
    this->parameters.setParameters(_recordingParameters, this->writeRotation);
}

bool WriteRecording::writeData(Mat *image, rs2::depth_frame *depth, unsigned long long counter) {
    if (depth == nullptr) {
        return this->writeData(image, (Mat *) nullptr, counter);
    }
    Mat depthMat = depth_frame_to_meters(*depth);
    return this->writeData(image, &depthMat, counter);
}

bool WriteRecording::writeData(Mat *image, Mat *depth, unsigned long long counter) {
    if ((image != nullptr && !this->imageWriterInitialized) || (depth != nullptr && !this->depthWriterInitialized)) {
        if (!this->imageWriterInitialized && !this->depthWriterInitialized) {
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
        this->imageBuffer[this->bufferEndIndex] = new Mat(*image);
    }

    if (this->depthBuffer[this->bufferEndIndex] != nullptr) {
        delete this->depthBuffer[this->bufferEndIndex];
        this->depthBuffer[this->bufferEndIndex] = nullptr;
    }
    if (depth != nullptr) {
        this->depthBuffer[this->bufferEndIndex] = new Mat(*depth);
    }

    this->bufferEndIndex = (this->bufferEndIndex + 1) % WriteRecording::dataBufferSize;

    this->lock.lock();
    this->bufferSize++;
    this->lock.unlock();
}

bool WriteRecording::writeData(Mat &image, Mat &depth, unsigned long long counter) {
    this->writeData(&image, &depth, counter);
}

void WriteRecording::bufferThreadWrite() {
    while (this->writeFlag || this->bufferSize > 0) {
        while (this->writeFlag && this->bufferSize == 0) {
            this_thread::yield();
        }
        if (this->bufferSize == 0) {
            break;
        }

        Mat *data;
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

        this->bufferStartIndex = (this->bufferStartIndex + 1) % WriteRecording::dataBufferSize;

        this->lock.lock();
        this->bufferSize--;
        this->lock.unlock();
    }
}

void WriteRecording::initializeThreadAndBuffers() {
    this->writerThread = thread(&WriteRecording::bufferThreadWrite, this);
    if (WriteRecording::dataBufferSize == 0) {
        auto config = readJsonFile(configDirectoryLocation + "recordingOutputDirectory.cfg");
        WriteRecording::dataBufferSize = config["writeBufferSize"].get<int>();
        if (WriteRecording::dataBufferSize < 1) {
            throw runtime_error(
                    "Can not work with a buffer size < 1! Was " + to_string(WriteRecording::dataBufferSize));
        }
    }

    this->imageBuffer.resize(WriteRecording::dataBufferSize);
    this->depthBuffer.resize(WriteRecording::dataBufferSize);
    this->countBuffer.resize(WriteRecording::dataBufferSize);
}

void WriteRecording::writeImage(Mat *image) {
    imageRotation(image);
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
    imageRotation(depth);
    if (this->parameters.depthFormat == "bin") {
        Mat convertedData;
        convertDepthToMillimetersUInt16(depth, convertedData);
        matWriteBinary(this->depthWriterBinary, convertedData);
        return;
    }
    throw runtime_error("Unknown depth format: \"" + this->parameters.depthFormat + "\"");
}

bool WriteRecording::initializeImageWriter() {
    if (this->parameters.imageFormat == "avi") {
        auto fourcc = VideoWriter::fourcc('M', 'J', 'P', 'G');
        auto size = Size(this->parameters.width, this->parameters.height);

        this->imageWriter = new VideoWriter(this->imageFile, fourcc, this->parameters.fps, size, true);
        if (!this->imageWriter->isOpened()) {
            delete this->imageWriter;
            this->imageWriter = nullptr;
            cerr << "Cannot save video to file" << endl;
            return false;
        }
        return true;
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
        this->imageWriter->release();
        delete this->imageWriter;
        return;
    } else if (this->parameters.imageFormat == "bin") {
        this->imageWriterBinary->close();
        delete this->imageWriterBinary;
        return;
    }
    throw runtime_error("Unknown image format: \"" + this->parameters.imageFormat + "\"");
}

void WriteRecording::releaseDepthWriter() {
    if (this->parameters.depthFormat.empty()) {
        return;
    } else if (this->parameters.depthFormat == "bin") {
        this->depthWriterBinary->close();
        delete this->depthWriterBinary;
        return;
    }
    throw runtime_error("Unknown depth format: \"" + this->parameters.depthFormat + "\"");
}
