//
// Created by andrei on 25.08.21.
//

#include <RealsenseRecording/RealsenseCapture.h>
#include <AndreiUtils/enums/StandardTypes.h>
#include <AndreiUtils/utilsRealsense.h>
#include <iostream>

#ifdef OPENCV

#include <AndreiUtils/utilsOpenCVRealsense.h>

#endif

using namespace AndreiUtils;
using namespace RealsenseRecording;
using namespace rs2;
using namespace std;

RealsenseCapture::RealsenseCapture(int fps, bool withRecord, int recordedFileNumber, const string &recordedBagFile,
                                   int colorWidth, int colorHeight, int depthWidth, int depthHeight,
                                   const string &recordImageFormat, const string &recordDepthFormat,
                                   const string &recordParametersFormat, bool withOpenCV, bool writeFPSOnImage) :
        IMAGE_FPS(fps), DEPTH_FPS(fps), inputRecording(), outputRecording(), IMAGE_HEIGHT(colorHeight),
        IMAGE_WIDTH(colorWidth), DEPTH_HEIGHT(depthHeight), DEPTH_WIDTH(depthWidth), depthIntrinsics(),
        writeFPSOnImage(writeFPSOnImage), withOpenCV(withOpenCV) {
    if (recordedFileNumber > -1) {
        this->inputRecording = new ReadRecording(recordedFileNumber);
        if (withRecord) {
            this->outputRecording = new WriteRecording(recordImageFormat, recordDepthFormat, recordParametersFormat,
                                                       this->inputRecording->getParameters(),
                                                       RecordingParametersType::RECORDING_PARAMETERS);
        }
        this->DEPTH_HEIGHT = this->IMAGE_HEIGHT = this->inputRecording->getParameters()->height;
        this->DEPTH_WIDTH = this->IMAGE_WIDTH = this->inputRecording->getParameters()->width;
        this->DEPTH_FPS = this->IMAGE_FPS = (int) this->inputRecording->getParameters()->fps;
        this->sleepTime = 1000 / fps;
    } else {
        if (!recordedBagFile.empty()) {
            this->startConfig.enable_device_from_file(recordedBagFile, false);
        } else {
            this->startConfig.enable_stream(RS2_STREAM_COLOR, this->IMAGE_WIDTH, this->IMAGE_HEIGHT, RS2_FORMAT_RGB8,
                                            this->IMAGE_FPS);
            this->startConfig.enable_stream(RS2_STREAM_DEPTH, this->DEPTH_WIDTH, this->DEPTH_HEIGHT, RS2_FORMAT_Z16,
                                            this->DEPTH_FPS);
        }

        // Start the pipeline for the streams
        auto config = this->pipeline.start(this->startConfig);
        video_stream_profile colorProfile = config.get_stream(RS2_STREAM_COLOR).as<video_stream_profile>();
        this->IMAGE_WIDTH = colorProfile.width();
        this->IMAGE_HEIGHT = colorProfile.height();
        this->IMAGE_FPS = colorProfile.fps();
        video_stream_profile depthProfile = config.get_stream(RS2_STREAM_DEPTH).as<video_stream_profile>();
        this->DEPTH_WIDTH = depthProfile.width();
        this->DEPTH_HEIGHT = depthProfile.height();
        this->DEPTH_FPS = depthProfile.fps();
        if (withRecord) {
            this->outputRecording = new WriteRecording(recordImageFormat, recordDepthFormat, recordParametersFormat,
                                                       &colorProfile, RecordingParametersType::REALSENSE_INTRINSICS);
        }
    }

    if (withRecord) {
        this->outputRecording->setFiles(false);
    }
}

RealsenseCapture::~RealsenseCapture() {
    delete this->outputRecording;
    this->outputRecording = nullptr;

    if (this->inputRecording == nullptr) {
        // Terminate the pipeline
        this->pipeline.stop();
    } else {
        delete this->inputRecording;
        this->inputRecording = nullptr;
    }

    #ifdef OPENCV
    // Close all OpenCV windows
    cv::destroyAllWindows();
    #endif
}

void RealsenseCapture::run() {
    #ifdef OPENCV
    cv::setUseOptimized(true);
    cout << "Terminate by pressing the 'q' or Esc key\n";

    // Set the location of OpenCV windows
    cv::namedWindow("Color Image");
    cv::moveWindow("Color Image", 50, 100);
    cv::namedWindow("Depth Image");
    cv::moveWindow("Depth Image", this->IMAGE_WIDTH + 50 + 17, 100);
    #endif

    this->fpsTimer.start();
    while (true) {
        if (!this->updateFrame()) {
            break;
        }
        this->saveData();
        this->computeAndDisplayFps();

        #ifdef OPENCV
        // Show frame streams
        if (this->image.rows != 0 && this->image.cols != 0) {
            cv::imshow("Color Image", this->image);
        }
        if (this->depth.rows != 0 && this->depth.cols != 0) {
            cv::imshow("Depth Image", this->depth);
        }

        char c = (char) cv::waitKey(1);
        if (c == 'q' || c == 27) {
            break;
        }
        #endif
    }
}

bool RealsenseCapture::saveData() {
    if (this->outputRecording == nullptr) {
        return false;
    }
    if (this->IMAGE_WIDTH != this->DEPTH_WIDTH || this->IMAGE_HEIGHT != this->DEPTH_HEIGHT) {
        cout << "Recording with different image and depth resolutions is not yet supported..." << endl;
        // TODO: support it :D
        return false;
    }
    if (this->withOpenCV) {
        #ifdef OPENCV
        return this->outputRecording->writeData(&(this->image), &(this->depth));
        #else
        throw runtime_error("Can not save data in opencv format without opencv backend enabled!");
        #endif
    }
    int nrElements = this->IMAGE_WIDTH * this->IMAGE_HEIGHT;
    return this->outputRecording->writeData(this->imageData, 3 * nrElements, this->depthData, nrElements);
}

#ifdef OPENCV

cv::Mat &RealsenseCapture::getImage() {
    return this->image;
}

cv::Mat RealsenseCapture::getImage() const {
    return this->image;
}

void RealsenseCapture::setImage(const cv::Mat &_image) {
    this->image = _image;
}

cv::Mat &RealsenseCapture::getDepth() {
    return this->depth;
}

cv::Mat RealsenseCapture::getDepth() const {
    return this->depth;
}

void RealsenseCapture::setDepth(const cv::Mat &_depth) {
    this->depth = _depth;
}

#endif

rs2::frame &RealsenseCapture::getImageFrame() {
    return this->imageFrame;
}

rs2::video_frame RealsenseCapture::getImageFrame() const {
    return this->imageFrame.as<rs2::video_frame>();
}

rs2::frame &RealsenseCapture::getDepthFrame() {
    return this->depthFrame;
}

rs2::depth_frame RealsenseCapture::getDepthFrame() const {
    return this->depthFrame.as<rs2::depth_frame>();
}

rs2_intrinsics &RealsenseCapture::getDepthIntrinsics() {
    return this->depthIntrinsics;
}

rs2_intrinsics RealsenseCapture::getDepthIntrinsics() const {
    return this->depthIntrinsics;
}

void RealsenseCapture::setDepthIntrinsics(const rs2_intrinsics &_depthIntrinsics) {
    this->depthIntrinsics = _depthIntrinsics;
}

bool RealsenseCapture::updateFrame() {
    if (this->inputRecording != nullptr) {
        if (this->withOpenCV) {
            #ifdef OPENCV
            if (!this->inputRecording->readData(&(this->image), &(this->depth))) {
                return false;
            }
            #else
            throw runtime_error("Can not use opencv backend to read recording data when opencv is not enabled!");
            #endif
        } else {
            // the readData function will delete old and allocate new memory for the new image data
            if (!this->inputRecording->readData(this->imageData, this->depthData)) {
                return false;
            }
        }
        this->depthIntrinsics = this->inputRecording->getIntrinsics();
        this_thread::sleep_for(chrono::milliseconds(this->sleepTime - 5));
    } else {
        // Wait for next set of frames
        try {
            this->frames = this->pipeline.wait_for_frames(1000);
        } catch (exception &e) {
            cout << "Caught exception while waiting for frames: " << e.what() << endl;
            return false;
        }

        // Get color & depth frames
        this->imageFrame = this->frames.get_color_frame();
        this->depthFrame = this->frames.get_depth_frame();

        if (this->withOpenCV) {
            #ifdef OPENCV
            this->image = frame_to_mat(this->imageFrame);
            this->depth = depth_frame_to_meters(this->depthFrame);
            #else
            cout << "Can not use opencv backend without opencv enabled..." << endl;
            #endif
        } else {
            delete[] this->imageData;
            auto videoFrame = this->imageFrame.as<rs2::video_frame>();
            int nrElements = videoFrame.get_height() * videoFrame.get_width() * videoFrame.get_bytes_per_pixel();
            this->imageData = new uint8_t[nrElements];
            int imageDataType;
            frameToBytes(this->imageFrame, this->imageData, imageDataType, nrElements);
            assert (imageDataType == StandardTypes::TYPE_UINT_8);
            delete[] this->depthData;
            videoFrame = this->depthFrame.as<rs2::video_frame>();
            nrElements = videoFrame.get_height() * videoFrame.get_width();
            this->depthData = new double[nrElements];
            depthFrameToMeters(this->depthFrame, this->depthData, nrElements);
        }

        this->depthIntrinsics = this->depthFrame.get_profile().as<video_stream_profile>().get_intrinsics();
    }
    return true;
}

void RealsenseCapture::computeAndDisplayFps() {
    // Calculate frames per second (fps) and show it on depth frame
    double time = this->fpsTimer.measure("ms");
    this->fps = (int) (1000.0 / time);
    string fps_str = string(to_string(this->fps) + "fps");
    // cout << "fps = " << fps << endl;
    if (this->writeFPSOnImage) {
        #ifdef OPENCV
        cv::putText(this->image, fps_str, cv::Point(2, 28), cv::FONT_HERSHEY_COMPLEX, 1.0, this->SCALAR_BLUE, 1,
                    cv::LINE_AA);
        #else
        cout << "Can not write fps on image when opencv is not enabled; fps = " << fps << endl;
        #endif
    }
    this->fpsTimer.start();
}
