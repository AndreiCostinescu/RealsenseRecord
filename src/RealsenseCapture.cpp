//
// Created by andrei on 25.08.21.
//

#include <RealsenseCapture.h>
#include <AndreiUtils/utilsRealsense.h>

using namespace AndreiUtils;
using namespace cv;
using namespace rs2;
using namespace std;


RealsenseCapture::RealsenseCapture(int fps, bool withRecord, int recordedFileNumber, const string &recordedBagFile,
                                   int colorWidth, int colorHeight, int depthWidth, int depthHeight,
                                   const string &recordImageFormat, const string &recordDepthFormat,
                                   const string &recordParametersFormat, bool writeFPSOnImage) :
        IMAGE_FPS(fps), DEPTH_FPS(fps), inputRecording(), outputRecording(), IMAGE_HEIGHT(colorHeight),
        IMAGE_WIDTH(colorWidth), DEPTH_HEIGHT(depthHeight), DEPTH_WIDTH(depthWidth), depthIntrinsics(),
        writeFPSOnImage(writeFPSOnImage) {
    if (withRecord) {
        this->outputRecording = new WriteRecording(recordImageFormat, recordDepthFormat, recordParametersFormat);
        this->outputRecording->setFiles(false);
    }

    if (recordedFileNumber > -1) {
        this->inputRecording = new ReadRecording(recordedFileNumber);
        if (withRecord) {
            this->outputRecording = new WriteRecording(recordImageFormat, recordDepthFormat, recordParametersFormat,
                                                       this->inputRecording->getParameters());
        }
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
        auto config = this->pipeline.start(startConfig);
        if (withRecord) {
            video_stream_profile colorProfile = config.get_stream(RS2_STREAM_COLOR).as<video_stream_profile>();
            this->outputRecording = new WriteRecording(recordImageFormat, recordDepthFormat, recordParametersFormat,
                                                       nullptr, &colorProfile);
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

    // Close all OpenCV windows
    destroyAllWindows();
}

void RealsenseCapture::run() {
    setUseOptimized(true);
    cout << "Terminate by pressing the 'q' or Esc key\n";

    // Set the location of OpenCV windows
    namedWindow("Color Image");
    moveWindow("Color Image", 50, 100);
    namedWindow("Depth Image");
    moveWindow("Depth Image", this->IMAGE_WIDTH + 50 + 17, 100);

    this->start = chrono::system_clock::now();
    while (true) {
        if (!this->updateFrame()) {
            break;
        }
        this->saveData();
        this->computeAndDisplayFps();

        // Show frame streams
        if (this->image.rows != 0 && this->image.cols != 0) {
            imshow("Color Image", this->image);
        }
        if (this->depth.rows != 0 && this->depth.cols != 0) {
            imshow("Depth Image", this->depth);
        }

        char c = (char) waitKey(1);
        if (c == 'q' || c == 27) {
            break;
        }
    }
}

bool RealsenseCapture::saveData() {
    if (this->outputRecording == nullptr) {
        return false;
    }
    return this->outputRecording->writeData(&(this->image), &(this->depth));
}

cv::Mat &RealsenseCapture::getImage() {
    return this->image;
}

cv::Mat RealsenseCapture::getImage() const {
    return this->image;
}

void RealsenseCapture::setImage(const Mat &_image) {
    this->image = _image;
}

cv::Mat &RealsenseCapture::getDepth() {
    return this->depth;
}

cv::Mat RealsenseCapture::getDepth() const {
    return this->depth;
}

void RealsenseCapture::setDepth(const Mat &_depth) {
    this->depth = _depth;
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
        if (!this->inputRecording->readData(&(this->image), &(this->depth))) {
            return false;
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

        this->image = frame_to_mat(this->imageFrame);

        this->depth = depth_frame_to_meters(this->depthFrame);
        this->depthIntrinsics = this->depthFrame.get_profile().as<video_stream_profile>().get_intrinsics();
    }

    return true;
}

void RealsenseCapture::computeAndDisplayFps() {
    // Calculate frames per second (fps) and show it on depth frame
    this->end = chrono::system_clock::now();
    this->fps = (int) (1000.0 / chrono::duration_cast<chrono::milliseconds>(this->end - this->start).count());
    string fps_str = string(to_string(this->fps) + "fps");
    // cout << "fps = " << fps << endl;
    if (this->writeFPSOnImage) {
        putText(this->image, fps_str, Point(2, 28), FONT_HERSHEY_COMPLEX, 1.0, this->SCALAR_BLUE, 1, LINE_AA);
    }
    this->start = chrono::system_clock::now();
}
