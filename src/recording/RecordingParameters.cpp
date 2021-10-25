//
// Created by andrei on 27.11.20.
//

#include <RealsenseRecording/recording/RecordingParameters.h>
#include <utility>
#include <AndreiUtils/utils.hpp>
#include <AndreiUtils/utilsJson.h>

#ifdef OPENCV
#include <opencv2/opencv.hpp>
#endif

using namespace AndreiUtils;
using namespace nlohmann;
using namespace RealsenseRecording;
using namespace std;

const map<string, rs2_distortion> RecordingParameters::DISTORTION_MODELS = {
        {rs2_distortion_to_string(RS2_DISTORTION_NONE),                   RS2_DISTORTION_NONE},
        {rs2_distortion_to_string(RS2_DISTORTION_MODIFIED_BROWN_CONRADY), RS2_DISTORTION_MODIFIED_BROWN_CONRADY},
        {rs2_distortion_to_string(RS2_DISTORTION_INVERSE_BROWN_CONRADY),  RS2_DISTORTION_INVERSE_BROWN_CONRADY},
        {rs2_distortion_to_string(RS2_DISTORTION_FTHETA),                 RS2_DISTORTION_FTHETA},
        {rs2_distortion_to_string(RS2_DISTORTION_BROWN_CONRADY),          RS2_DISTORTION_BROWN_CONRADY},
        {rs2_distortion_to_string(RS2_DISTORTION_KANNALA_BRANDT4),        RS2_DISTORTION_KANNALA_BRANDT4},
        {rs2_distortion_to_string(RS2_DISTORTION_COUNT),                  RS2_DISTORTION_COUNT},
};

const vector<string> RecordingParameters::PARAMETER_FORMATS = {"xml", "json",};

RecordingParameters::RecordingParameters() : RecordingParameters("", "", "", RotationType::NO_ROTATION) {}

RecordingParameters::RecordingParameters(string imageFormat, string depthFormat, string parametersFormat,
                                         RotationType rotationType) :
        RecordingParameters(move(imageFormat), move(depthFormat), move(parametersFormat), nullptr,
                            RecordingParametersType::NO_PARAMETERS, rotationType) {}

RecordingParameters::RecordingParameters(string imageFormat, string depthFormat, string parametersFormat,
                                         const void *parameters, RecordingParametersType parametersType,
                                         RotationType rotationType) :
        fps(), rotation(rotationType), model(), coefficients(), imageFormat(move(imageFormat)),
        depthFormat(move(depthFormat)), parametersFormat(move(parametersFormat)), initialized(false) {
    if (parameters != nullptr) {
        switch (parametersType) {
            case RECORDING_PARAMETERS: {
                this->setParameters((RecordingParameters *) parameters, rotationType);
                break;
            }
            case REALSENSE_INTRINSICS: {
                this->setParameters((rs2::video_stream_profile *) parameters, rotationType);
                break;
            }
                #ifdef OPENCV
                case OPENCV: {
                    this->setParameters((cv::VideoCapture *) parameters, rotationType);
                    break;
                }
                #endif
            default: {
                break;
            }
        }
        this->initialized = true;
    }
}

RecordingParameters::RecordingParameters(double fps, int width, int height, float fx, float fy, float ppx, float ppy,
                                         rs2_distortion model, const float *coefficients, string imageFormat,
                                         string depthFormat, string parametersFormat, RotationType rotationType) :
        fps(fps), rotation(rotationType), model(model), coefficients(), imageFormat(move(imageFormat)),
        depthFormat(move(depthFormat)), parametersFormat(move(parametersFormat)) {
    this->setRotationDependentParameters(rotationType, width, height, fx, fy, ppx, ppy);
    for (int i = 0; i < 5; i++) {
        this->coefficients[i] = coefficients[i];
    }
    this->initialized = true;
}

RecordingParameters::RecordingParameters(double fps, rs2_intrinsics intrinsics, string imageFormat, string depthFormat,
                                         string parametersFormat, RotationType rotationType) :
        fps(fps), rotation(rotationType), width(intrinsics.width), height(intrinsics.height), fx(intrinsics.fx),
        fy(intrinsics.fy), ppx(intrinsics.ppx), ppy(intrinsics.ppy), model(intrinsics.model), coefficients(),
        imageFormat(move(imageFormat)), depthFormat(move(depthFormat)), parametersFormat(move(parametersFormat)) {
    this->setRotationDependentParameters(rotationType, intrinsics.width, intrinsics.height, intrinsics.fx,
                                         intrinsics.fy, intrinsics.ppx, intrinsics.ppy);
    for (int i = 0; i < 5; i++) {
        this->coefficients[i] = intrinsics.coeffs[i];
    }
    this->initialized = true;
}

RecordingParameters::~RecordingParameters() = default;

void RecordingParameters::clear() {
    this->fps = 0.0;
    this->rotation = RotationType::NO_ROTATION;
    this->width = 0;
    this->height = 0;
    this->fx = 0;
    this->fy = 0;
    this->ppx = 0;
    this->ppy = 0;
    for (float &coefficient: this->coefficients) {
        coefficient = 0;
    }
    this->model = RS2_DISTORTION_NONE;
    this->imageFormat = "";
    this->depthFormat = "";
    this->parametersFormat = "";
    this->initialized = false;
}

void RecordingParameters::setParameters(const rs2::video_stream_profile *_videoStreamProfile,
                                        RotationType rotationType) {
    this->fps = _videoStreamProfile->fps();
    rs2_intrinsics intrinsics = _videoStreamProfile->get_intrinsics();
    this->setRotationDependentParameters(rotationType, intrinsics.width, intrinsics.height, intrinsics.fx,
                                         intrinsics.fy, intrinsics.ppx, intrinsics.ppy);
    this->model = intrinsics.model;
    for (int i = 0; i < 5; i++) {
        this->coefficients[i] = intrinsics.coeffs[i];
    }
    this->initialized = true;
}

#ifdef OPENCV
void RecordingParameters::setParameters(const cv::VideoCapture *_videoCapture, RotationType rotationType) {
    this->fps = _videoCapture->get(cv::CAP_PROP_FPS);
    this->setRotationDependentParameters(rotationType, (int) _videoCapture->get(cv::CAP_PROP_FRAME_WIDTH),
                                         (int) _videoCapture->get(cv::CAP_PROP_FRAME_HEIGHT));
    this->initialized = true;
}
#endif

void RecordingParameters::setParameters(const RecordingParameters *recordingParameters,
                                        RotationType rotationType) {
    this->fps = recordingParameters->fps;
    this->setRotationDependentParameters(rotationType, recordingParameters->width, recordingParameters->height,
                                         recordingParameters->fx, recordingParameters->fy, recordingParameters->ppx,
                                         recordingParameters->ppy);
    this->model = recordingParameters->model;
    for (int i = 0; i < 5; i++) {
        this->coefficients[i] = recordingParameters->coefficients[i];
    }
    this->initialized = true;
}

void RecordingParameters::serialize(const string &parametersFile) const {
    if (this->parametersFormat == "xml") {
        #ifdef OPENCV
        cv::FileStorage fs(parametersFile, cv::FileStorage::WRITE);
        fs << "recordingData" << (*this);
        fs.release();
        #else
        cout << "Can not serialize parameters in xml format when opencv is not enabled!" << endl;
        #endif
    } else if (this->parametersFormat == "json") {
        json data;
        this->to_json(data);
        writeJsonFile(parametersFile, data);
    } else {
        throw runtime_error("Unknown parameter format...");
    }
}

void RecordingParameters::deserialize(const string &parametersFile, const string &_parametersFormat) {
    if (_parametersFormat == "xml") {
        #ifdef OPENCV
        cv::FileStorage recordingParameters(parametersFile, cv::FileStorage::READ);
        recordingParameters["recordingData"] >> (*this);
        recordingParameters.release();
        #else
        cout << "Can not deserialize parameters in xml format when opencv is not enabled!" << endl;
        #endif
    } else if (_parametersFormat == "json") {
        this->from_json(readJsonFile(parametersFile));
    } else {
        throw runtime_error("Unknown parameter format: " + _parametersFormat + "...");
    }
    this->initialized = true;
}

#ifdef OPENCV
void RecordingParameters::writeParameters(cv::FileStorage &fs) const {
    fs << "{";
    fs << "fps" << this->fps;
    fs << "rotation" << (int) this->rotation;
    fs << "w" << this->width;
    fs << "h" << this->height;
    fs << "fx" << this->fx;
    fs << "fy" << this->fy;
    fs << "ppx" << this->ppx;
    fs << "ppy" << this->ppy;
    fs << "imageFormat" << this->imageFormat;
    fs << "depthFormat" << this->depthFormat;
    fs << "parametersFormat" << this->parametersFormat;
    fs << "coefficient_0" << this->coefficients[0];
    fs << "coefficient_1" << this->coefficients[1];
    fs << "coefficient_2" << this->coefficients[2];
    fs << "coefficient_3" << this->coefficients[3];
    fs << "coefficient_4" << this->coefficients[4];
    fs << "distortion_model" << rs2_distortion_to_string(this->model);
    fs << "}";
}

void RecordingParameters::readParameters(const FileNode &node) {
    this->fps = (double) node["fps"];
    this->rotation = (RotationType) (int) node["rotation"];
    this->width = (int) node["w"];
    this->height = (int) node["h"];
    this->fx = (float) node["fx"];
    this->fy = (float) node["fy"];
    this->ppx = (float) node["ppx"];
    this->ppy = (float) node["ppy"];
    this->imageFormat = (string) node["imageFormat"];
    this->depthFormat = (string) node["depthFormat"];
    this->parametersFormat = (string) node["parametersFormat"];
    for (int i = 0; i < 5; i++) {
        this->coefficients[i] = (float) node["coefficient_" + to_string(i)];
    }

    rs2_distortion savedModel;
    bool found = mapGetIfContains(RecordingParameters::DISTORTION_MODELS, (string) node["distortion_model"],
                                  savedModel);
    if (!found) {
        throw std::runtime_error("Unknown distortion model " + (string) (node["distortion_model"]));
    }
    this->model = savedModel;
}
#endif

void RecordingParameters::to_json(json &j) const {
    j.clear();
    j["fps"] = this->fps;
    j["rotation"] = (int) this->rotation;
    j["w"] = this->width;
    j["h"] = this->height;
    j["fx"] = this->fx;
    j["fy"] = this->fy;
    j["ppx"] = this->ppx;
    j["ppy"] = this->ppy;
    j["imageFormat"] = this->imageFormat;
    j["depthFormat"] = this->depthFormat;
    j["parametersFormat"] = this->parametersFormat;
    j["coefficient_0"] = this->coefficients[0];
    j["coefficient_1"] = this->coefficients[1];
    j["coefficient_2"] = this->coefficients[2];
    j["coefficient_3"] = this->coefficients[3];
    j["coefficient_4"] = this->coefficients[4];
    j["distortion_model"] = rs2_distortion_to_string(this->model);
}

void RecordingParameters::from_json(const json &j) {
    this->fps = j["fps"].get<double>();
    this->rotation = (RotationType) j["rotation"].get<int>();
    this->width = j["w"].get<int>();
    this->height = j["h"].get<int>();
    this->fx = j["fx"].get<float>();
    this->fy = j["fy"].get<float>();
    this->ppx = j["ppx"].get<float>();
    this->ppy = j["ppy"].get<float>();
    this->imageFormat = j["imageFormat"].get<string>();
    this->depthFormat = j["depthFormat"].get<string>();
    this->parametersFormat = j["parametersFormat"].get<string>();
    for (int i = 0; i < 5; i++) {
        this->coefficients[i] = j["coefficient_" + to_string(i)].get<float>();
    }

    rs2_distortion savedModel;
    bool found = mapGetIfContains(RecordingParameters::DISTORTION_MODELS, j["distortion_model"].get<string>(),
                                  savedModel);
    if (!found) {
        throw std::runtime_error("Unknown distortion model " + j["distortion_model"].get<string>());
    }
    this->model = savedModel;
}

rs2_intrinsics RecordingParameters::getIntrinsics() {
    rs2_intrinsics intrinsics;
    intrinsics.width = this->width;
    intrinsics.height = this->height;
    intrinsics.fx = this->fx;
    intrinsics.fy = this->fy;
    intrinsics.ppx = this->ppx;
    intrinsics.ppy = this->ppy;
    intrinsics.model = this->model;
    for (int i = 0; i < 5; i++) {
        intrinsics.coeffs[i] = this->coefficients[i];
    }
    return intrinsics;
}

void RecordingParameters::setRotationDependentParameters(RotationType _rotation, int _width, int _height) {
    this->rotation = _rotation;
    switch (this->rotation) {
        case LEFT_90:
        case LEFT_270:
            this->width = _height;
            this->height = _width;
            break;
        default: {
            this->width = _width;
            this->height = _height;
            break;
        }
    }
}

void RecordingParameters::setRotationDependentParameters(RotationType _rotation, int _width, int _height, float _fx,
                                                         float _fy, float _ppx, float _ppy) {
    this->rotation = _rotation;
    switch (this->rotation) {
        case LEFT_90:
        case LEFT_270:
            this->width = _height;
            this->height = _width;
            this->fx = _fy;
            this->fy = _fx;
            this->ppx = _ppy;
            this->ppy = _ppx;
            break;
        default: {
            this->width = _width;
            this->height = _height;
            this->fx = _fx;
            this->fy = _fy;
            this->ppx = _ppx;
            this->ppy = _ppy;
            break;
        }
    }
}

bool RecordingParameters::isInitialized() const {
    return this->initialized;
}

#ifdef OPENCV
void cv::write(cv::FileStorage &fs, const string &, const RecordingParameters &x) {
    x.writeParameters(fs);
}

void cv::read(const cv::FileNode &node, RecordingParameters &x, const RecordingParameters &default_value) {
    if (node.empty()) {
        x = default_value;
    } else {
        x.readParameters(node);
    }
}
#endif

void to_json(nlohmann::json &j, const RecordingParameters &p) {
    p.to_json(j);
}

void from_json(const nlohmann::json &j, RecordingParameters &p) {
    p.from_json(j);
}
