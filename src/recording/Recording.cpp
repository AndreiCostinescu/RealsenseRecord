//
// Created by andrei on 26.11.20.
//

#include <RealsenseRecording/recording/Recording.h>
#include <AndreiUtils/utilsFiles.h>
#include <AndreiUtils/utilsJson.h>
#include <configDirectoryLocation.h>
#include <iostream>

using namespace AndreiUtils;
using namespace RealsenseRecording;
using namespace std;

string Recording::outputDirectory;
bool Recording::outputDirectoryInitialized = false;

string Recording::getOutputDirectory() {
    if (!Recording::outputDirectoryInitialized) {
        if (RealsenseRecording::configDirectoryLocation.empty()) {
            throw runtime_error("RealsenseRecording: configDirectoryLocation is not set...");
        }
        auto config = readJsonFile(RealsenseRecording::configDirectoryLocation + "recordingOutputDirectory.cfg");
        Recording::outputDirectory = config["outputDirectory"];
        Recording::outputDirectoryInitialized = true;
    }
    return Recording::outputDirectory;
}

string Recording::format(int number, const char *type, const string &format) {
    char buffer[50], outputFileNameBuffer[100];
    sprintf(buffer, "%d", number);
    strcpy(outputFileNameBuffer, "recording_");
    strcat(outputFileNameBuffer, type);
    strcat(outputFileNameBuffer, "_");
    strcat(outputFileNameBuffer, buffer);

    if (strcmp(type, "image") == 0 || strcmp(type, "video") == 0) {
        if (format != "bin" && format != "avi") {
            throw runtime_error("At file " + to_string(number) + ": unknown format for image: \"" + format +
                                R"(". Accepted are "bin" and "avi")");
        }
    } else if (strcmp(type, "depth") == 0) {
        if (format != "bin") {
            throw runtime_error("At file " + to_string(number) + ": unknown format for depth: \"" + format +
                                R"(". Accepted is "bin")");
        }
    } else if (strcmp(type, "parameters") == 0) {
        if (format != "xml" && format != "json") {
            throw runtime_error("At file " + to_string(number) + ": unknown format for parameters: \"" + format +
                                R"(". Accepted are "json" and "xml")");
        }
    } else {
        throw runtime_error("At file " + to_string(number) + ": unknown formatting type: " + string(type));
    }

    strcat(outputFileNameBuffer, ".");
    strcat(outputFileNameBuffer, format.c_str());
    return outputFileNameBuffer;
}

Recording::Recording(const string &imageFormat, const string &depthFormat, const string &parameterFormat,
                     RotationType rotationType) :
        Recording(imageFormat, depthFormat, parameterFormat, nullptr, RecordingParametersType::NO_PARAMETERS,
                  rotationType) {}

Recording::Recording(const string &imageFormat, const string &depthFormat, const string &parameterFormat,
                     const void *parameters, RecordingParametersType parametersType,
                     AndreiUtils::RotationType rotationType) :
        parameters(imageFormat, depthFormat, parameterFormat, parameters, parametersType, rotationType), imageFile(),
        depthFile(), parameterFile() {}

Recording::Recording(double fps, int width, int height, float fx, float fy, float ppx, float ppy,
                     rs2_distortion model, const float *coefficients, const string &imageFormat,
                     const string &depthFormat, const string &parameterFormat, RotationType rotationType) :
        parameters(fps, width, height, fx, fy, ppx, ppy, model, coefficients, imageFormat, depthFormat, parameterFormat,
                   rotationType), imageFile(), depthFile(), parameterFile() {}

Recording::Recording(double fps, rs2_intrinsics intrinsics, const string &imageFormat, const string &depthFormat,
                     const string &parameterFormat, RotationType rotationType) :
        parameters(fps, intrinsics, imageFormat, depthFormat, parameterFormat, rotationType),
        imageFile(), depthFile(), parameterFile() {}

Recording::~Recording() = default;

void Recording::setFiles(bool read, int fileNumber) {
    // if fileNumber < 0
    //      if read, then select the number of the last file that has content
    //      if write, then select the number of the last file that has content + 1
    // else
    //      if read, check that the parameter, image and depth files are there
    //      if write, delete the other files if any
    string checkImageFile, checkParameterFile, checkDepthFile, parameterFileFormat;
    bool foundFiles = false;

    for (int i = (fileNumber >= 0) ? fileNumber : 0;; i++) {
        // check the existence of a parameter file with number "i"
        for (const auto &j: RecordingParameters::PARAMETER_FORMATS) {
            parameterFileFormat = j;
            checkParameterFile = Recording::getOutputDirectory() +
                                 Recording::format(i, "parameters", parameterFileFormat);
            if (fileExists(checkParameterFile)) {
                break;
            }
            checkParameterFile = "";
        }
        if (read && checkParameterFile.empty()) {
            if (fileNumber >= 0) {
                throw runtime_error("Can't find parameter file for fileNumber: " + to_string(fileNumber));
            }
            break;
        } else if (!read && !checkParameterFile.empty()) {
            if (fileNumber >= 0) {
                // Delete the other files!
                RecordingParameters p;
                p.deserialize(checkParameterFile, parameterFileFormat);
                cout << "Warning: Deleting: " << checkParameterFile << endl;
                deleteFile(checkParameterFile);
                cout << "Warning: Deleting: "
                     << Recording::getOutputDirectory() + Recording::format(fileNumber, "video", p.imageFormat) << endl;
                deleteFile(Recording::getOutputDirectory() + Recording::format(fileNumber, "video", p.imageFormat));
                cout << "Warning: Deleting: "
                     << Recording::getOutputDirectory() + Recording::format(fileNumber, "depth", p.depthFormat) << endl;
                deleteFile(Recording::getOutputDirectory() + Recording::format(fileNumber, "depth", p.depthFormat));
            } else {
                continue;
            }
        }

        if (read) {
            this->parameters.deserialize(checkParameterFile, parameterFileFormat);
        } else {
            checkParameterFile = Recording::getOutputDirectory() +
                                 Recording::format(i, "parameters", this->parameters.parametersFormat);
        }
        checkImageFile = Recording::getOutputDirectory() +
                         Recording::format(i, "video", this->parameters.imageFormat);
        checkDepthFile = Recording::getOutputDirectory() +
                         Recording::format(i, "depth", this->parameters.depthFormat);

        if (read) {
            if (fileExists(checkImageFile) && fileExists(checkDepthFile)) {
                this->parameterFile = checkParameterFile;
                this->imageFile = checkImageFile;
                this->depthFile = checkDepthFile;
                foundFiles = true;
                if (fileNumber >= 0) {
                    break;
                }
            } else {
                break;
            }
        } else {
            this->parameterFile = checkParameterFile;
            this->imageFile = checkImageFile;
            this->depthFile = checkDepthFile;
            break;
        }
    }
    if (read && !foundFiles) {
        this->parameters.clear();
        string errorMsg = "One of these files is missing: " + checkImageFile + ", " + checkDepthFile;
        if (fileNumber >= 0) {
            throw runtime_error(errorMsg);
        } else {
            throw runtime_error(errorMsg + ", " + checkParameterFile);
        }
    }
}

rs2_intrinsics Recording::getIntrinsics() {
    return this->parameters.getIntrinsics();
}

const RecordingParameters *Recording::getParameters() {
    return &this->parameters;
}
