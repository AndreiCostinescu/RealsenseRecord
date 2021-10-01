#include <AndreiUtils/utilsJson.h>
#include <iostream>
#include <RealsenseCapture.h>
#include <stdexcept>
#include <utils.h>

using namespace AndreiUtils;
using namespace std;

int main() {
    cout << "Hello World!" << endl;

    auto config = readJsonFile(configDirectoryLocation + "realsenseCaptureArguments.cfg");
    int fps = config["fps"].get<int>();
    bool withRecord = false;
    if (config.contains("withRecord")) {
        withRecord = config["withRecord"].get<bool>();
    }
    int recordedFileNumber = -1;
    if (config.contains("recordedFileNumber")) {
        recordedFileNumber = config["recordedFileNumber"].get<int>();
    }
    string bagFile;
    if (config.contains("bagFile")) {
        bagFile = config["bagFile"].get<string>();
    }
    int colorHeight = 720, colorWidth = 1280, depthHeight = 480, depthWidth = 640;
    if (config.contains("colorHeight")) {
        colorHeight = config["colorHeight"].get<int>();
    }
    if (config.contains("colorWidth")) {
        colorWidth = config["colorWidth"].get<int>();
    }
    if (config.contains("depthHeight")) {
        depthHeight = config["depthHeight"].get<int>();
    }
    if (config.contains("depthWidth")) {
        depthWidth = config["depthWidth"].get<int>();
    }
    string recordImageFormat = "avi", recordDepthFormat = "bin", recordParametersFormat = "json";
    if (config.contains("recordImageFormat") && !config["recordImageFormat"].get<string>().empty()) {
        recordImageFormat = config["recordImageFormat"].get<string>();
    }
    if (config.contains("recordDepthFormat") && !config["recordDepthFormat"].get<string>().empty()) {
        recordDepthFormat = config["recordDepthFormat"].get<string>();
    }
    if (config.contains("recordParametersFormat") && !config["recordParametersFormat"].get<string>().empty()) {
        recordParametersFormat = config["recordParametersFormat"].get<string>();
    }

    try {
        RealsenseCapture capture(fps, withRecord, recordedFileNumber, bagFile, colorWidth, colorHeight, depthWidth,
                                 depthHeight, recordImageFormat, recordDepthFormat, recordParametersFormat);
        capture.run();
    } catch (exception &ex) {
        cv::destroyAllWindows();
        cout << "Caught exception in main function: " << ex.what() << endl;
        return 1;
    }

    return 0;
}
