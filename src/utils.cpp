//
// Created by andrei on 01.10.21.
//

#include <RealsenseRecording/utils.h>
#include <configDirectoryLocation.h>

using namespace std;

void RealsenseRecording::setConfigDirectoryLocation(const std::string &_configDirectoryLocation) {
    RealsenseRecording::configDirectoryLocation = _configDirectoryLocation;
}
