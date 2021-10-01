//
// Created by andrei on 27.08.21.
//

#ifndef REALSENSERECORD_UTILS_H
#define REALSENSERECORD_UTILS_H

#include <string>

namespace RealsenseRecording {
    extern std::string configDirectoryLocation;

    void setConfigDirectoryLocation(const std::string &_configDirectoryLocation);
}

#endif //REALSENSERECORD_UTILS_H
