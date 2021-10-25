//
// Created by Andrei on 20-Oct-21.
//

#ifndef REALSENSERECORD_RECORDINGPARAMETERSTYPE_H
#define REALSENSERECORD_RECORDINGPARAMETERSTYPE_H

namespace RealsenseRecording {
    enum RecordingParametersType {
        NO_PARAMETERS,
        RECORDING_PARAMETERS,
        REALSENSE_INTRINSICS,
        #ifdef OPENCV
        OPENCV_VIDEO_CAPTURE,
        #endif
    };
}

#endif //REALSENSERECORD_RECORDINGPARAMETERSTYPE_H
