#ifndef LIBUVC_CALIBRATION_H_
#define LIBUVC_CALIBRATION_H_

namespace libuvc_cam {
std::string uvc_read_calibration(int vid, int pid, const std::string& name);
}

#endif
