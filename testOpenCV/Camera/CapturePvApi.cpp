#include <thread>
#include <chrono>
#include "../namespaces/tg.h"
#include "CapturePvApi.h"

using namespace tg;

char const* CapturePvApi::error_last(tPvErr error) {

    std::string return_string;

    switch (error) {
    case ePvErrInternalFault: return_string += "an internal fault occurred";
        break;
    case ePvErrBadHandle: return_string += "the handle of the camera is invalid";
        break;
    case ePvErrBadSequence: return_string += "API isn't initialized or capture already started/camera already open";
        break;
    case ePvErrNotFound: return_string += "the requested attribute doesn't exist or the camera was not found";
        break;
    case ePvErrUnplugged: return_string += "the camera was found but unplugged during the function call";
        break;
    case ePvErrOutOfRange: return_string += "the supplied value is out of range";
        break;
    case ePvErrWrongType: return_string += "the requested attribute is not of the correct type";
        break;
    case ePvErrForbidden: return_string += "the requested attribute forbid this operation";
        break;
    case ePvErrResources: return_string += "resources requested from the OS were not available";
        break;
    case ePvErrAccessDenied: return_string += "the camera couldn't be open in the requested mode";
        break;
    case ePvErrBadParameter: return_string += "a valid pointer for pCamera was not supplied";
        break;
    default: return_string += "unknown error";
    }

    return return_string.c_str();

}

void CapturePvApi::reset_binning() const {

    // reset stuff like binning etc
    const auto def_binning = 1;
    auto err_code = PvAttrUint32Set(camera_.Handle, "BinningX", def_binning);
    err_code = PvAttrUint32Set(camera_.Handle, "BinningY", def_binning);

}

bool CapturePvApi::is_open() const {
    return is_open_;
}

void CapturePvApi::is_open(bool new_value) {
    is_open_ = new_value;
    if (new_value) {
        log_time << "Warning. Manually opening of camera unit invoked.\n";
    }
}

bool CapturePvApi::initialized() const {
    return initialized_;
}

void CapturePvApi::initialized(bool new_value) {
    this->initialized_ = new_value;
    if (new_value) {
        log_time << "Warning. Manually initialization of camera unit invoked.\n";
    }
}

unsigned CapturePvApi::retry_count() const {
    return retry_count_;
}

void CapturePvApi::retry_count(unsigned new_value) {
    if (new_value == retry_count()) {
        log_time << cv::format("Capture retry count already set to %i.\n", new_value);
        return;
    }
    log_time << cv::format("Capture retry count set to %i.\n", new_value);
    retry_count_ = new_value;

}

unsigned long CapturePvApi::frame_size() const {
    return frame_size_;
}

std::string CapturePvApi::version() const {
    unsigned long major = 0;
    unsigned long minor = 0;
    PvVersion(&major, &minor);
    return cv::format("%i.%i", major, minor);
}

bool CapturePvApi::region(cv::Rect_<unsigned long> new_region) const {

    auto failures = 0;

    if (!region_x(new_region.x))
        failures++;

    if (!region_y(new_region.y))
        failures++;

    if (!region_width(new_region.width))
        failures++;

    if (!region_height(new_region.height))
        failures++;

    return failures == 0;
}

cv::Rect_<unsigned long> CapturePvApi::region() const {
    auto x = region_x();
    auto y = region_y();
    auto width = region_width();
    auto height = region_height();
    return cv::Rect_<unsigned long>(x, y, width, height);
}

bool CapturePvApi::region_x(unsigned new_x) const {
    auto err_code = PvAttrUint32Set(camera_.Handle, "RegionX", new_x);
    if (err_code == ePvErrSuccess)
        return true;
    log_time << cv::format("Error setting roi x.. %s\n", error_last(err_code));
    return false;
}

unsigned long CapturePvApi::region_x() const {
    unsigned long x = 0;
    auto err_code = PvAttrUint32Get(camera_.Handle, "RegionX", &x);
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error getting roi x.. %s\n", error_last(err_code));
    }
    return x;
}

bool CapturePvApi::region_y(unsigned new_y) const {
    auto err_code = PvAttrUint32Set(camera_.Handle, "RegionY", new_y);
    if (err_code == ePvErrSuccess)
        return true;
    log_time << cv::format("Error setting roi y.. %s\n", error_last(err_code));
    return false;
}

unsigned long CapturePvApi::region_y() const {
    unsigned long y = 0;
    auto err_code = PvAttrUint32Get(camera_.Handle, "RegionY", &y);
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error getting roi y.. %s\n", error_last(err_code));
    }
    return y;
}

bool CapturePvApi::region_height(unsigned new_height) const {
    auto err_code = PvAttrUint32Set(camera_.Handle, "Height", new_height);
    if (err_code == ePvErrSuccess)
        return true;
    log_time << cv::format("Error setting roi height.. %s\n", error_last(err_code));
    return false;
}

unsigned long CapturePvApi::region_height() const {
    unsigned long height = 0;
    auto err_code = PvAttrUint32Get(camera_.Handle, "Height", &height);
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error getting roi height.. %s\n", error_last(err_code));
    }
    return height;
}

bool CapturePvApi::region_width(unsigned new_width) const {
    auto err_code = PvAttrUint32Set(camera_.Handle, "Width", new_width);
    if (err_code == ePvErrSuccess)
        return true;
    log_time << cv::format("Error setting roi width.. %s\n", error_last(err_code));
    return false;
}

unsigned long CapturePvApi::region_width() const {
    unsigned long width = 0;
    auto err_code = PvAttrUint32Get(camera_.Handle, "Width", &width);
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error getting roi width.. %s\n", error_last(err_code));
    }
    return width;
}

void CapturePvApi::cap(int frame_count, std::vector<cv::Mat>& target_vector, unsigned long exposure_to_use) {

    // Get the image size of every capture
    PvAttrUint32Get(camera_.Handle, "TotalBytesPerFrame", &frame_size_);

    auto err_code = PvCaptureAdjustPacketSize(camera_.Handle, def_packet_size);
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error.. adjusting packet size.. %s\n", error_last(err_code));
    }

    // Allocate a buffer to store the image
    memset(&camera_.Frame, 0, sizeof(tPvFrame));
    camera_.Frame.ImageBufferSize = frame_size_;
    camera_.Frame.ImageBuffer = new char[frame_size_];

    // Set maximum camera parameters - camera specific
    //int max_width = 2448;
    //int max_heigth = 2050;

    exposure(exposure_to_use);

    region(default_roi);

    auto roi = region();

    // Start the camera
    PvCaptureStart(camera_.Handle);

    // Set the camera to capture continuously
    PvAttrEnumSet(camera_.Handle, "AcquisitionMode", "Continuous");
    PvCommandRun(camera_.Handle, "AcquisitionStart");
    PvAttrEnumSet(camera_.Handle, "FrameStartTriggerMode", "Freerun");

    auto m = cv::Mat(roi.height, roi.width, CV_8UC1);

    for (auto i = frame_count; i--;) {
        if (!PvCaptureQueueFrame(camera_.Handle, &(camera_.Frame), nullptr)) {

            while (PvCaptureWaitForFrameDone(camera_.Handle, &(camera_.Frame), 100) == ePvErrTimeout) {
            }

            // Create an image header (mono image)
            // Push ImageBuffer data into the image matrix and clone it into target vector
            m.data = static_cast<uchar *>(camera_.Frame.ImageBuffer);
            target_vector.emplace_back(m.clone());
            cv::imwrite("ostefars.png", target_vector.back());
        }
    }

    // Stop the acquisition
    err_code = PvCommandRun(camera_.Handle, "AcquisitionStop");
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error..AcquisitionStop.. %s\n", error_last(err_code));
        throw err_code;
    }

    PvCaptureEnd(camera_.Handle);

}

bool CapturePvApi::initialize() {

    tPvErr err_code;
    if (!initialized_) {
        err_code = PvInitialize();
        initialized_ = err_code == ePvErrSuccess;
        is_open_ = false;
        if (!initialized_) { // something went to shiets...
            log_time << cv::format("Error. initialize(). %s\n", error_last(err_code));
            return false;
        }
    }

    auto retry = retry_count();

    unsigned long cameras = 0;

    log_time << cv::format("Getting basic camera information..");

    if (retry) {
        while (retry--) {
            cameras = camera_count();
            if (cameras)
                break;
            std::cout << '.';
            tg::sleep(150);
        }
    } else
        cameras = camera_count();

    std::cout << '\n';

    if (cameras && retry_count_) {
        log_time << cv::format("Found %i camera(s).\n", cameras);
    } else {
        log_time << cv::format("Failed to locate camera, try increasing retry amount. Current is %i\n", retry_count());
        close();
        return false;
    }

    unsigned int cam_list_count = PvCameraList(&camera_info_, 1, nullptr);

    is_open_ = cam_list_count;

    if (!is_open_) {
        log_time << "Error while getting camera info.\n";
        return false;
    }

    camera_.UID = camera_info_.UniqueId;

    return true;
}

void CapturePvApi::uninitialize() {
    PvUnInitialize();
    initialized_ = false;
}

unsigned long CapturePvApi::camera_count() {
    return PvCameraCount();
}

bool CapturePvApi::open() {

    if (!initialized_) {
        log_time << "PvApi not initialized.\n";
        return false;
    }

    auto err_code = PvCameraOpen(camera_.UID, ePvAccessMaster, &(camera_.Handle));
    is_open_ = err_code == ePvErrSuccess;

    if (!is_open_) { // something went to shiets...
        log_time << cv::format("Error. camera not open.. %s\n", error_last(err_code));
    }

    log_time << "Camera opened ok.\n";
    return true;
}

void CapturePvApi::close() {

    auto err_code = PvCameraClose(camera_.Handle);
    is_open_ = err_code != ePvErrSuccess;

    if (is_open_) {
        log_time << cv::format("Error. close(), %s\n", error_last(err_code));
        return;
    }
    log_time << "Camera closed.\n";
}

void CapturePvApi::exposure(unsigned long new_value) const {
    auto err_code = PvAttrUint32Set(camera_.Handle, "ExposureValue", new_value);
    if (err_code != ePvErrSuccess) {
        log_time << "Exposure changed failed.\n";
        return;
    }
    log_time << "Exposure changed to " << new_value << std::endl;
}

unsigned long CapturePvApi::exposure() const {
    unsigned long val = 0;
    auto err_code = PvAttrUint32Get(camera_.Handle, "ExposureValue", &val);
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error. exposure(). %s\n", error_last(err_code));
        return 0;
    }
    log_time << "Exposure fetched " << val << std::endl;
    return val;
}

void CapturePvApi::exposure_add(unsigned long value) const {
    exposure(exposure() + value);
}

void CapturePvApi::exposure_sub(unsigned long value) const {
    exposure(exposure() - value);
}

void CapturePvApi::exposure_mul(unsigned long value) const {
    exposure(exposure() * value);
}
