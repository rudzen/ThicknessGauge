#include <thread>
#include <chrono>
#include "../namespaces/tg.h"
#include "CapturePvApi.h"

using namespace tg;

std::string CapturePvApi::error_attr(tPvErr error) {

    switch (error) {
    case ePvErrInternalFault: return std::string("an internal fault occurred");
    case ePvErrBadHandle: return std::string("the handle of the camera is invalid");
    case ePvErrBadSequence: return std::string("API isn't initialized");
    case ePvErrNotFound: return std::string("the requested attribute doesn't exist");
    case ePvErrUnplugged: return std::string("the camera has been unplugged");
    case ePvErrOutOfRange: return std::string("the supplied value is out of range");
    case ePvErrWrongType: return std::string("the requested attribute is not of the correct type");
    case ePvErrForbidden: return std::string("the requested attribute forbid this operation");
    default: return std::string("unknown error");
    }
}

void CapturePvApi::reset() {

    // reset stuff like binning etc
    const auto def_binning = 1;
    Errcode = PvAttrUint32Set(myCamera.Handle, "BinningX", def_binning);
    Errcode = PvAttrUint32Set(myCamera.Handle, "BinningY", def_binning);

}

bool CapturePvApi::isOpen() const {
    return isOpen_;
}

void CapturePvApi::isOpen(bool new_value) {
    isOpen_ = new_value;
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

unsigned CapturePvApi::retryCount() const {
    return retryCount_;
}

void CapturePvApi::retryCount(unsigned new_value) {
    if (new_value == retryCount()) {
        log_time << cv::format("Capture retry count already set to %i.\n", new_value);
        return;
    }
    log_time << cv::format("Capture retry count set to %i.\n", new_value);
    retryCount_ = new_value;

}

std::string CapturePvApi::version() const {
    unsigned long major = 0;
    unsigned long minor = 0;
    PvVersion(&major, &minor);
    return cv::format("%i.%i", major, minor);
}

bool CapturePvApi::region(cv::Rect_<unsigned long> new_region) {

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

cv::Rect_<unsigned long> CapturePvApi::region() {
    auto x = region_x();
    auto y = region_y();
    auto width = region_width();
    auto height = region_height();
    return cv::Rect_<unsigned long>(x, y, width, height);
}

bool CapturePvApi::region_x(unsigned new_x) {
    Errcode = PvAttrUint32Set(myCamera.Handle, "RegionX", new_x);
    if (Errcode == ePvErrSuccess)
        return true;
    log_time << cv::format("Error setting roi x.. %s\n", error_attr(Errcode));
    return false;
}

unsigned long CapturePvApi::region_x() {
    unsigned long x = 0;
    Errcode = PvAttrUint32Get(myCamera.Handle, "RegionX", &x);
    if (Errcode != ePvErrSuccess) {
        log_time << cv::format("Error getting roi x.. %s\n", error_attr(Errcode));
    }
    return x;
}

bool CapturePvApi::region_y(unsigned new_y) {
    Errcode = PvAttrUint32Set(myCamera.Handle, "RegionY", new_y);
    if (Errcode == ePvErrSuccess)
        return true;
    log_time << cv::format("Error setting roi y.. %s\n", error_attr(Errcode));
    return false;
}

unsigned long CapturePvApi::region_y() {
    unsigned long y = 0;
    Errcode = PvAttrUint32Get(myCamera.Handle, "RegionY", &y);
    if (Errcode != ePvErrSuccess) {
        log_time << cv::format("Error getting roi y.. %s\n", error_attr(Errcode));
    }
    return y;
}

bool CapturePvApi::region_height(unsigned new_height) {
    Errcode = PvAttrUint32Set(myCamera.Handle, "Height", new_height);
    if (Errcode == ePvErrSuccess)
        return true;
    log_time << cv::format("Error setting roi height.. %s\n", error_attr(Errcode));
    return false;
}

unsigned long CapturePvApi::region_height() {
    unsigned long height = 0;
    Errcode = PvAttrUint32Get(myCamera.Handle, "Height", &height);
    if (Errcode != ePvErrSuccess) {
        log_time << cv::format("Error getting roi height.. %s\n", error_attr(Errcode));
    }
    return height;
}

bool CapturePvApi::region_width(unsigned new_width) {
    Errcode = PvAttrUint32Set(myCamera.Handle, "Width", new_width);
    if (Errcode == ePvErrSuccess)
        return true;
    log_time << cv::format("Error setting roi width.. %s\n", error_attr(Errcode));
    return false;
}

unsigned long CapturePvApi::region_width() {
    unsigned long width = 0;
    Errcode = PvAttrUint32Get(myCamera.Handle, "Width", &width);
    if (Errcode != ePvErrSuccess) {
        log_time << cv::format("Error getting roi width.. %s\n", error_attr(Errcode));
    }
    return width;
}

void CapturePvApi::retrieveAllInfo() {
}

void CapturePvApi::capture(int frame_count, std::vector<cv::Mat>& target_vector, unsigned long exposure_to_use) {

    exposure(exposure_to_use);
    capture(frame_count, target_vector);

}

void CapturePvApi::capture(int frame_count, std::vector<cv::Mat>& target_vector) {

    // Get the image size of every capture
    PvAttrUint32Get(myCamera.Handle, "TotalBytesPerFrame", &frameSize);

    // Allocate a buffer to store the image
    memset(&myCamera.Frame, 0, sizeof(tPvFrame));
    myCamera.Frame.ImageBufferSize = frameSize;
    myCamera.Frame.ImageBuffer = new char[frameSize];

    // Set maximum camera parameters - camera specific
    //int max_width = 2448;
    //int max_heigth = 2050;

    region(default_roi);

    auto roi = region();

    // Start the camera
    PvCaptureStart(myCamera.Handle);

    // Set the camera to capture continuously
    PvAttrEnumSet(myCamera.Handle, "AcquisitionMode", "Continuous");
    PvCommandRun(myCamera.Handle, "AcquisitionStart");
    PvAttrEnumSet(myCamera.Handle, "FrameStartTriggerMode", "Freerun");

    auto m = cv::Mat(roi.height, roi.width, CV_8UC1);

    for (auto i = frame_count; i--;) {
        if (!PvCaptureQueueFrame(myCamera.Handle, &(myCamera.Frame), nullptr)) {

            while (PvCaptureWaitForFrameDone(myCamera.Handle, &(myCamera.Frame), 100) == ePvErrTimeout) {
            }

            // Create an image header (mono image)
            // Push ImageBuffer data into the image matrix and clone it into target vector
            m.data = static_cast<uchar *>(myCamera.Frame.ImageBuffer);
            target_vector.emplace_back(m.clone());
        }
    }

    // Stop the acquisition
    Errcode = PvCommandRun(myCamera.Handle, "AcquisitionStop");
    if (Errcode != ePvErrSuccess)
        throw Errcode;

    PvCaptureEnd(myCamera.Handle);

}

bool CapturePvApi::initialize() {

    if (!initialized_) {
        Errcode = PvInitialize();
        initialized_ = Errcode == ePvErrSuccess;
        isOpen_ = false;
        if (!initialized_) { // something went to shiets...
            switch (Errcode) {
            case ePvErrResources:
                log_time << "Error.. resources requested from the OS were not available\n";
                break;
            case ePvErrInternalFault:
                log_time << "Error.. an internal fault occurred\n";
                break;
            default: ;
                log_time << "Error.. Unknown error while attempting to initialize PvApi\n";
                break;
            }
            return false;
        }
    }

    auto retry_count = retryCount();

    log_time << cv::format("Getting camera count\n");

    if (retry_count) {
        while (retry_count--) {
            camera_count = PvCameraCount();
            if (camera_count)
                break;
            log_time << cv::format("Retrying... %i..\n", retry_count);
            tg::sleep(150);
        }
    } else
        camera_count = PvCameraCount();

    std::cout << '\n';

    if (camera_count && retryCount_) {
        log_time << cv::format("Found %i cameras.\n", camera_count);
    } else {
        log_time << cv::format("Failed to locate any cameras, please try increasing retry amount. Current retry amount is %i\n", retryCount());
        isOpen_ = false;
        close();
    }

    unsigned int cam_list_count = PvCameraList(&cameraInfo, 1, nullptr);

    isOpen_ = cam_list_count;

    if (!isOpen_) {
        log_time << "Error while getting camera info.\n";
        return false;
    }

    myCamera.UID = cameraInfo.UniqueId;
    return true;
}

void CapturePvApi::uninitialize() {
    PvUnInitialize();
    initialized_ = false;
}

bool CapturePvApi::open() {

    if (!initialized_) {
        log_time << "PvApi not initialized.\n";
        return false;
    }

    Errcode = PvCameraOpen(myCamera.UID, ePvAccessMaster, &(myCamera.Handle));
    isOpen_ = Errcode == ePvErrSuccess;

    if (!isOpen_) { // something went to shiets...
        switch (Errcode) {
        case ePvErrAccessDenied:
            log_time << "Error.. the camera couldn't be open in the requested mode\n";
            return false;
        case ePvErrNotFound:
            log_time << "Error.. the camera was not found (unplugged)\n";
            return false;
        case ePvErrUnplugged:
            log_time << "Error.. the camera was found but unplugged during the function call\n";
            return false;
        case ePvErrBadParameter:
            log_time << "Error.. a valid pointer for pCamera was not supplied\n";
            return false;
        case ePvErrResources:
            log_time << "Error.. resources requested from the OS were not available\n";
            return false;
        case ePvErrInternalFault:
            log_time << "Error.. an internal fault occurred\n";
            return false;
        case ePvErrBadSequence:
            log_time << "Error.. API isn't initialized or camera is alreay open\n";
            return false;
        default:
            log_time << "Error.. some mysterious error happend.. but what?\n";
            return false;
        }
    }

    log_time << "Camera opened ok.\n";
    return true;
}

void CapturePvApi::close() {

    Errcode = PvCameraClose(myCamera.Handle);
    isOpen_ = Errcode != ePvErrSuccess;

    if (isOpen_) {
        switch (Errcode) {
        case ePvErrBadHandle:
            log_time << "Error.. the handle of the camera is invalid\n";
            return;
        case ePvErrBadSequence:
            log_time << "Error.. API isn't initialized\n";
            return;
        default:
            log_time << "Error.. unknown error while closing camera.\n";
            return;
        }
    }
    log_time << "Camera closed.\n";
}

void CapturePvApi::exposure(unsigned long new_value) {
    Errcode = PvAttrUint32Set(myCamera.Handle, "ExposureValue", new_value);
    if (Errcode != ePvErrSuccess) {
        log_time << "Exposure changed failed.\n";
        return;
    }
    log_time << "Exposure changed to " << new_value << std::endl;
}

unsigned long CapturePvApi::exposure() {
    unsigned long val = 0;
    Errcode = PvAttrUint32Get(myCamera.Handle, "ExposureValue", &val);
    if (Errcode != ePvErrSuccess) {
        log_time << "Exposure reading failed.\n";
        return 0;
    }
    log_time << "Exposure fetched " << val << std::endl;
    return val;
}

void CapturePvApi::exposure_add(unsigned long value) {
    exposure(exposure() + value);
}

void CapturePvApi::exposure_sub(unsigned long value) {
    exposure(exposure() - value);
}

void CapturePvApi::exposure_mul(unsigned long value) {
    exposure(exposure() * value);
}
