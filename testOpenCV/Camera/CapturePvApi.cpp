#include "CapturePvApi.h"

#include "../namespaces/tg.h"
#include <chrono>
#include <thread>

using namespace tg;

std::string CapturePvApi::version() const {
    unsigned long major = 0;
    unsigned long minor = 0;
    PvVersion(&major, &minor);
    return cv::format("%i.%i", major, minor);
}

std::string CapturePvApi::error() {

    //*               ePvErrBadHandle,       the handle of the camera is invalid
    //*               ePvErrUnplugged,       the camera has been unplugged 
    //*               ePvErrNotFound,        the requested attribute doesn't exist
    //*               ePvErrWrongType,       the requested attribute is not of the correct type
    //*               ePvErrForbidden,       the requested attribute forbid this operation
    //*               ePvErrOutOfRange,      the supplied value is out of range
    //*               ePvErrInternalFault,   an internal fault occurred
    //*               ePvErrBadSequence,     API isn't initialized

}

void CapturePvApi::region(cv::Rect_<unsigned long>& new_region) {

    Errcode = PvAttrUint32Set(myCamera.Handle, "RegionX", new_region.x);
    Errcode = PvAttrUint32Set(myCamera.Handle, "RegionY", new_region.y);
    Errcode = PvAttrUint32Set(myCamera.Handle, "Width", new_region.width);
    Errcode = PvAttrUint32Set(myCamera.Handle, "Height", new_region.height);

}

cv::Rect_<unsigned long> CapturePvApi::region() {

    unsigned long x;
    unsigned long y;
    unsigned long width;
    unsigned long height;

    Errcode = PvAttrUint32Get(myCamera.Handle, "RegionX", &x);
    Errcode = PvAttrUint32Get(myCamera.Handle, "RegionY", &y);
    Errcode = PvAttrUint32Get(myCamera.Handle, "Width", &width);
    Errcode = PvAttrUint32Get(myCamera.Handle, "Height", &height);

    return cv::Rect_<unsigned long>(x, y, width, height);

}

void CapturePvApi::retrieveAllInfo() {
}

void CapturePvApi::capture(int frame_count, double exposure) {
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
            //double time = static_cast<double>(cv::getTickCount());

            while (PvCaptureWaitForFrameDone(myCamera.Handle, &(myCamera.Frame), 100) == ePvErrTimeout) {
            }

            ////////////////////////////////////////////////////////
            // Here comes the OpenCV functionality for each frame //
            ////////////////////////////////////////////////////////

            // Create an image header (mono image)
            // Push ImageBuffer data into the image matrix
            //auto m = cv::Mat(roi.height, roi.width, CV_8UC1);
            m.data = static_cast<uchar *>(myCamera.Frame.ImageBuffer);
            target_vector.emplace_back(m.clone());

            //if (i)
            //    exposure(exposure() + 500);
        }
    }

    // Stop the acquisition
    Errcode = PvCommandRun(myCamera.Handle, "AcquisitionStop");
    if (Errcode != ePvErrSuccess)
        throw Errcode;

    PvCaptureEnd(myCamera.Handle);

}

void CapturePvApi::initialize() {

    if (initialized)
        return;

    Errcode = PvInitialize();

    initialized = Errcode == ePvErrSuccess;

    if (!initialized) { // something went to shiets...
        switch (Errcode) {
        case ePvErrResources:
            log_time << "Error while initializing PvApi, ePvErrResources\n";
            return;
        case ePvErrInternalFault:
            log_time << "Error while initializing PvApi, ePvErrInternalFault\n";
            return;
        default: ;
            log_time << "Error while initializing PvApi, Unknown\n";
            return;
        }
    }

    unsigned int retry_count = 10;

    while (retry_count--) {
        camera_count = PvCameraCount();
        if (camera_count)
            break;
        log_time << "Waiting for interface..\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }

    log_time << "Found " << camera_count << " cameras.\n";

    unsigned int cam_list_count = PvCameraList(&cameraInfo, 1, nullptr);

    if (!cam_list_count) {
        log_time << "Error while getting camera info.\n";
        return;
    }

    myCamera.UID = cameraInfo.UniqueId;

}

void CapturePvApi::open() {

    if (!initialized) {
        log_time << "PvApi not initialized.\n";
        return;
    }

    Errcode = PvCameraOpen(myCamera.UID, ePvAccessMaster, &(myCamera.Handle));

    isOpen = Errcode == ePvErrSuccess;

    if (!isOpen) { // something went to shiets...
        switch (Errcode) {
        case ePvErrAccessDenied:
            log_time << "Error.. the camera couldn't be open in the requested mode\n";
            return;
        case ePvErrNotFound:
            log_time << "Error.. the camera was not found (unplugged)\n";
            return;
        case ePvErrUnplugged:
            log_time << "Error.. the camera was found but unplugged during the function call\n";
            return;
        case ePvErrBadParameter:
            log_time << "Error.. a valid pointer for pCamera was not supplied\n";
            return;
        case ePvErrResources:
            log_time << "Error.. resources requested from the OS were not available\n";
            return;
        case ePvErrInternalFault:
            log_time << "Error.. an internal fault occurred\n";
            return;
        case ePvErrBadSequence:
            log_time << "Error.. API isn't initialized or camera is alreay open\n";
            return;
        default:
            log_time << "Error.. some mysterious error happend.. but what?\n";
            return;

        }
    }

    log_time << "Camera opened ok.\n";


}

void CapturePvApi::close() {

    Errcode = PvCameraClose(myCamera.Handle);

    isOpen = Errcode != ePvErrSuccess;

    if (isOpen) {
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
