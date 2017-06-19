#pragma once
#include <memory>
#include <ostream>
#include <opencv2/core/base.hpp>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/videoio/videoio_c.h>

#include "CaptureInterface.h"
#include "../namespaces/tg.h"

using namespace tg;

/**
 * \brief Wrapper for generic camera capture.
	// PVAPI
    CV_CAP_PROP_PVAPI_MULTICASTIP           = 300, // ip for anable multicast master mode. 0 for disable multicast
    CV_CAP_PROP_PVAPI_FRAMESTARTTRIGGERMODE = 301, // FrameStartTriggerMode: Determines how a frame is initiated
    CV_CAP_PROP_PVAPI_DECIMATIONHORIZONTAL  = 302, // Horizontal sub-sampling of the image
    CV_CAP_PROP_PVAPI_DECIMATIONVERTICAL    = 303, // Vertical sub-sampling of the image
    CV_CAP_PROP_PVAPI_BINNINGX              = 304, // Horizontal binning factor
    CV_CAP_PROP_PVAPI_BINNINGY              = 305, // Vertical binning factor
    CV_CAP_PROP_PVAPI_PIXELFORMAT           = 306, // Pixel format
 */
class OpenCVCap : public CaptureInterface {

private:

    std::unique_ptr<CaptureSettings> settings = std::make_unique<CaptureSettings>();

    // The target stddev to set configuration for.
    double targetStdDev_;

    double delta_;

public:

    cv::VideoCapture cap;

    OpenCVCap() {
        targetStdDev_ = NAN;
        delta_ = NAN;
    }

    ~OpenCVCap() = default;

    void initialize() {
        cap.set(CV_CAP_PROP_SETTINGS, 1);
        initialize(5000.0, 0.0);
    }

    void initialize(double exposure, double gain) {
        cap.set(CV_CAP_PROP_AUTO_EXPOSURE, 0);
        cap.set(CV_CAP_PROP_PVAPI_PIXELFORMAT, 0);
        exposure = align_min_value(exposure);
        gain = align_min_value(gain);
        setExposure(exposure);
        setGain(gain);
    }

    template <typename T>
    void initialize_vimba(Data<T>* data) {

        // MESSY FOR NOW

        // just to check if everything was fine :-)
        auto cam_ok = false;

        AVT::VmbAPI::CameraPtrVector cameras;
        auto& system = AVT::VmbAPI::VimbaSystem::GetInstance();
        if (VmbErrorSuccess == system.Startup()) {
            if (VmbErrorSuccess == system.GetCameras(cameras))
                data->cameraData->parse(cameras);
            if (cameras.empty())
            log_time << "Error, no cameras currently available.";
            data->cameraPtr = cameras.front();
        }

        // gather information if the camera is found
        if (data->cameraPtr != nullptr) {

            //interface permissions
            VmbErrorType error = data->cameraPtr->GetPermittedAccess(data->vimbaData->interfacePermittedAccess);
            if (error == VmbErrorSuccess) {
                log_time << "data->cameraPtr->GetPermittedAccess ok.\n";
            } else {
                log_time << "data->cameraPtr->GetPermittedAccess fail.\n";
            }

            // interface type
            error = data->cameraPtr->GetInterfaceType(data->vimbaData->interfaceType);

            if (error == VmbErrorSuccess) {
                log_time << "data->cameraPtr->GetInterfaceType ok.\n";
            } else {
                log_time << "data->cameraPtr->GetInterfaceType fail.\n";
            }

            auto vimb = data->vimbaData.get();

            // copy over data from the camera which info is already known
            vimb->pCameraID = data->cameraData->getId();
            vimb->pCameraName = data->cameraData->getName();
            vimb->pCameraModel = data->cameraData->getModel();
            vimb->pCameraSerialNumber = data->cameraData->getSn();
            vimb->pInterfaceID = data->cameraData->getInterfaceId();

            // attempt to retrieve the interface information
            AVT::VmbAPI::InterfacePtr pInterface;
            error = system.GetInterfaceByID(vimb->pInterfaceID, pInterface);
            if (error == VmbErrorSuccess) {
                log_time << "system.GetInterfaceByID ok.\n";
                std::string output_string;
                error = pInterface->GetSerialNumber(output_string);
                if (error == VmbErrorSuccess) {
                    vimb->pInterfaceSerialNumber = output_string.c_str();
                    log_time << cv::format("pInterface->GetSerialNumber ok : %s\n", output_string);
                    output_string.clear();
                } else {
                    log_time << "pInterface->GetSerialNumber fail.\n";
                    cam_ok = false;
                }
                error = pInterface->GetName(output_string);
                if (error == VmbErrorSuccess) {
                    vimb->pInterfaceName = output_string.c_str();
                    log_time << cv::format("pInterface->GetName ok : %s\n", output_string);
                    output_string.clear();
                } else {
                    log_time << "pInterface->GetName fail.\n";
                    cam_ok = false;
                }
            } else {
                log_time << "system.GetInterfaceByID fail.\n";
                cam_ok = false;
            }

            if (cam_ok) {
                // create the camera object pointer, with shareable properties
                data->camera = std::make_shared<GC2450MCamera>(vimb->pCameraID,
                                                               vimb->pCameraName,
                                                               vimb->pCameraModel,
                                                               vimb->pCameraSerialNumber,
                                                               vimb->pInterfaceID,
                                                               vimb->interfaceType,
                                                               vimb->pInterfaceName,
                                                               vimb->pInterfaceSerialNumber,
                                                               vimb->interfacePermittedAccess);

            } else {
                log_time << "Camera data reading failed..\n";
            }

        }

    }

    bool isOpen() const {
        return cap.isOpened();
    }

    void targetStdDev(const double target) {
        targetStdDev_ = target;
    }

    void deltaValue(const double delta) {
        delta_ = delta;
    }

    void setBrightness(double value) {
        value = align_min_value(value);
        cap.set(CV_CAP_PROP_BRIGHTNESS, value);
        settings->brightness = value;
    }

    double getBrightness(bool repoll = true) const {
        if (repoll)
            settings->brightness = cap.get(CV_CAP_PROP_BRIGHTNESS);
        return settings->brightness;
    }

    void setContrast(double value) {
        value = align_min_value(value);
        cap.set(CV_CAP_PROP_CONTRAST, value);
        settings->contrast = value;
    }

    double getContrast(bool repoll = true) const {
        if (repoll)
            settings->contrast = cap.get(CV_CAP_PROP_CONTRAST);
        return settings->contrast;
    }

    void setExposure(double value) {
        value = align_min_value(value);
        cap.set(CV_CAP_PROP_EXPOSURE, value);
        settings->exposure = value;
    }

    double getExposure(bool repoll = true) const {
        if (repoll)
            settings->exposure = cap.get(CV_CAP_PROP_EXPOSURE);
        return settings->exposure;
    }

    void setGain(double value) {
        value = align_min_value(value);
        cap.set(CV_CAP_PROP_GAIN, value);
        settings->gain = value;
    }

    double getGain(bool repoll = true) const {
        if (repoll)
            settings->gain = cap.get(CV_CAP_PROP_GAIN);
        return settings->gain;
    }

    void retrieveAllInfo() final;

    double detectExposure();

    friend std::ostream& operator<<(std::ostream& os, const OpenCVCap& obj) {
        os << "Capture device settings :\n";
        os << cv::format("brightness: %f\n", obj.settings->brightness);
        os << cv::format("contrast: %f\n", obj.settings->contrast);
        os << cv::format("saturation: %f\n", obj.settings->saturation);
        os << cv::format("hue: %f\n", obj.settings->hue);
        os << cv::format("gain: %f\n", obj.settings->gain);
        os << cv::format("exposure: %f\n", obj.settings->exposure);
        os << cv::format("Rgb: %f\n", obj.settings->Rgb);
        os << cv::format("white_balance_u: %f\n", obj.settings->white_balance_u);
        os << cv::format("white_balance_v: %f\n", obj.settings->white_balance_v);
        os << cv::format("rectification: %f\n", obj.settings->rectification);
        os << cv::format("iso_speed: %f\n", obj.settings->iso_speed);
        os << cv::format("buffersize: %f\n", obj.settings->buffersize);
        os << cv::format("Brightness: %f\n", obj.settings->brightness);
        os << cv::format("Brightness: %f\n", obj.settings->brightness);
        os << cv::format("Brightness: %f\n", obj.settings->brightness);
        return os;
    }
};
