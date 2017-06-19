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
#include <VimbaImageTransform/Include/VmbTransform.h>

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

    std::unique_ptr<CaptureSettings> settings = std::make_unique<CaptureSettings>();

    // The target stddev to set configuration for.
    double targetStdDev_;

    double delta_;

    //
    // frame data temporary storage
    //
    struct frame_store {
    private:
        typedef std::vector<VmbUchar_t> data_vector;
        data_vector m_Data; // Frame data
        VmbUint32_t m_Width; // frame width
        VmbUint32_t m_Height; // frame height
        VmbPixelFormat_t m_PixelFormat; // frame pixel format
    public:
        //
        // Method: frame_store()
        //
        // Purpose: default constructing frame store from data pointer and dimensions
        //
        frame_store(const VmbUchar_t* pBuffer, VmbUint32_t BufferByteSize, VmbUint32_t Width, VmbUint32_t Height, VmbPixelFormatType PixelFormat)
            : m_Data(pBuffer, pBuffer + BufferByteSize)
              , m_Width(Width)
              , m_Height(Height)
              , m_PixelFormat(PixelFormat) {
        }

        //
        // Method: equal
        //
        // Purpose: compare frame store to frame dimensions
        //
        bool equal(VmbUint32_t Width, VmbUint32_t Height, VmbPixelFormat_t PixelFormat) const {
            return m_Width == Width
                && m_Height == Height
                && m_PixelFormat == PixelFormat;
        }

        //
        // Method: setData
        //
        // Purpose: copy data into frame store from matching source
        //
        // Returns: false if data size not equal to internal buffer size
        //
        bool setData(const VmbUchar_t* Buffer, VmbUint32_t BufferSize) {
            if (BufferSize == dataSize()) {
                std::copy(Buffer, Buffer + BufferSize, m_Data.begin());
                return true;
            }
            return false;
        }

        //
        // Methode: PixelFormat()
        //
        // Purpose: get pixel format of internal buffer.
        //
        VmbPixelFormat_t pixelFormat() const { return m_PixelFormat; }
        //
        // Methode: Width()
        //
        // Purpose: get image width.
        //
        VmbUint32_t width() const { return m_Width; }
        //
        // Methode: Height()
        //
        // Purpose: get image height
        //
        VmbUint32_t height() const { return m_Height; }
        //
        // Methode: dataSize()
        //
        // Purpose: get buffer size of internal data.
        //
        VmbUint32_t dataSize() const { return static_cast<VmbUint32_t>(m_Data.size()); }
        //
        // Methode: data()
        //
        // Purpose: get constant internal data pointer.
        //
        const VmbUchar_t* data() const { return &*m_Data.begin(); }
        //
        // Methode: data()
        //
        // Purpose: get internal data pointer.
        //
        VmbUchar_t* data() { return &*m_Data.begin(); }
    };

public:

    cv::VideoCapture cap;

    OpenCVCap() {
        targetStdDev_ = NAN;
        delta_ = NAN;
    }

    ~OpenCVCap() = default;

    cv::Mat m_ConvertImage;

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

    //
    // Method:      convertImage()
    //
    // Purpose:     converts frame_store data to internal openCV image for video encoding.
    //
    // Parameters:
    //
    //  [in]    frame   internal frame_store struct from queue to convert into m_ConvertImage
    //
    // Note:        access to m_Convert image will race the function is not thread safe and meant to be used as single writer to data.
    //
    bool convertImage(frame_store& frame) {
        VmbImage srcImage;
        VmbImage dstImage;
        srcImage.Size = sizeof(srcImage);
        dstImage.Size = sizeof(dstImage);
        VmbSetImageInfoFromPixelFormat(frame.pixelFormat(), frame.width(), frame.height(), & srcImage);
        VmbSetImageInfoFromPixelFormat(VmbPixelFormatRgb8, m_ConvertImage.cols, m_ConvertImage.rows, & dstImage);
        srcImage.Data = frame.data();
        dstImage.Data = m_ConvertImage.data;
        return VmbErrorSuccess == VmbImageTransform(&srcImage, &dstImage, NULL, 0);

    }

    template <typename T>
    void initialize_vimba(Data<T>* data) {

        // MESSY FOR NOW

        log_time << "Camera initialization in progress.\n";

        // just to check if everything was fine :-)
        auto cam_ok = true;

        AVT::VmbAPI::CameraPtrVector cameras;
        auto& system = AVT::VmbAPI::VimbaSystem::GetInstance();
        if (VmbErrorSuccess == system.Startup()) {
            if (VmbErrorSuccess == system.GetCameras(cameras))
                data->cameraData->parse(cameras);
            if (cameras.empty()) {
                log_time << "Error, no cameras currently available.";
                return;
            }
            data->cameraPtr = cameras.front();
        }

        // gather information if the camera is found
        if (data->cameraPtr != nullptr) {

            bool success;

            //interface permissions
            VmbErrorType error = data->cameraPtr->GetPermittedAccess(data->vimbaData->interfacePermittedAccess);
            success = error == VmbErrorSuccess;
            log_time << status("data->cameraPtr->GetPermittedAccess", success);
            if (!success)
                cam_ok = false;

            //if (success) {
            //    log_time << "data->cameraPtr->GetPermittedAccess ok.\n";
            //} else {
            //    log_time << "data->cameraPtr->GetPermittedAccess fail.\n";
            //    cam_ok = false;
            //}

            // interface type
            error = data->cameraPtr->GetInterfaceType(data->vimbaData->interfaceType);

            if (error == VmbErrorSuccess) {
                log_time << "data->data->cameraPtr->GetInterfaceType ok.\n";
            } else {
                log_time << "data->data->cameraPtr->GetInterfaceType fail.\n";
                cam_ok = false;
            }

            std::string output_string;

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
                error = pInterface->GetSerialNumber(output_string);
                if (error == VmbErrorSuccess) {
                    vimb->pInterfaceSerialNumber = output_string.c_str();
                    log_time << cv::format("pInterface->GetSerialNumber ok : %s\n", output_string.c_str());
                } else {
                    log_time << "pInterface->GetSerialNumber fail.\n";
                    cam_ok = false;
                }

                output_string.clear();

                error = pInterface->GetName(output_string);
                if (error == VmbErrorSuccess) {
                    vimb->pInterfaceName = output_string.c_str();
                    log_time << cv::format("pInterface->GetName ok : %s\n", output_string.c_str());
                } else {
                    log_time << "pInterface->GetName fail.\n";
                    cam_ok = false;
                }

                output_string.clear();

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

                if (data->camera == nullptr) {
                    log_time << "Failed to initialize camera.\n";
                } else {
                    log_time << "Camera initialized. Getting information.\n";
                }

                // test it

                log_time << "Checking for auto exposure. ";

                AVT::VmbAPI::FeaturePtr feature;
                double exposure_time = 0.0;
                error = data->camera->SetExposureTimeAbs(200.0);
                if (error != VmbErrorSuccess) {
                    log_time << "failed to set exposure....\n";
                }



                error = data->camera->GetExposureTimeAbs(exposure_time);
                log_time << exposure_time << std::endl;
                //error = data->camera->GetExposureAutoFeature(feature);
                if (error == VmbErrorSuccess) {
                    std::cout << "supported. ";
                    error = feature->GetName(output_string);
                    if (error == VmbErrorSuccess) {
                        std::cout << "activated.\n";
                        auto_exposure = true;
                    } else {
                        std::cout << "deactivated.\n";
                    }
                } else {
                    std::cout << "not supported.\n";
                }

            } else {
                log_time << "Generic fail while initializing camera.\n";
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
        return os;
    }

};
