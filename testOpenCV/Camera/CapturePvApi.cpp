#include <thread>
#include <chrono>
#include "../namespaces/tg.h"
#include "CapturePvApi.h"

using namespace tg;

char const* CapturePvApi::error_last(tPvErr error) {

    std::string return_string;

    switch (error) {
    case ePvErrSuccess:
        return_string += "no error";
        break;
    case ePvErrInternalFault:
        return_string += "an internal fault occurred";
        break;
    case ePvErrBadHandle:
        return_string += "the handle of the camera is invalid";
        break;
    case ePvErrBadSequence:
        return_string += "API isn't initialized or capture already started/camera already open";
        break;
    case ePvErrNotFound:
        return_string += "the requested attribute doesn't exist or the camera was not found";
        break;
    case ePvErrUnplugged:
        return_string += "the camera was found but unplugged during the function call";
        break;
    case ePvErrOutOfRange:
        return_string += "the supplied value is out of range";
        break;
    case ePvErrWrongType:
        return_string += "the requested attribute is not of the correct type";
        break;
    case ePvErrForbidden:
        return_string += "the requested attribute forbid this operation";
        break;
    case ePvErrResources:
        return_string += "resources requested from the OS were not available";
        break;
    case ePvErrAccessDenied:
        return_string += "the camera couldn't be open in the requested mode";
        break;
    case ePvErrBadParameter:
        return_string += "a valid pointer for pCamera was not supplied";
        break;
    default:
        return_string += "unknown error";
    }

    return return_string.c_str();

}

const char* CapturePvApi::data_type_to_string(tPvDatatype aType) {
    switch (aType) {
    case ePvDatatypeUnknown:
        return "unknown";
    case ePvDatatypeCommand:
        return "command";
    case ePvDatatypeRaw:
        return "raw";
    case ePvDatatypeString:
        return "string";
    case ePvDatatypeEnum:
        return "enum";
    case ePvDatatypeUint32:
        return "uint32";
    case ePvDatatypeFloat32:
        return "float32";
    case ePvDatatypeInt64:
        return "int64";
    case ePvDatatypeBoolean:
        return "boolean";
    default:
        return "";
    }
}

void CapturePvApi::query_attribute(const char* aLabel) const {
    tPvAttributeInfo lInfo;

    if (PvAttrInfo(camera_.Handle, aLabel, &lInfo) != ePvErrSuccess)
        return;

    char lFlags[5];

    memset(lFlags, ' ', sizeof(char) * 4);

    if (lInfo.Flags & ePvFlagRead)
        lFlags[0] = 'r';
    if (lInfo.Flags & ePvFlagWrite)
        lFlags[1] = 'w';
    if (lInfo.Flags & ePvFlagVolatile)
        lFlags[2] = 'v';
    if (lInfo.Flags & ePvFlagConst)
        lFlags[3] = 'c';
    lFlags[4] = '\0';

    //	printf("%30s (%30s) [%7s]{%s}",aLabel,lInfo.Category,DatatypeToString(lInfo.Datatype),lFlags); 
    //    printf("%s/%s = %s [%s]{%s}\n",lInfo.Category,aLabel,lValue,DatatypeToString(lInfo.Datatype),lFlags); 

    switch (lInfo.Datatype) {
    case ePvDatatypeString:
        {
            char lValue[128];

            // we assume here that any string value will be less than 128 characters
            // long, which we may not be the case

            if (PvAttrStringGet(camera_.Handle, aLabel, lValue, 128, nullptr) == ePvErrSuccess)
                printf("%s/%s = %s [%s,%s]\n", lInfo.Category, aLabel, lValue, data_type_to_string(lInfo.Datatype), lFlags);
            else
                printf("ERROR!\n");

            break;
        }
    case ePvDatatypeEnum:
        {
            char lValue[128];

            // we assume here that any string value will be less than 128 characters
            // long, which we may not be the case

            if (PvAttrEnumGet(camera_.Handle, aLabel, lValue, 128, nullptr) == ePvErrSuccess)
                printf("%s/%s = %s [%s,%s]\n", lInfo.Category, aLabel, lValue, data_type_to_string(lInfo.Datatype), lFlags);
            else
                printf("ERROR!\n");
            break;
        }
    case ePvDatatypeUint32:
        {
            tPvUint32 lValue;

            if (PvAttrUint32Get(camera_.Handle, aLabel, &lValue) == ePvErrSuccess)
                printf("%s/%s = %lu [%s,%s]\n", lInfo.Category, aLabel, lValue, data_type_to_string(lInfo.Datatype), lFlags);
            else
                printf("ERROR!\n");
            break;
        }
    case ePvDatatypeInt64:
        {
            tPvInt64 lValue;

            if (PvAttrInt64Get(camera_.Handle, aLabel, &lValue) == ePvErrSuccess)
                printf("%s/%s = %lld [%s,%s]\n", lInfo.Category, aLabel, lValue, data_type_to_string(lInfo.Datatype), lFlags);
            else
                printf("ERROR!\n");
            break;
        }
    case ePvDatatypeFloat32:
        {
            tPvFloat32 lValue;

            if (PvAttrFloat32Get(camera_.Handle, aLabel, &lValue) == ePvErrSuccess)
                printf("%s/%s = %f [%s,%s]\n", lInfo.Category, aLabel, lValue, data_type_to_string(lInfo.Datatype), lFlags);
            else
                printf("ERROR!\n");
            break;
        }
    case ePvDatatypeBoolean:
        {
            tPvBoolean lValue;

            if (PvAttrBooleanGet(camera_.Handle, aLabel, &lValue) == ePvErrSuccess)
                printf("%s/%s = %s [%s,%s]\n", lInfo.Category, aLabel, lValue ? "true" : "false", data_type_to_string(lInfo.Datatype), lFlags);
            else
                printf("ERROR!\n");
            break;
        }
    default:
        //command
        printf("%s/%s [%s,%s]\n", lInfo.Category, aLabel, data_type_to_string(lInfo.Datatype), lFlags);
    }
}

bool CapturePvApi::load_calibration_data(std::string& filename) const {
    
    cv::FileStorage fs;
    fs.open(filename, cv::FileStorage::READ);
   
    if (!fs.isOpened()) {
        std::cerr << "Failed to open " << filename << std::endl;
        return false;
    }

    std::string time;

    //calibration_Time
    fs["calibration_Time"] >> time;
    log_time << "Loading calibration file created @ " << time << '\n';
    int width = 0;
    int height = 0;
    fs["image_width"] >> width;
    fs["image_height"] >> height;
    log_time << cv::format("Calibration dimensions : %ix%i.\n", width, height);
    int hsquares = 0;
    int vsquares = 0;
    fs["horizontal_squares"] >> hsquares;
    fs["vertical_squares"] >> vsquares;
    log_time << cv::format("Square count : %ix%i.\n", hsquares, vsquares);

    fs["rvecs"] >> cal->rvecs;
    log_time << cv::format("rvecs size : %i.\n", cal->rvecs.size());
    fs["tvecs"] >> cal->tvecs;
    log_time << cv::format("tvecs size : %i.\n", cal->tvecs.size());
    fs["intrinsic"] >> cal->intrinsic;
    log_time << cv::format("intrinsic size : %i.\n", cal->intrinsic.size());
    fs["dist_coeffs"] >> cal->dist_coeffs;
    log_time << cv::format("dist_coeffs size : %i.\n", cal->dist_coeffs.size());

    log_time << __FUNCTION__ << "Calibration data loaded ok!\n";

    cal->loaded = true;

    fs.release();

    return true;
}

bool CapturePvApi::frame_init() {

    // Get the image size of every capture
    auto err_code = PvAttrUint32Get(camera_.Handle, "TotalBytesPerFrame", &frame_size_);
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error while initializing frame! %s\n", error_last(err_code));
        return false;
    }

    // Allocate a buffer to store the image
    memset(&camera_.Frame, 0, sizeof(tPvFrame));
    camera_.Frame.ImageBufferSize = frame_size_;
    camera_.Frame.ImageBuffer = new char[frame_size_];

    return true;

}

int CapturePvApi::cap_init() const {
    // Start the camera
    auto err_code = PvCaptureStart(camera_.Handle);
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error starting capture.. %s\n", error_last(err_code));
        return -1;
    }

    unsigned long ready = 0;
    err_code = PvCaptureQuery(camera_.Handle, &ready);
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error. Capture query failed. %s\n", error_last(err_code));
        return -2;
    }
    if (ready == 0) {
        log_time << cv::format("Error. Unit is not ready to capture.\n");
        return -3;
    }

    return 0;
}

bool CapturePvApi::cap_end() const {
    auto err_code = PvCaptureEnd(camera_.Handle);
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error. Failed to end capture. %s\n", error_last(err_code));
        return false;
    }
    return true;
}

bool CapturePvApi::aquisition_init() const {
    // Set the camera to capture continuously
    auto err_code = PvAttrEnumSet(camera_.Handle, "AcquisitionMode", "Continuous");
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error setting acquisition mode.. %s\n", error_last(err_code));
        return false;
    }
    err_code = PvCommandRun(camera_.Handle, "AcquisitionStart");
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error setting acquisition start.. %s\n", error_last(err_code));
        return false;
    }
    err_code = PvAttrEnumSet(camera_.Handle, "FrameStartTriggerMode", "Freerun");
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error setting frame start trigger mode.. %s\n", error_last(err_code));
        return false;
    }
    return true;
}

bool CapturePvApi::aquisition_end() const {
    // Stop the acquisition
    auto err_code = PvCommandRun(camera_.Handle, "AcquisitionStop");
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error..AcquisitionStop.. %s\n", error_last(err_code));
        return false;
    }

    return true;
}

bool CapturePvApi::exposure_auto_reset() const {
    //--- /Controls/Exposure/Auto/ExposureAutoAlg = Mean [enum,rw  ]
    //--- /Controls/Exposure/Auto/ExposureAutoMax = 500000 [uint32,rw  ]
    //--- /Controls/Exposure/Auto/ExposureAutoMin = 25 [uint32,rw  ]

    const unsigned long auto_max = 500000;
    const unsigned long auto_min = 25;
    const std::string auto_alg = "Mean";

    using namespace tg;

    // set the defaults
    auto err_code = PvAttrUint32Set(camera_.Handle, "ExposureAutoMax", auto_max);
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error.. ExposureAutoMax.. %s\n", error_last(err_code));
        return false;
    }
    err_code = PvAttrUint32Set(camera_.Handle, "ExposureAutoMin", auto_min);
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error.. ExposureAutoMin.. %s\n", error_last(err_code));
        return false;
    }
    err_code = PvAttrEnumSet(camera_.Handle, "ExposureAutoAlg", auto_alg.c_str());
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error.. ExposureAutoAlg.. %s\n", error_last(err_code));
        return false;
    }

    // check the defaults to make sure they are configured correctly
    unsigned long auto_max_check = 0;
    unsigned long auto_min_check = 0;
    char lValue[128];

    err_code = PvAttrUint32Get(camera_.Handle, "ExposureAutoMax", &auto_max_check);
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error.. ExposureAutoMax.. %s\n", error_last(err_code));
        return false;
    }

    err_code = PvAttrUint32Get(camera_.Handle, "ExposureAutoMin", &auto_max_check);
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error.. ExposureAutoMin.. %s\n", error_last(err_code));
        return false;
    }


    err_code = PvAttrEnumGet(camera_.Handle, "ExposureAutoAlg", lValue, 128, nullptr);
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error.. ExposureAutoAlg.. %s\n", error_last(err_code));
        return false;
    }

    if (auto_max_check != auto_max) {
        log_time << cv::format("Error.. ExposureAutoMax mismatch.. %s\n", error_last(err_code));
        return false;
    }

    if (auto_max_check != auto_max) {
        log_time << cv::format("Error.. ExposureAutoMin mismatch.. %s\n", error_last(err_code));
        return false;
    }

    if (std::strcmp(auto_alg.c_str(), lValue) != 0) {
        log_time << cv::format("Error.. ExposureAutoAlg mismatch.. %s\n", error_last(err_code));
        return false;
    }

    return true;

}

bool CapturePvApi::exposure_auto_adjust_tolerance(unsigned long new_value) const {

    auto err_code = PvAttrUint32Set(camera_.Handle, "ExposureAutoAdjustTol", new_value);

    if (err_code == ePvErrSuccess)
        return true;

    using namespace tg;

    log_time << cv::format("Error.. ExposureAutoAdjustTol.. %s\n", error_last(err_code));
    return false;

}

unsigned long CapturePvApi::exposure_auto_adjust_tolerance() const {

    unsigned long ret_val = 0;

    auto err_code = PvAttrUint32Get(camera_.Handle, "ExposureAutoAdjustTol", &ret_val);

    if (err_code == ePvErrSuccess)
        return ret_val;

    using namespace tg;

    log_time << cv::format("Error.. ExposureAutoAdjustTol.. %s\n", error_last(err_code));
    return ret_val;

}

void CapturePvApi::reset_binning() const {
    // reset stuff like binning etc
    const auto def_binning = 1;
    auto err_code = PvAttrUint32Set(camera_.Handle, "BinningX", def_binning);
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error setting BinningX.. %s\n", error_last(err_code));
    }
    err_code = PvAttrUint32Set(camera_.Handle, "BinningY", def_binning);
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error setting BinningY.. %s\n", error_last(err_code));
    }
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

void CapturePvApi::cap(int frame_count, std::vector<cv::Mat>& target_vector) {

    // retrieve the roi to use
    auto roi = region();

    auto m = cv::Mat(roi.height, roi.width, CV_8UC1);

    cv::Mat undistorted;

    for (auto i = frame_count; i--;) {
        if (!PvCaptureQueueFrame(camera_.Handle, &(camera_.Frame), nullptr)) {

            while (true) {
                auto err_code = PvCaptureWaitForFrameDone(camera_.Handle, &(camera_.Frame), 100);
                if (err_code == ePvErrTimeout)
                    continue;
                if (err_code == ePvErrSuccess)
                    break;
                log_time << cv::format("Error while waiting for frame. %s\n", error_last(err_code));
            }

            // Create an image header (mono image)
            // Push ImageBuffer data into the image matrix and clone it into target vector

            m.data = static_cast<uchar *>(camera_.Frame.ImageBuffer);

            // if the calibration data has been loaded, the undistorted image is then used
            if (cal->loaded) {
                cv::undistort(m, undistorted, cal->intrinsic, cal->dist_coeffs);
                target_vector.emplace_back(undistorted);
            } else {
                target_vector.emplace_back(m.clone());
            }


            //cv::imwrite("ostefars.png", target_vector.back());
        }
    }
}

bool CapturePvApi::initialize() {

    if (!initialized_) {
        auto err_code = PvInitialize();
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

void CapturePvApi::packet_size(const unsigned long new_value) const {
    auto err_code = PvCaptureAdjustPacketSize(camera_.Handle, new_value);
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error.. adjusting packet size.. %s\n", error_last(err_code));
    }
}

void CapturePvApi::gain(unsigned long new_value) const {
    auto err_code = PvAttrUint32Set(camera_.Handle, "GainValue", new_value);
    if (err_code != ePvErrSuccess) {
        log_time << "Gain changed failed.\n";
        return;
    }
    log_time << "Gain changed to " << new_value << std::endl;
}

unsigned long CapturePvApi::gain() const {
    unsigned long val = 0;
    auto err_code = PvAttrUint32Get(camera_.Handle, "GainValue", &val);
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error. gain(). %s\n", error_last(err_code));
        return 0;
    }
    log_time << "Gain fetched " << val << std::endl;
    return val;
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

void CapturePvApi::print_attr() const {
    unsigned long count = 0;
    tPvAttrListPtr pListPtr;
    auto err_code = PvAttrList(camera_.Handle, &pListPtr, &count);
    if (err_code != ePvErrSuccess) {
        log_time << "Unable to read attributes..\n";
    }

    for (unsigned long i = 0; i < count; i++)
        query_attribute(pListPtr[i]);
}

void CapturePvApi::pixel_format(const PixelFormat format) const {
    std::string sformat;
    tPvImageFormat f;
    switch (format) {
    case PixelFormat::MONO8:
        sformat += "Mono8";
        f = ePvFmtMono8;
        break;
    case PixelFormat::MONO12:
        sformat += "Mono12";
        f = ePvFmtMono8;
        break;
    case PixelFormat::MONO12_PACKED:
        sformat += "Mono12Packed";
        f = ePvFmtMono12Packed;
        break;
    default:
        sformat += "Mono8";
        f = ePvFmtMono8;
    }

    // temporary check
    if (format != PixelFormat::MONO8) {
        log_time << "Warning, data structure not enabled for this format (yet).\n";
        sformat = "Mono8";
        f = ePvFmtMono8;
    }

    auto err_code = PvAttrEnumSet(camera_.Handle, "PixelFormat", sformat.c_str());
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error while setting pixel format. %s\n", error_last(err_code));
        return;
    }

    log_time << "Pixel format updated : " << sformat << '\n';


}

CapturePvApi::PixelFormat CapturePvApi::pixel_format() const {

    char lValue[128];
    auto err_code = PvAttrEnumGet(camera_.Handle, "PixelFormat", lValue, 128, nullptr);
    if (err_code != ePvErrSuccess) {
        log_time << cv::format("Error. Retrieving pixel format. %s\n", error_last(err_code));
        return PixelFormat::UNKNOWN;
    }

    std::string ret_string = lValue;

    log_time << "Pixel format : " << ret_string << '\n';

    if (ret_string == "Mono8")
        return PixelFormat::MONO8;
    if (ret_string == "Mono12")
        return PixelFormat::MONO12;
    if (ret_string == "Mono12Packed")
        return PixelFormat::MONO12_PACKED;
    
    return PixelFormat::UNKNOWN;

}
