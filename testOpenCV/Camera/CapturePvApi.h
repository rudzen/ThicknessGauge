#pragma once

#include "CaptureInterface.h"
#include <PvApi.h>
#include "UI/ProgressBar.h"
#include "namespaces/validate.h"

/**
 * \brief Allows capture through PvAPI -> OpenCV data structure
 */
class CapturePvApi {

public:

    enum class PixelFormat {
        MONO8, MONO12, MONO12_PACKED, UNKNOWN
    };

private:

    using calibrations = struct calibration_config {
        cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);

        cv::Mat dist_coeffs;

        std::vector<cv::Mat> rvecs;

        std::vector<cv::Mat> tvecs;

        bool loaded = false;

        /*
         * intrinsic.ptr<float>(0)[0] = 1;
         * intrinsic.ptr<float>(1)[1] = 1;
         */
    };

    std::unique_ptr<calibrations> cal = std::make_unique<calibrations>();

    const int mono = 1;

    const unsigned long def_packet_size = 8228;

    tg::tCamera camera_;

    tPvCameraInfo camera_info_;

    unsigned long frame_size_;

    unsigned int retry_count_;

    bool initialized_;

    bool is_open_;

    bool exposure_target_reached_;

    static char const* error_last(tPvErr error);

    static const char* data_type_to_string(tPvDatatype aType);

    void query_attribute(const char* aLabel) const;

public:

    const cv::Rect_<unsigned long> default_roi = cv::Rect_<unsigned long>(0, 1006, 2448, 256);

    CapturePvApi()
        : frame_size_(0)
          , retry_count_(10)
          , initialized_(false)
          , is_open_(false)
          , exposure_target_reached_(false) { }

    CapturePvApi(tg::tCamera myCamera, tPvCameraInfo cameraInfo, unsigned long frameSize)
        : camera_(myCamera)
          , camera_info_(cameraInfo)
          , frame_size_(frameSize)
          , retry_count_(10)
          , initialized_(true)
          , is_open_(false)
          , exposure_target_reached_(false) { }

    ~CapturePvApi() {
        delete[] static_cast<char*>(camera_.Frame.ImageBuffer);
    }

    bool load_calibration_data(std::string& filename) const;

    bool frame_init();

    int cap_init() const;

    bool cap_end() const;

    bool aquisition_init() const;

    bool aquisition_end() const;

    bool exposure_auto_reset() const;

    //Controls/Exposure/Auto/ExposureAutoOutliers = 0 [uint32,rw  ]
    //Controls/Exposure/Auto/ExposureAutoRate = 100 [uint32,rw  ]
    //Controls/Exposure/Auto/ExposureAutoTarget = 50 [uint32,rw  ]
    //Controls/Exposure/ExposureMode = Manual [enum,rwv ]

    void exposure_mode() { }

    bool exposure_auto_adjust_tolerance(unsigned long new_value) const;

    unsigned long exposure_auto_adjust_tolerance() const;

    bool exposure_auto_adjust_outliers(unsigned long new_value) const;

    unsigned long exposure_auto_adjust_outliers() const;

    void exposure_auto_rate(unsigned long new_value);

    unsigned long exposure_auto_rate();

    void exposure_auto_target(unsigned long new_value);

    unsigned long exposure_auto_target();

    /**
     * \brief Resets binning
     */
    void reset_binning() const;

    bool is_open() const;

    void is_open(bool new_value);

    bool initialized() const;

    void initialized(bool new_value);

    unsigned retry_count() const;

    void retry_count(unsigned new_value);

    unsigned long frame_size() const;

    std::string version() const;

    template <typename T>
    bool region_add_def_offset(cv::Rect_<T>& target) {
        static_assert(std::is_arithmetic<T>::value, "Wrong type.");
        target.y += default_roi.y;
        return validate::validate_rect(target);
    }

    template <typename T>
    bool region_sub_def_offset(cv::Rect_<T>& target) {
        static_assert(std::is_arithmetic<T>::value, "Wrong type.");
        target.y -= default_roi.y;
        return validate::validate_rect(target);
    }

    /**
     * \brief Apply a specific ROI to the camera
     * \param new_region The region as opencv rect of unsigned long
     * \return true if all 4 regions were set without errors
     */
    bool region(cv::Rect_<unsigned long> new_region) const;

    /**
     * \brief Retrieves the camera ROI
     * \return The roi as opencv rect type unsigned long
     */
    cv::Rect_<unsigned long> region() const;

    bool region_x(unsigned new_x) const;

    unsigned long region_x() const;

    bool region_y(unsigned new_y) const;

    unsigned long region_y() const;

    bool region_height(unsigned new_height) const;

    unsigned long region_height() const;

    bool region_width(unsigned new_width) const;

    unsigned long region_width() const;

    /**
     * \brief Captures frames synchron into a vector of opencv matricies using specified exposure
     * \param frame_count Amount of frames to capture
     * \param target_vector The target vector for the captured images
     */
    void cap(int frame_count, std::vector<cv::Mat>& target_vector);

    void cap_single(cv::Mat& target);

    bool initialize();

    void uninitialize();

    static unsigned long camera_count();

    bool open();

    void close();

    void packet_size(const unsigned long new_value) const;

    void gain(unsigned long new_value) const;

    unsigned long gain() const;

    void exposure(unsigned long new_value) const;

    unsigned long exposure() const;

    void exposure_add(unsigned long value_to_add) const;

    void exposure_sub(unsigned long value_to_sub) const;

    void exposure_mul(unsigned long value_to_mul) const;

    void pixel_format(const PixelFormat format) const;

    PixelFormat pixel_format() const;

    void print_attr() const;

};
