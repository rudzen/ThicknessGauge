#pragma once

#include <PvApi.h>
#include "../namespaces/validate.h"

/**
 * \brief Allows capture through PvAPI -> OpenCV data structure
 */
class CapturePvApi {

public:

    /**
     * \brief Pixel format to be used with the camera
     */
    enum class PixelFormat {
        MONO8, MONO12, MONO12_PACKED, UNKNOWN
    };

private:

    /**
     * \brief Calibration configuration
     */
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

    const cv::Rect_<unsigned long> default_roi_full = cv::Rect_<unsigned long>(0, 0, 2448, 2040);

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
        // clean up!!
        delete[] static_cast<char*>(camera_.Frame.ImageBuffer);
    }

    /**
     * \brief Loads calibration information from file
     * \param filename The file that contains the calibration information
     * \return true if loaded, otherwise false
     */
    bool load_calibration_data(std::string& filename) const;

    /**
     * \brief Initializes the cameras frame buffer
     * \return true if initialized, otherwise false
     */
    bool frame_init();

    /**
     * \brief Initializes the capture process
     * \return 0 if ok, -1 if the capture could not be initialized, -2 if the query for capture failed, -3 if the unit is not ready
     */
    int cap_init() const;

    /**
     * \brief Closes the capture process
     * \return true if closed, false if something went wrong
     */
    bool cap_end() const;

    /**
     * \brief Sets the default aquisition settings for the camera:
     * 1) AcquisitionMode = Continuous
     * 2) Starts the aquisition
     * 3) FrameStartTriggerMode = Freerun
     * \return true if everything went ok, otherwise false
     */
    bool aquisition_init() const;

    /**
     * \brief Calls AcquisitionStop on the camera
     * \return true if ok, otherwise false
     */
    bool aquisition_end() const;

    /**
     * \brief Resets the cameras automatic exposure
     * \return true if everything was ok, otherwise false
     */
    bool exposure_auto_reset() const;

    //Controls/Exposure/Auto/ExposureAutoOutliers = 0 [uint32,rw  ]
    //Controls/Exposure/Auto/ExposureAutoRate = 100 [uint32,rw  ]
    //Controls/Exposure/Auto/ExposureAutoTarget = 50 [uint32,rw  ]
    //Controls/Exposure/ExposureMode = Manual [enum,rwv ]

    void exposure_mode() { }

    /**
     * \brief Sets the cameras auto exposure tolerance
     * \param new_value The new value to apply
     * \return 
     */
    bool exposure_auto_adjust_tolerance(unsigned long new_value) const;

    unsigned long exposure_auto_adjust_tolerance() const;

    bool exposure_auto_adjust_outliers(unsigned long new_value) const;

    unsigned long exposure_auto_adjust_outliers() const;

    void exposure_auto_rate(unsigned long new_value);

    unsigned long exposure_auto_rate();

    void exposure_auto_target(unsigned long new_value);

    unsigned long exposure_auto_target();

    /**
     * \brief Resets binning for X and Y on the camera to it's default (1)
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

    /**
     * \brief Sets the ROI x position on the camera
     * \param new_x The new x position to apply
     * \return true if set, false if failed
     */
    bool region_x(unsigned new_x) const;

    /**
     * \brief Gets the ROI x position from the camera
     * \return The x position currently used
     */
    unsigned long region_x() const;

    /**
     * \brief Sets the ROI y position on the camera
     * \param new_y The new y position to apply
     * \return true if set, false if failed
     */
    bool region_y(unsigned new_y) const;

    /**
     * \brief Gets the ROI y position from the camera
     * \return The y position currently used
     */
    unsigned long region_y() const;

    /**
     * \brief Sets the ROI height position on the camera
     * \param new_height The new height position to apply
     * \return true if set, false if failed
     */
    bool region_height(unsigned new_height) const;

    /**
     * \brief Gets the ROI height position from the camera
     * \return The height position currently used
     */
    unsigned long region_height() const;

    /**
     * \brief Apply a new ROI width to the camera
     * \param new_width The width to apply
     * \return true if set, false if failed
     */
    bool region_width(unsigned new_width) const;

    /**
     * \brief Retrives the current ROI width from the camera
     * \return The ROI width as reported by the camera
     */
    unsigned long region_width() const;

    /**
     * \brief Captures frames synchron into a vector of opencv matricies using specified exposure
     * \param frame_count Amount of frames to capture
     * \param target_vector The target vector for the captured images
     */
    void cap(int frame_count, std::vector<cv::Mat>& target_vector);

    /**
     * \brief Captures a single frame from the camera
     * \param target The matrix where the frame should be stored
     */
    void cap_single(cv::Mat& target);

    bool initialize();

    void uninitialize();

    /**
     * \brief Retrives the amount of cameras currently used through the API
     * \return The number of cameras reported back by the API
     */
    static unsigned long camera_count();

    /**
     * \brief Open the camera for use
     * \return true if ok, otherwise false
     */
    bool open();

    /**
     * \brief Close the camera
     */
    void close();

    /**
     * \brief Sets the packet size for the camera to use
     * \param new_value The new packet size (high jumbo frame value recommended)
     */
    void packet_size(const unsigned long new_value) const;

    /**
     * \brief Sets the gain value on the camera
     * \param new_value The new gain value to apply
     */
    void gain(unsigned long new_value) const;

    /**
     * \brief Retrives the current gain value from the camera
     * \return The current gain value
     */
    unsigned long gain() const;

    /**
     * \brief Sets a new exposure value at the camera
     * \param new_value The new value to apply
     * \return true if ok, otherwise false
     */
    bool exposure(unsigned long new_value) const;

    /**
     * \brief Retrieves the current camera exposure
     * \return The exposure
     */
    unsigned long exposure() const;

    /**
     * \brief Adds a value to the cameras current exposure
     * \param value_to_add The value to add
     */
    void exposure_add(unsigned long value_to_add) const;

    /**
     * \brief Substracts a value from the cameras current exposure
     * \param value_to_sub 
     */
    void exposure_sub(unsigned long value_to_sub) const;

    /**
     * \brief Divides the cameras current exposure with a value
     * \param value The value to divide the current exposure with
     */
    void exposure_div(unsigned long value) const;

    /**
     * \brief Multiplies the current camera exposure with a value
     * \param value_to_mul The value to multiply with
     */
    void exposure_mul(unsigned long value_to_mul) const;

    /**
     * \brief Sets a new pixel format for the camera to use (warning, only Mono8 appears to be valid)
     * \param format The format to set
     */
    void pixel_format(const PixelFormat format) const;

    /**
     * \brief Retrieves the current pixel format the camera uses
     * \return The pixel format
     */
    PixelFormat pixel_format() const;

    /**
     * \brief Prints all the attributes currently supported by the camera to the console.
     */
    void print_attr() const;

};
