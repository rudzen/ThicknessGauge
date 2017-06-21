#pragma once

#include "CaptureInterface.h"
#include <PvApi.h>

/**
 * \brief Allows capture through PvAPI -> OpenCV data structure
 */
class CapturePvApi {

    const int mono = 1;

    const unsigned long def_packet_size = 8228;

    const cv::Rect_<unsigned long> default_roi = cv::Rect_<unsigned long>(0, 1006, 2448, 256);

    tg::tCamera camera_;

    tPvCameraInfo camera_info_;

    unsigned long frame_size_;

    unsigned long packet_size_;

    unsigned int retry_count_;

    bool initialized_;

    bool is_open_;

    static char const* error_last(tPvErr error);

    static const char* data_type_to_string(tPvDatatype aType);

    void query_attribute(const char* aLabel) const;

public:

    CapturePvApi()
        : frame_size_(0), packet_size_(def_packet_size), retry_count_(10), initialized_(false), is_open_(false) { }

    CapturePvApi(tg::tCamera myCamera, tPvCameraInfo cameraInfo, unsigned long frameSize)
        : camera_(myCamera),
          camera_info_(cameraInfo),
          frame_size_(frameSize), packet_size_(def_packet_size), retry_count_(10), initialized_(true), is_open_(false) { }

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
     * \param exposure The exposure to use (us)
     */
    void cap(int frame_count, std::vector<cv::Mat>& target_vector, unsigned long exposure);

    bool initialize();

    void uninitialize();

    static unsigned long camera_count();

    bool open();

    void close();

    void packet_size(const unsigned long new_value);

    unsigned long packet_size() const;

    void gain(unsigned long new_value) const;

    unsigned long gain() const;

    void exposure(unsigned long new_value) const;

    unsigned long exposure() const;

    void exposure_add(unsigned long value_to_add) const;

    void exposure_sub(unsigned long value_to_sub) const;

    void exposure_mul(unsigned long value_to_mul) const;

    void print_attr() const;

};
