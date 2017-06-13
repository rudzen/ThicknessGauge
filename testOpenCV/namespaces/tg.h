#pragma once
#include <memory>
#include <vector>
#include <opencv2/opencv.hpp>
#include "Vimba/CameraData.h"

/**
 * \brief Contains utilitary functionality for the entire program
 */
namespace tg {

    typedef std::vector<cv::Point2d> vd;
    typedef std::vector<cv::Point2i> vi;

    typedef std::pair<cv::Point2f, cv::Point2f> linePair;

    enum Side { Left = 0, Right = 1, Center = 2 };

    enum XY { X = 0, Y = 1 };

    enum class SortBy { X, Y };

    enum class TextDrawPosition { UpperLeft, UpperRight, LowerLeft, LowerRight };

    enum class WindowType { Input, Output, Temp };

    enum class SaveType { Image_Jpeg, Image_Png, Video };

    enum class Information { None, Basic, Full };

    enum class GlobType { Test, Calibration, Sequence };

    enum class FilterKernels { Simple3x3, Horizontal10x10 };

    using Data = struct Data {

        // the points of the laser on the marking
        std::vector<cv::Point2d> centerPoints;

        // the points thwere the laser hits ground zero on the LEFT side of the marking
        std::vector<cv::Point2d> leftPoints;

        // the points thwere the laser hits ground zero on the RIGHT side of the marking
        std::vector<cv::Point2d> rightPoints;

        // the missing link to the left
        std::vector<cv::Point2d> middleLeft;

        // the missing link to the right
        std::vector<cv::Point2d> middleRight;

        // start location for the 3 point vectors
        cv::Vec3d pointsStart;

        // the camera meta data
        std::unique_ptr<CameraData> cameraData = std::make_unique<CameraData>();

        // the main camera
        AVT::VmbAPI::CameraPtr cameraPtr;

        // the name of the glob loaded (or "camera" for live feed)
        std::string globName;

        // the rectangle which includes the entire marking
        cv::Rect2d markingRect;

        // left border vector
        cv::Vec4d leftBorder;

        // right border vector
        cv::Vec4d rightBorder;

        // the base lines where the laser hits the actual ground-zero
        cv::Vec4d baseLines;

        // the locations for where the base lines intersect with the marking border
        cv::Vec4d intersections;

        // enclusure of the laser line
        cv::Vec4d centerLine;

        // the points where the intersections are cut.
        // adjusted so potential unwanted information is not included in further calculations
        cv::Vec2d intersectionCuts;

        // the x coordinates for the pieces that are cut out.
        cv::Vec4d middlePieces;

        // the avg
        double leftAvg;
        double leftMiddleAvg;
        double centerAvg;
        double rightMiddleAvg;
        double rightAvg;

        // the difference between the baseline(s) and the laser line on the marking in sub-pixels
        double difference;

    };

    // ----------- output stuff ---------------
    enum class SyncCout { IO_LOCK, IO_UNLOCK };

    std::ostream& operator<<(std::ostream&, SyncCout);

#define sync_cout std::cout << SyncCout::IO_LOCK
#define sync_endl std::endl << SyncCout::IO_UNLOCK

    enum class LogTime { LOG_TIME, LOG_DATE, LOG_TIME_DATE };

    /**
     * \brief Overload of ostream to insert time and/or date beforehand
     * \return The processed ostream
     */
    std::ostream& operator<<(std::ostream&, LogTime);

#define log_timedate std::cout << LogTime::LOG_TIME_DATE
#define log_time std::cout << LogTime::LOG_TIME
#define log_date std::cout << LogTime::LOG_DATE

    // ----------------------------------------------

    // ----------- misc utility functions -----------

    /**
     * \brief Aligns a value to be no less than 0
     * \tparam T The value type (only fundamental types allowed)
     * \param value The value to align
     * \return The aligned value
     */
    template <class T>
    __forceinline T alignMinValue(T value);

    /**
     * \brief Fetches the current time and date in specificed format.
     * For more information : http://www.cplusplus.com/reference/ctime/strftime/
     * \return Date and/or time related information as string
     */
    std::string get_time_date(const std::string format);

    /**
     * \brief Fetches the current time and date as Date and time representation (%c).
     * For more information : http://www.cplusplus.com/reference/ctime/strftime/
     * \return Date and time as string
     */
    std::string get_time_date();

    /**
     * \brief Fetches the current time as ISO 8601 time format (HH:MM:SS) (%T).
     * For more information : http://www.cplusplus.com/reference/ctime/strftime/
     * \return Time as string
     */
    std::string get_time();

    /**
     * \brief Fetches the current time as Short YYYY-MM-DD date (%F).
     * For more information : http://www.cplusplus.com/reference/ctime/strftime/
     * \return Date as string
     */
    std::string get_date();

}
