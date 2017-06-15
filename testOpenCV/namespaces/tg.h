//          Copyright Rudy Alex Kohn 2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once
#include <memory>
#include <vector>
#include <opencv2/opencv.hpp>
#include "Vimba/CameraData.h"

/**
 * \brief Contains utilitary functionality for the entire program
 */
namespace tg {

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

    enum class ValidationStatus { SUCESS, FAIL, ACCEPTABLE };

    enum class DataMemberIndex {
        CenterPoints,
        LeftPoints,
        RightPoints,
        PointsStart,
        CameraData,
        CameraPtr,
        GlobName,
        MarkingRect,
        LeftBorder,
        RightBorder,
        CenterLine,
        BaseLines,
        Intersections,
        IntersectionCuts,
        MiddlePieces,
        LeftAvg,
        CenterAvg,
        RightAvg,
        Difference
    };


    typedef std::tuple<DataMemberIndex, bool, ValidationStatus, std::string> ValidationResult;

    template <class T>
    class Data {
    public:

        // the points of the laser on the marking
        std::vector<cv::Point_<T>> centerPoints;

        // the points thwere the laser hits ground zero on the LEFT side of the marking
        std::vector<cv::Point_<T>> leftPoints;

        // the points thwere the laser hits ground zero on the RIGHT side of the marking
        std::vector<cv::Point_<T>> rightPoints;

        // start location for the 3 point vectors
        cv::Vec<T, 3> pointsStart;

        // the camera meta data
        std::unique_ptr<CameraData> cameraData = std::make_unique<CameraData>();

        // the main camera
        AVT::VmbAPI::CameraPtr cameraPtr;

        // the name of the glob loaded (or "camera" for live feed)
        std::string globName;

        // the rectangle which includes the entire marking
        cv::Rect_<T> markingRect;

        // left border vector
        cv::Vec<T, 4> leftBorder;

        // right border vector
        cv::Vec<T, 4> rightBorder;

        // the base lines where the laser hits the actual ground-zero
        cv::Vec<T, 4> baseLines;

        // the locations for where the base lines intersect with the marking border
        cv::Vec<T, 4> intersections;

        // enclusure of the laser line
        cv::Vec<T, 4> centerLine;

        // the points where the intersections are cut.
        // adjusted so potential unwanted information is not included in further calculations
        cv::Vec<T, 2> intersectionCuts;

        // the x coordinates for the pieces that are cut out.
        cv::Vec<T, 4> middlePieces;

        // validation results for the individual members of this structure
        std::vector<ValidationResult> validationResults;

        // overall validation status for this structure
        ValidationStatus status;

        // the avg
        T leftAvg;
        T leftMiddleAvg;
        T centerAvg;
        T rightMiddleAvg;
        T rightAvg;

        // the difference between the baseline(s) and the laser line on the marking in sub-pixels
        T difference;

        static std::string DataMemberName(DataMemberIndex index) {
            switch (index) {
            case DataMemberIndex::CenterPoints: return "CenterPoints";
            case DataMemberIndex::LeftPoints: return "LeftPoints";
            case DataMemberIndex::RightPoints: return "RightPoints";
            case DataMemberIndex::PointsStart: return "PointsStart";
            case DataMemberIndex::CameraData: return "CameraData";
            case DataMemberIndex::CameraPtr: return "CameraPtr";
            case DataMemberIndex::GlobName: return "GlobName";
            case DataMemberIndex::MarkingRect: return "MarkingRect";
            case DataMemberIndex::LeftBorder: return "LeftBorder";
            case DataMemberIndex::RightBorder: return "RightBorder";
            case DataMemberIndex::BaseLines: return "BaseLines";
            case DataMemberIndex::Intersections: return "Intersections";
            case DataMemberIndex::IntersectionCuts: return "IntersectionCuts";
            case DataMemberIndex::MiddlePieces: return "MiddlePieces";
            case DataMemberIndex::LeftAvg: return "LeftAvg";
            case DataMemberIndex::CenterAvg: return "CenterAvg";
            case DataMemberIndex::RightAvg: return "RightAvg";
            case DataMemberIndex::Difference: return "Difference";
            default: return "feeeeek";
            }
        }

        void addValidationResult(DataMemberIndex index, bool ok, ValidationStatus validation, std::string information) {
            validationResults.emplace_back(ValidationResult(index, ok, validation, information));
        }

    };

    // ----------- misc utility functions -----------

    /**
     * \brief Aligns a value to be no less than 0
     * \tparam T The value type (only fundamental types allowed)
     * \param value_to_align The value to align
     * \return The aligned value
     */
    template <typename T>
    T align_min_value(T value_to_align, T min_val = 0) {
        static_assert(std::is_fundamental<T>::value, "alignment is only possible for fundamental types.");
        if (value_to_align < min_val)
            value_to_align = 0;
        return value_to_align;
    }

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


}
