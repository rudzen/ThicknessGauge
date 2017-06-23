//          Copyright Rudy Alex Kohn 2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once
#include <memory>
#include <vector>
#include <opencv2/opencv.hpp>
#include <PvApi.h>
#include <thread>
#include <chrono>
#include "../Vimba/GC2450MCamera.h"
#include "Vimba/CameraData.h"

/**
 * \brief Contains utilitary functionality for the entire program
 */
namespace tg {

    template <typename T>
    struct line_pair {
        cv::Point_<T> p1;

        cv::Point_<T> p2;

        line_pair(T x1, T x2, T y1, T y2) {
            p1.x = x1;
            p1.y = y1;
            p2.x = x2;
            p2.y = y2;
        }

        line_pair(cv::Point_<T> p1, cv::Point_<T> p2)
            : p1(p1)
              , p2(p2) {}

        friend bool operator==(const line_pair& lhs, const line_pair& rhs) {
            return lhs.p1 == rhs.p1
                && lhs.p2 == rhs.p2;
        }

        friend bool operator!=(const line_pair& lhs, const line_pair& rhs) {
            return !(lhs == rhs);
        }
    };

    //    typedef std::pair<cv::Point2f, cv::Point2f> linePair;

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

    /**
     * \brief Vimba camera structure
     */
    using VimbaData = struct vim {
        const char* pCameraID;

        const char* pCameraName;

        const char* pCameraModel;

        const char* pCameraSerialNumber;

        const char* pInterfaceID;

        VmbInterfaceType interfaceType;

        const char* pInterfaceName;

        const char* pInterfaceSerialNumber;

        VmbAccessModeType interfacePermittedAccess;
    };

    /**
     * \brief PvApi camera structure
     */
    typedef struct {
        unsigned long UID;

        tPvHandle Handle;

        tPvFrame Frame;
    } tCamera;

    template <class T>
    class Data {
    public:

        // the points of the laser on the marking
        std::vector<cv::Point_<T>> center_points;

        // the points thwere the laser hits ground zero on the LEFT side of the marking
        std::vector<cv::Point_<T>> left_points;

        // the points thwere the laser hits ground zero on the RIGHT side of the marking
        std::vector<cv::Point_<T>> right_points;

        // start location for the 3 point vectors
        cv::Vec<T, 3> points_start;

        // the camera meta data
        std::unique_ptr<CameraData> camera_data = std::make_unique<CameraData>();

        //// the main camera pointer through the system
        //AVT::VmbAPI::CameraPtr cameraPtr;

        //// holding data to construct the actual camera
        //std::unique_ptr<VimbaData> vimbaData = std::make_unique<VimbaData>();

        //// the actual camera used
        //std::shared_ptr<GC2450MCamera> camera;

        // the name of the glob loaded (or "camera" for live feed)
        std::string glob_name;

        // the rectangle which includes the entire marking
        cv::Rect_<T> marking_rect;

        // left border vector
        cv::Vec<T, 4> left_border;

        // right border vector
        cv::Vec<T, 4> right_border;

        // the base lines where the laser hits the actual ground-zero
        cv::Vec<T, 4> base_lines;

        // the locations for where the base lines intersect with the marking border
        cv::Vec<T, 4> intersections;

        // enclusure of the laser line
        cv::Vec<T, 4> center_line;

        /**
         * \brief the points where the intersections are cut.
         * adjusted so potential unwanted information is not included in further calculations
         */
        cv::Vec<T, 2> intersection_cuts;

        // the x coordinates for the pieces that are cut out.
        cv::Vec<T, 4> middle_pieces;

        // validation results for the individual members of this structure
        std::vector<ValidationResult> validation_results;

        // overall validation status for this structure
        ValidationStatus status;

        // the avg
        T left_avg;

        T left_mid_avg;

        T center_avg;

        T right_mid_avg;

        T right_avg;

        // the difference between the baseline(s) and the laser line on the marking in sub-pixels
        T difference;

        static std::string DataMemberName(DataMemberIndex index) {
            switch (index) {
            case DataMemberIndex::CenterPoints:
                return "CenterPoints";
            case DataMemberIndex::LeftPoints:
                return "LeftPoints";
            case DataMemberIndex::RightPoints:
                return "RightPoints";
            case DataMemberIndex::PointsStart:
                return "PointsStart";
            case DataMemberIndex::CameraData:
                return "CameraData";
            case DataMemberIndex::CameraPtr:
                return "CameraPtr";
            case DataMemberIndex::GlobName:
                return "GlobName";
            case DataMemberIndex::MarkingRect:
                return "MarkingRect";
            case DataMemberIndex::LeftBorder:
                return "LeftBorder";
            case DataMemberIndex::RightBorder:
                return "RightBorder";
            case DataMemberIndex::BaseLines:
                return "BaseLines";
            case DataMemberIndex::Intersections:
                return "Intersections";
            case DataMemberIndex::IntersectionCuts:
                return "IntersectionCuts";
            case DataMemberIndex::MiddlePieces:
                return "MiddlePieces";
            case DataMemberIndex::LeftAvg:
                return "LeftAvg";
            case DataMemberIndex::CenterAvg:
                return "CenterAvg";
            case DataMemberIndex::RightAvg:
                return "RightAvg";
            case DataMemberIndex::Difference:
                return "Difference";
            default:
                return "feeeeek";
            }
        }

        void addValidationResult(DataMemberIndex index, bool ok, ValidationStatus validation, std::string information) {
            validation_results.emplace_back(ValidationResult(index, ok, validation, information));
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
            value_to_align = min_val;
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

    // ----------------------------------------------
    // -------------- output stuff ------------------
    // ----------------------------------------------

#ifndef _SYNC_OUT
#define _SYNC_OUT

    enum class SyncCout { IO_LOCK, IO_UNLOCK };

    std::ostream& operator<<(std::ostream&, SyncCout);

#define sync_cout std::cout << SyncCout::IO_LOCK
#define sync_endl std::endl << SyncCout::IO_UNLOCK

#endif

    enum class LogTime { LOG_TIME, LOG_DATE, LOG_TIME_DATE };

#ifndef _TIME_DATE_OUT
#define _TIME_DATE_OUT

    /**
     * \brief Overload of ostream to insert time and/or date beforehand
     * \return The processed ostream
     */
    std::ostream& operator<<(std::ostream&, LogTime);

#define log_timedate std::cout << LogTime::LOG_TIME_DATE
#define log_time std::cout << LogTime::LOG_TIME
#define log_date std::cout << LogTime::LOG_DATE

#endif

    // ----------------------------------------------

    // ----------------------------------------------
    // -------------- thread related ----------------
    // ----------------------------------------------

    template <typename T>
    void sleep(T ms) {
        static_assert(std::is_integral<T>::value, "Wrong type, must be integral.");
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }

    template <typename T1, typename T2>
    T2 iif(T1 expression, T2 true_part, T2 false_part) {
        static_assert(std::is_same<T1, bool>::value, "Only bool expression is accepted.");
        return expression ? true_part : false_part;
    }
}
