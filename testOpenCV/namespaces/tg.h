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
#include "tg.h"

/**
 * \brief Contains utilitary functionality for the entire program
 */
namespace tg {

    //    typedef std::pair<cv::Point2f, cv::Point2f> linePair;

    enum Side { Left = 0, Right = 1, Center = 2 };

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
    //using VimbaData = struct vim {
    //    const char* pCameraID;

    //    const char* pCameraName;

    //    const char* pCameraModel;

    //    const char* pCameraSerialNumber;

    //    const char* pInterfaceID;

    //    VmbInterfaceType interfaceType;

    //    const char* pInterfaceName;

    //    const char* pInterfaceSerialNumber;

    //    VmbAccessModeType interfacePermittedAccess;
    //};

    /**
     * \brief PvApi camera structure
     */
    typedef struct {
        unsigned long UID;

        tPvHandle Handle;

        tPvFrame Frame;
    } tCamera;

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

    // ----------------------------------------------

    // ----------------------------------------------
    // -------------- timer related ----------------
    // ----------------------------------------------

    /**
     * Retrieves the current time as high resolution clock (ms)
     * @return The current time as time_point
     */
    inline long long int get_now_ms() {
        return std::chrono::high_resolution_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
    }

    /**
     * Retrieves the current time as high resolution clock (ns)
     * @return The current time as time_point
     */
    inline long long int get_now_ns() {
        return std::chrono::high_resolution_clock::now().time_since_epoch() / std::chrono::nanoseconds(1);
    }

    /**
     * Computes the difference in time from the parsed time and now
     * @tparam T The time type, must be arithmetic
     * @param t The time to substract from now
     * @return The difference between parsed time and now in ms
     */
    template <typename T>
    long long int diff_now_ms(T& t) {
        static_assert(std::is_arithmetic<T>::value, "feck, nan.");
        return get_now_ms() - t;
    }

    /**
     * Computes the difference in time from the parsed time and now in ns
     * @tparam T The time type, must be arithmetic
     * @param t The time to substract from now
     * @return The difference between parsed time and now in ns
     */
    template <typename T>
    long long int diff_now_ns(T& t) {
        static_assert(std::is_arithmetic<T>::value, "feck, nan.");
        return get_now_ms() - t;
    }

    // ----------------------------------------------

    /**
     * \brief Wrapped ternary expression handler, to allow for better typechecking etc
     * \tparam T1 expression type, bool
     * \tparam T2 true/false part type
     * \param expression The expression part
     * \param true_part if expression is true
     * \param false_part if expression is false
     * \return true_part if expression is true, otherwise false_part
     */
    template <typename T1, typename T2>
    T2 iif(T1 expression, T2 true_part, T2 false_part) {
        static_assert(std::is_same<T1, bool>::value, "Only bool expression is accepted.");
        return expression ? true_part : false_part;
    }
}
