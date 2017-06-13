#pragma once
#include <string>
#include <opencv2/core/types.hpp>
#include "validate.h"

#if defined(_MSC_VER) && !defined(inline)
#define inline __forceinline
#elif defined(__GNUC__) && !defined(inline)
#define inline __always_inline
#endif

// helper functions for open cv related stuff.
namespace cvr {

    std::string type2str(int type);

    /**
    * \brief Adjust the marking rectangle based on the intersection points and specified buffer
    * \param marking_rect The rectangle to adjust
    * \param intersection_points The intersection points
    * \param buffer The buffer to apply
    */
    template <typename T>
    inline
    void adjust_marking_rect(cv::Rect_<T>& marking_rect, cv::Vec<T, 4>& intersection_points, T buffer) {
        marking_rect.x = intersection_points[0] + buffer;
        marking_rect.width = intersection_points[2] - marking_rect.x - buffer;
    }

}
