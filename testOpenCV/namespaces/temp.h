#pragma once
#include <xmmintrin.h>
#include <emmintrin.h>

// stuff made while the measurements ran

namespace temp {

    template <class T>
    struct binary_right_shift {
        T operator()(const T& x, const T& s) const {
            return x >> s;
        }

        typedef T argument_type;
        typedef T result_type;
    };

    template <class T>
    struct binary_left_shift {
        T operator()(const T& x, const T& s) const {
            return x << s;
        }

        typedef T argument_type;
        typedef T result_type;
    };

    template <class T>
    struct binary_and {
        T operator()(const T& x, const T& s) const {
            return x & s;
        }

        typedef T argument_type;
        typedef T result_type;
    };

    inline union __m128 floor_SIMD(const __m128& a) {
        static const auto one = _mm_set1_ps(1.0f);
        auto fval = _mm_cvtepi32_ps(_mm_cvttps_epi32(a));
        return _mm_sub_ps(fval, _mm_and_ps(_mm_cmplt_ps(a, fval), one));
    }

}
