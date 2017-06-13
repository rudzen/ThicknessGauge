#pragma once
#include <string>
#include <opencv2/core/types.hpp>

#if defined(_MSC_VER) && !defined(inline)
#define inline __forceinline
#elif defined(__GNUC__) && !defined(inline)
#define inline __always_inline
#endif

// helper functions for open cv
namespace cvr {

    std::string type2str(int type);

}
