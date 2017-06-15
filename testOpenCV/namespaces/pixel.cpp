
//          Copyright Rudy Alex Kohn 2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include "pixel.h"

namespace pixel {

    void remove_pepper_noise(cv::Mat& mask) {
        // For simplicity, ignore the top & bottom row border.
        for (auto y = 2; y < mask.rows - 2; y++) {
            // Get access to each of the 5 rows near this pixel.
            auto p_this = mask.ptr(y);
            auto p_up1 = mask.ptr(y - 1);
            auto p_up2 = mask.ptr(y - 2);
            auto p_down1 = mask.ptr(y + 1);
            auto p_down2 = mask.ptr(y + 2);

            // For simplicity, ignore the left & right row border.
            p_this += 2;
            p_up1 += 2;
            p_up2 += 2;
            p_down1 += 2;
            p_down2 += 2;
            for (auto x = 2; x < mask.cols - 2; x++) {
                auto v = *p_this; // Get the current pixel value (either 0 or 255).
                // If the current pixel is black, but all the pixels on the 2-pixel-radius-border are white
                // (ie: it is a small island of black pixels, surrounded by white), then delete that island.
                if (v == 0) {
                    auto all_above = *(p_up2 - 2) && *(p_up2 - 1) && *(p_up2) && *(p_up2 + 1) && *(p_up2 + 2);
                    auto all_left = *(p_up1 - 2) && *(p_this - 2) && *(p_down1 - 2);
                    auto all_below = *(p_down2 - 2) && *(p_down2 - 1) && *(p_down2) && *(p_down2 + 1) && *(p_down2 + 2);
                    auto all_right /*yes, everything is alright*/ = *(p_up1 + 2) && *(p_this + 2) && *(p_down1 + 2);
                    auto surroundings = all_above && all_left && all_below && all_right;
                    if (surroundings) {
                        // Fill the whole 5x5 block as white. Since we know the 5x5 borders
                        // are already white, just need to fill the 3x3 inner region.
                        *(p_up1 - 1) = 255;
                        *(p_up1 + 0) = 255;
                        *(p_up1 + 1) = 255;
                        *(p_this - 1) = 255;
                        *(p_this + 0) = 255;
                        *(p_this + 1) = 255;
                        *(p_down1 - 1) = 255;
                        *(p_down1 + 0) = 255;
                        *(p_down1 + 1) = 255;
                    }
                    // Since we just covered the whole 5x5 block with white, we know the next 2 pixels
                    // won't be black, so skip the next 2 pixels on the right.
                    p_this += 2;
                    p_up1 += 2;
                    p_up2 += 2;
                    p_down1 += 2;
                    p_down2 += 2;
                }
                // Move to the next pixel.
                p_this++;
                p_up1++;
                p_up2++;
                p_down1++;
                p_down2++;
            }
        }
    }
}
