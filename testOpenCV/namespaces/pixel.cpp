
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
            auto pThis = mask.ptr(y);
            auto pUp1 = mask.ptr(y - 1);
            auto pUp2 = mask.ptr(y - 2);
            auto pDown1 = mask.ptr(y + 1);
            auto pDown2 = mask.ptr(y + 2);

            // For simplicity, ignore the left & right row border.
            pThis += 2;
            pUp1 += 2;
            pUp2 += 2;
            pDown1 += 2;
            pDown2 += 2;
            for (auto x = 2; x < mask.cols - 2; x++) {
                auto v = *pThis; // Get the current pixel value (either 0 or 255).
                // If the current pixel is black, but all the pixels on the 2-pixel-radius-border are white
                // (ie: it is a small island of black pixels, surrounded by white), then delete that island.
                if (v == 0) {
                    auto allAbove = *(pUp2 - 2) && *(pUp2 - 1) && *(pUp2) && *(pUp2 + 1) && *(pUp2 + 2);
                    auto allLeft = *(pUp1 - 2) && *(pThis - 2) && *(pDown1 - 2);
                    auto allBelow = *(pDown2 - 2) && *(pDown2 - 1) && *(pDown2) && *(pDown2 + 1) && *(pDown2 + 2);
                    auto allRight = *(pUp1 + 2) && *(pThis + 2) && *(pDown1 + 2);
                    auto surroundings = allAbove && allLeft && allBelow && allRight;
                    if (surroundings) {
                        // Fill the whole 5x5 block as white. Since we know the 5x5 borders
                        // are already white, just need to fill the 3x3 inner region.
                        *(pUp1 - 1) = 255;
                        *(pUp1 + 0) = 255;
                        *(pUp1 + 1) = 255;
                        *(pThis - 1) = 255;
                        *(pThis + 0) = 255;
                        *(pThis + 1) = 255;
                        *(pDown1 - 1) = 255;
                        *(pDown1 + 0) = 255;
                        *(pDown1 + 1) = 255;
                    }
                    // Since we just covered the whole 5x5 block with white, we know the next 2 pixels
                    // won't be black, so skip the next 2 pixels on the right.
                    pThis += 2;
                    pUp1 += 2;
                    pUp2 += 2;
                    pDown1 += 2;
                    pDown2 += 2;
                }
                // Move to the next pixel.
                pThis++;
                pUp1++;
                pUp2++;
                pDown1++;
                pDown2++;
            }
        }
    }
}
