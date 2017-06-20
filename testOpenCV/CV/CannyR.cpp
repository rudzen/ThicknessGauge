#include "CannyR.h"

#include "../namespaces/tg.h"

using namespace tg;

void CannyR::doCanny() {

    try {
        Canny(image_, edges_, threshold1_, threshold2_, apertureSize_, gradient_ > 0);

        if (removePepperNoise_)
            pixel::remove_pepper_noise(image_);

        if (showWindow_)
            imshow(windowName, edges_);
    } catch (cv::Exception& e) {
        log_time << "Caught exception in CannyR.\n";
    }

}

cv::Mat& CannyR::getResult() {
    return edges_;
}
