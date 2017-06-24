#include "CannyR.h"

#include "../namespaces/tg.h"

using namespace tg;

void CannyR::do_canny() {
    try {
        Canny(image_, edges_, threshold_1_, threshold_2_, aperture_size_, gradient_ > 0);

        if (remove_pepper_noise_)
            pixel::remove_pepper_noise(image_);

        if (show_windows_)
            imshow(window_name_, edges_);
    } catch (cv::Exception& e) {
        log_time << "Caught exception in CannyR.\n" << e.what();
    }

}

cv::Mat& CannyR::result() {
    return edges_;
}
