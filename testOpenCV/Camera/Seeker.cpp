#include "Seeker.h"


Seeker::Seeker()
    : current_phase_(Phase::ONE) {
    phase_roi_[0] = def_phase_one_roi_;
}

Seeker::Seeker(capture_roi phase_one_roi)
    : current_phase_(Phase::ONE) {
    phase_roi_[0] = phase_one_roi;
}

bool Seeker::initialize() {

    // generate phase frameset pointers
    for (auto i = 0; i < frameset_.size(); i++)
        frameset_[i] = std::make_unique<Frames>(i);

    if (pcapture == nullptr) {
        // run the basic process for the capture object
        pcapture = std::make_unique<CapturePvApi>();
    } else {
        // double check for weirdness
        if (pcapture->is_open()) {
            pcapture->close();
        }
    }

    // always perform complete re-init.

    auto capture_device_ok = pcapture->initialize();

    if (!capture_device_ok) {
        log_time << "Capture device could not be initialized, aborting.\n";
        return false;
    }

    //initVideoCapture(); // for opencv capture object

    capture_device_ok = pcapture->open();

    if (!capture_device_ok) {
        log_time << "Capture device could not be openened, aborting.\n";
        pcapture->initialized(false);
        return false;
    }

    //capture->print_attr();

    pcapture->pixel_format();

    pcapture->reset_binning();

    pcapture->packet_size(8228);

    auto def_roi = cv::Rect_<unsigned long>(0, 1006, 2448, 256);

    pcapture->region(def_roi);

    pcapture->frame_init();

    pcapture->cap_init();

    pcapture->aquisition_init();

    for (auto& fs : frameset_) {
        pcapture->exposure(fs->exp_ms_);
        pcapture->cap(25, fs->frames_);
    }

}

bool Seeker::shut_down() {
    pcapture->aquisition_end();
    pcapture->cap_end();
    pcapture->close();
    pcapture->uninitialize();
    return true;
}

void Seeker::switch_phase() {
    switch (current_phase_) {
    case Phase::NONE:
        current_phase_ = Phase::ONE;
        break;
    case Phase::ONE:
        current_phase_ = Phase::TWO;
        break;
    case Phase::TWO:
        current_phase_ = Phase::THREE;
        break;
    case Phase::THREE:
        current_phase_ = Phase::DONE;
        break;
    case Phase::DONE:
        current_phase_ = Phase::NONE;
        break;
    default:
        current_phase_ = Phase::FAIL;
    }

}

int Seeker::frameset(Phase phase) {
    switch (phase) {
    case Phase::ONE:
        return 0;
        break;
    case Phase::TWO:
        return 1;
        break;
    case Phase::THREE:
        return 2;
        break;
    default:
        return -1;
    }
}
