#pragma once
#include <memory>
#include <array>
#include "CapturePvApi.h"
#include "CV/Frames.h"

/**
 * \brief Initiates the search of the marking rectangle.
 * ********************************************************************************************
 * Phase documentation:
 * 
 * Phase One - Marking rectangle search
 * ------------------------------------
 * 
 * 
 * 
 * ********************************************************************************************
 * By capturing a series of frames from the PvAPI capture wrapper class and
 * running the algorithm on those until it is satisfied with the results.
 * ********************************************************************************************
 * It then uses the resulting frame mean to determine how high and how low the next
 * phases should set their exposures.
 * ********************************************************************************************
 * It also contains the roi for each phase, which is automaticly by the phase initiator.
 * Each phase (except the first) is computed by the previous phase as they progress,
 * This has the following consequences :
 * - If a phase fails, the next phase will not proceed with failed attempts, since the roi is invalid.
 * 
 * 
 * * before the foll
 */
class Seeker {

public:

    enum class Phase {
        ONE, TWO, THREE, DONE, NONE, FAIL
    };

private:

    using ulong = unsigned long;

    using capture_roi = cv::Rect_<ulong>;

    using phase_one_exp = struct phase_exp {
        const ulong exposure_start = 500;
        const ulong exposure_end = 30000;
        const ulong exposure_increment = 500;
    };

    std::shared_ptr<CapturePvApi> pcapture = std::make_shared<CapturePvApi>();

    const capture_roi def_phase_one_roi_ = capture_roi(0UL, 1006, 2448, 256);

    const capture_roi phase_null_ = capture_roi(0UL, 0UL, 0UL, 0UL);

    Phase current_phase_;

    // roi for the different phases (3)
    // note that only phase one is known from the beginning
    std::array<capture_roi, 3> phase_roi_{
        def_phase_one_roi_,
        phase_null_,
        phase_null_
    };

    std::array<ulong, 3> exposures_ = {5000, 20000, 40000};

    std::array<std::string, 3> exposures_short_ = {"_5k", "_20k", "_40k"};

    std::array<std::unique_ptr<Frames>, 3> frameset_;

    Seeker();

    explicit Seeker(capture_roi phase_one_roi);

    /**
     * \brief Initializes all sekker class data members
     */
    bool initialize();

    bool shut_down();

private: // internal functions

    void switch_phase();

    void process_phase(ulong set_index);

    int frameset(Phase phase);

};
