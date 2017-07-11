#pragma once
#include <memory>
#include <array>
#include "CapturePvApi.h"
#include "CV/Frames.h"
#include "CV/CannyR.h"
#include "CV/FilterR.h"
#include "CV/MorphR.h"
#include "CV/HoughLinesPR.h"

/**
 * * NOT COMPLETE YET *
 *
 * \brief Initiates the search of the marking rectangle.
 * ********************************************************************************************
 * By capturing a series of frames from the PvAPI capture wrapper class and
 * running the algorithm on those until it is satisfied with the results.
 * This documentation is part of the whole documentation for the software package.
 * ********************************************************************************************
 * ********************************************************************************************
 * Phase documentation:
 * 
 * Phase One - Marking rectangle search
 * --------------------------------------
 * The algorithm will auto-detect the marking rectangle by progressing frames with increasingly
 * longer exposure values, grabbing a single frame for each setting. Each frame recieved is
 * processed through a diagonal image folding matrix before edge detection (canny) is utilized.
 * The resulting data is then processed with hough lines where it only focuses on lines within a specific
 * angle in an attempt to localize the borders of the marking.
 * It does so by splitting the located lines in two sets, one for each side of the entirety
 * of the frame. For each side, it groups the lines into properbilities by taking one line and
 * checking if it intersects with all other lines on the same side, a bit like hough algorithm,
 * but on the results of the hough algorithm instead of the frame it self. If it fails, it will retry
 * the process by attempting to remove any pepper noise from the frame (small un-closed islands
 * of pixels which increases the chances of finding a line somewhere unwanted). If it still fails,
 * it will repeat the process on a frame taken with 25% exposure step reduction and continue up to
 * four times until it reports back with a failure.
 * If it finally detects two border areas, it creates a rectangle that hold the entirety of both
 * sides line areas. Finally it will generate three new region of interests areas based on the
 * found rectangle which will be needed in later phases.
 * 
 * Phase Two - Base line search
 * --------------------------------------
 * This phase consist of two sub phases, one for each side. It begins by generating yet another
 * two sub regions, one for each side. Then the exposure is increased by four in comparison with
 * Phase one exposure, because the laser target in the roi is with high (> 90%) properbility much
 * weaker in intensity than the desired intensity to be able to detect it with a high degree of certainty.
 * The frame processing steps are almost the same as phase one, except that it adds a morphology filter
 * between the edge detection and hough line detection. Because of the difficulty in detecting the line,
 * it utilizes the morphology filter to "fill" minor gabs in the edged detected, which are caused by the
 * low intensity. The configuration of the hough line algorithm is different than the one used in phase one,
 * mainly because it has to treat small gabs as if they are part of the "line" it detects. This greatly
 * increases the chance of it finding the line that fits inside the "real" locality of the laser beam.
 * In other words, the hough is relaxed because it knows that the laser is actually a line and not
 * many small spots, so it treats them as such. Once it has detected lines, it will generate a region
 * where the lines are located, which contains the locality of the laser beam.
 * At this point, it will then slice that sub region into sub regions which are just a single pixel
 * wide. These small sub regions are then analyzed and the intensity of the pixels within are used
 * to compute the "height" within that sub region based on the mass distribution of the intensity levels.
 * 
 * This process is done on both sides.
 * If no laser is detected, the process is aborted and restartet with phase one again.
 * The amount of frames used in this phase is never lower than 5, or higher than 25.
 * 
 * Phase Three - The laser on the marking
 * --------------------------------------
 * Based on the markting rectangle detected in phase one, the algorithm will then proceed and
 * try to detect the laser beam in the frame. It does so by same frame capture procedure as
 * phase one and the same line detection in phase two. Once it has an area which is believes is
 * the laser beam, it will proceed with the same mass intensity adjustment as phase two.
 * Finally the difference between the baselines (avg of both side) and the laser on the marking
 * is computed and the result is kept.
 * 
 * In the process of all three phases all data is kept in a seperate data container.
 * This container easily be extracted to another part of the software there the numbers can be
 * manipulated further with respect to their transformation based on a specific camera calibration matrix.
 * 
 * General Phase Information
 * --------------------------------------
 * Once phase two and phase three has achived a useable exposure, it will capture a series of frames.
 * The computations are all done on all the frames and the average is then used as the result.
 * This is because the light can slightly alterate, and by using a series of frames, the result will be
 * more precise as it adds a degree of "noise" to the average result.
 * 
 * ********************************************************************************************
 * ********************************************************************************************
 * ********************************************************************************************
 * 
 * The class it self
 * --------------------------------------
 * - Each phase has their own dataset which is computed and retrieved when needed.
 *   Not necessarily by the active phase.
 * - Configured as a tiny state machine.
 * - Automatic detection of failures, and when to abort etc. etc.
 * - When a phase is completed, it will initiate mean and std dev computation in parallel for the current frames in the phase.
 * - The changing of the camera ROI is GREATLY increasing the performance, since the process of capturing
 *   larger frames and the process of manually cutting the region from the full frame is completely gone.
 * - When all phases are complete the process is halted until the mean/std calculations are done.
 * 
 * It then uses the resulting frame mean to determine how high and how low the next
 * phases should set their exposures.
 * ********************************************************************************************
 * It also contains the roi for each phase, which is automaticly by the phase initiator.
 * Each phase roi (except the first) is computed by the previous phase as they progress,
 * This has the following consequences :
 * - If a phase fails, the next phase will not proceed with failed attempts, since the roi is invalid.
 * 
 */


constexpr int DEF_NEAR_EXTRACT = 5;

class Seeker : public std::enable_shared_from_this<Seeker> {

public:

    enum class Phase {
        ONE, TWO_RIGHT, TWO_LEFT, THREE, DONE, NONE, FAIL
    };

private:

    using ulong = unsigned long;

    using capture_roi = cv::Rect_<ulong>;

    /**
     * \brief Exposure seek configuration.
     * Start is the initial value set.
     * End is the upper limit
     * Increment is the amount to increment with for each "try"
     */
    using phase_one_exp = struct phase_exp {
        const ulong exposure_start = 1000;
        const ulong exposure_end = 30000;
        const ulong exposure_increment = 1000;
    };

    std::unique_ptr<phase_one_exp> exposure_levels = std::make_unique<phase_one_exp>();

    using phase_two_results = struct results {
        ulong ok; // <- important.. comparison to other side
        ulong fail; // <- important.. determine high fail chance
    };

    ulong phase_one_exposure = exposure_levels->exposure_start;

    ulong phase_two_base_exposure_ = 0;

    std::shared_ptr<CapturePvApi> pcapture = std::make_shared<CapturePvApi>();

    // common canny with default settings for detecting marking borders
    std::unique_ptr<CannyR> pcanny = std::make_unique<CannyR>(130, 200, 3, true, false, false);

    // filter used for base line detection
    std::unique_ptr<FilterR> pfilter = std::make_unique<FilterR>("Baseline filter");

    // morph for phase two and three
    std::unique_ptr<MorphR> pmorph = std::make_unique<MorphR>(cv::MORPH_GRADIENT, 1, false);

    std::shared_ptr<Data<double>> pdata = std::make_shared<Data<double>>();

public: // data return point
    template <typename T>
    std::shared_ptr<Data<T>> data() const {
        return pdata;
    }

    std::shared_ptr<Seeker> get_ptr() {
        return shared_from_this();
    }

private:

    const capture_roi def_phase_one_roi_ = capture_roi(0UL, 1006UL, 2448UL, 256UL);

    const capture_roi phase_roi_null_ = capture_roi(0UL, 0UL, 0UL, 0UL);

    const capture_roi buffer_clear_roi = capture_roi(1UL, 1UL, 1UL, 1UL);

    Phase current_phase_;

    // roi for the different phases (3)
    // note that only phase one is known from the beginning
    // 0 = default phase one frame (used to get offsets)
    // 1 = phase two left side
    // 2 = phase two right side
    // 3 = phase three
    std::array<capture_roi, 4> phase_roi_{
        def_phase_one_roi_,
        phase_roi_null_,
        phase_roi_null_,
        phase_roi_null_
    };

    std::array<ulong, 3> exposures_ = {5000, 20000, 40000};

    std::array<std::string, 3> exposures_short_ = {"_5k", "_20k", "_40k"};

    std::array<std::unique_ptr<Frames>, 4> frameset_;

    Frames* current_frameset_;

    bool shut_down() const;

private: // internal functions

    template <int upper>
    ulong phase_roi_y() {
        ulong return_value = 0;
        std::for_each(phase_roi_.begin(), phase_roi_.begin() + upper, [&] (const capture_roi& roi) {
                      return_value += roi.y;
                  });
        return return_value;
    }

    template <typename T, int index>
    bool phase_roi(cv::Rect_<T> roi) {
        phase_roi_[index].x = static_cast<unsigned long>(floor(roi.x));
        phase_roi_[index].y = static_cast<unsigned long>(floor(roi.y));
        phase_roi_[index].width = static_cast<unsigned long>(ceil(roi.width));
        phase_roi_[index].height = static_cast<unsigned long>(ceil(roi.height));
        return validate::validate_rect(phase_roi_[index]);
    }

    /**
     * \brief Processes the matrix for optimal output and computes the line information based on the results
     * \param org The matrix to perform the process on
     * \param hough The hough extension class used
     * \param morph The morphology extenstion class used
     */
    void process_mat_for_line(cv::Mat& org, std::shared_ptr<HoughLinesPR>& hough, MorphR* morph) const;

    void switch_phase();

    bool phase_one();

    bool phase_two_left();

    bool phase_two_right();
    bool phase_two_line();

    bool phase_three();

    double phase_finalize();

    static int frameset(Phase phase);

    /**
    * \brief Initializes all sekker class data members
    */
    bool initialize();


public:

    Seeker();

    explicit Seeker(capture_roi phase_one_roi);

    bool compute(bool do_null, cv::Rect_<unsigned long>& marking_rect, unsigned long p2_base_exposure);

};
