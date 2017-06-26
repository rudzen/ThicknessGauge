#pragma once
#include <vector>
#include <opencv2/core.hpp>
#include "../namespaces/tg.h"

/**
 * \brief Simple OpenCV glob generator.
 */
class GlobGenerator {

    cv::String pattern_;

    std::vector<cv::String> files_;

    bool recursive_;

    int count_;

    std::vector<cv::Mat> images_;

    int type_ = CV_8UC1;

    tg::GlobType glob_;

public:

    GlobGenerator();

    GlobGenerator(const std::string pattern, const bool recursive);

    explicit GlobGenerator(const std::string pattern)
        : pattern_(pattern)
        , recursive_(false) {
        generate_glob();
    }

    void generate_glob();

    void clear();

    /* getters */

    const std::vector<cv::Mat>& images() const;

    const std::vector<cv::String>& files() const;

    /* getters and setters */

    const cv::String& pattern() const;

    void pattern(const cv::String& pattern);

    const bool& recursive() const;

    void recursive(bool recursive);

    tg::GlobType glob() const;

    void glob(tg::GlobType glob);
};
