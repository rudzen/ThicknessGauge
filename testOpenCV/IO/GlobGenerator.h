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

    GlobGenerator()
        : recursive_(false)
        , count_(0)
        , glob_(tg::GlobType::Sequence) { }

    GlobGenerator(const std::string pattern, const bool recursive)
        : pattern_(pattern)
        , recursive_(recursive) {
        generate_glob();
    }

    explicit GlobGenerator(const std::string pattern)
        : pattern_(pattern)
        , recursive_(false) {
        generate_glob();
    }

    void generate_glob() {
        cv::glob(pattern_, files_, recursive_);
        for (auto& f : files_)
            images_.emplace_back(cv::imread(f, type_));
    }

    void clear() {
        images_.clear();
        files_.clear();
        pattern_.clear();
        recursive_ = false;
    }

    /* getters */

    const std::vector<cv::Mat>& images() const {
        return images_;
    }

    const std::vector<cv::String>& files() const {
        return files_;
    }

    /* getters and setters */

    const cv::String& pattern() const {
        return pattern_;
    }

    void pattern(const cv::String& pattern) {
        pattern_ = pattern;
    }

    const bool& recursive() const {
        return recursive_;
    }

    void recursive(bool recursive) {
        recursive_ = recursive;
    }

    tg::GlobType glob() const {
        return glob_;
    }

    void glob(tg::GlobType glob) {
        glob_ = glob;
    }
};
