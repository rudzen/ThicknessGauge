#include "GlobGenerator.h"

GlobGenerator::GlobGenerator()
    : recursive_(false)
    , count_(0)
    , glob_(tg::GlobType::Sequence) { }

GlobGenerator::GlobGenerator(const std::string pattern, const bool recursive)
    : pattern_(pattern)
    , recursive_(recursive) {
    generate_glob();
}

GlobGenerator::GlobGenerator(const std::string pattern)
    : pattern_(pattern)
    , recursive_(false) {
    generate_glob();
}

void GlobGenerator::generate_glob() {
    cv::glob(pattern_, files_, recursive_);
    for (auto& f : files_)
        images_.emplace_back(cv::imread(f, type_));
}

void GlobGenerator::clear() {
    images_.clear();
    files_.clear();
    pattern_.clear();
    recursive_ = false;
}

const std::vector<cv::Mat>& GlobGenerator::images() const {
    return images_;
}

const std::vector<cv::String>& GlobGenerator::files() const {
    return files_;
}

const cv::String& GlobGenerator::pattern() const {
    return pattern_;
}

void GlobGenerator::pattern(const cv::String& pattern) {
    pattern_ = pattern;
}

const bool& GlobGenerator::recursive() const {
    return recursive_;
}

void GlobGenerator::recursive(bool recursive) {
    recursive_ = recursive;
}

tg::GlobType GlobGenerator::glob() const {
    return glob_;
}

void GlobGenerator::glob(tg::GlobType glob) {
    glob_ = glob;
}
