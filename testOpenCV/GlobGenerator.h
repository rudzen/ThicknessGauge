#pragma once
#include <vector>
#include <opencv2/core.hpp>

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

public:

	GlobGenerator(const std::string pattern, const bool recursive)
		: pattern_(pattern),
		  recursive_(recursive) {
		generateGlob();
	}

	explicit GlobGenerator(const std::string pattern)
		: pattern_(pattern),
		recursive_(false) {
		generateGlob();
	}

	void generateGlob() {
		cv::glob(pattern_, files_, recursive_);
		for (auto& f : files_)
			images_.push_back(cv::imread(f, type_));
	}

	void clear() {
		images_.clear();
		files_.clear();
		pattern_.clear();
		recursive_ = false;
	}

	const std::vector<cv::Mat>& getImages() const {
		return images_;
	}

	const cv::String& getPattern() const {
		return pattern_;
	}

	void setPattern(const cv::String& pattern) {
		pattern_ = pattern;
	}

	const std::vector<cv::String>& getFiles() const {
		return files_;
	}

	const bool& isRecursive() const {
		return recursive_;
	}

	void setRecursive(bool recursive) {
		recursive_ = recursive;
	}
};
