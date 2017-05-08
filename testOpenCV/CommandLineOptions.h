#pragma once
#include <ostream>

class CommandLineOptions {
	
public:


	CommandLineOptions(const bool buildInfoMode, const bool testMode, const bool demoMode, const bool calibrationMode, const bool showWindows, const bool recordVideo, const std::string cameraFile, const std::string calibrationOutput, const int frames, const int testMax, const int testInterval, const int numOpenCVThreads)
		: buildInfoMode_(buildInfoMode),
		  testMode_(testMode),
		  demoMode_(demoMode),
		  calibrationMode_(calibrationMode),
		  showWindows_(showWindows),
		  recordVideo_(recordVideo),
		  cameraFile_(cameraFile),
		  calibrationOutput_(calibrationOutput),
		  frames_(frames),
		  testMax_(testMax),
		  testInterval_(testInterval),
		  numOpenCVThreads_(numOpenCVThreads) {
	}

	CommandLineOptions(): buildInfoMode_(false), testMode_(false), demoMode_(true), calibrationMode_(false), showWindows_(true), recordVideo_(false), frames_(25), testMax_(0), testInterval_(0), numOpenCVThreads_(4) {
	}

	friend bool operator==(const CommandLineOptions& lhs, const CommandLineOptions& rhs) {
		return lhs.buildInfoMode_ == rhs.buildInfoMode_
			&& lhs.testMode_ == rhs.testMode_
			&& lhs.demoMode_ == rhs.demoMode_
			&& lhs.calibrationMode_ == rhs.calibrationMode_
			&& lhs.showWindows_ == rhs.showWindows_
			&& lhs.recordVideo_ == rhs.recordVideo_
			&& lhs.cameraFile_ == rhs.cameraFile_
			&& lhs.calibrationOutput_ == rhs.calibrationOutput_
			&& lhs.testSuite_ == rhs.testSuite_
			&& lhs.frames_ == rhs.frames_
			&& lhs.testMax_ == rhs.testMax_
			&& lhs.testInterval_ == rhs.testInterval_
			&& lhs.numOpenCVThreads_ == rhs.numOpenCVThreads_;
	}

	friend bool operator!=(const CommandLineOptions& lhs, const CommandLineOptions& rhs) {
		return !(lhs == rhs);
	}


	friend std::size_t hash_value(const CommandLineOptions& obj) {
		std::size_t seed = 0x1C34AD51;
		seed ^= (seed << 6) + (seed >> 2) + 0x7BEF55B5 + static_cast<std::size_t>(obj.buildInfoMode_);
		seed ^= (seed << 6) + (seed >> 2) + 0x374B74F9 + static_cast<std::size_t>(obj.testMode_);
		seed ^= (seed << 6) + (seed >> 2) + 0x776E5645 + static_cast<std::size_t>(obj.demoMode_);
		seed ^= (seed << 6) + (seed >> 2) + 0x3552D071 + static_cast<std::size_t>(obj.calibrationMode_);
		seed ^= (seed << 6) + (seed >> 2) + 0x72B298C4 + static_cast<std::size_t>(obj.showWindows_);
		seed ^= (seed << 6) + (seed >> 2) + 0x1B22BAF2 + static_cast<std::size_t>(obj.recordVideo_);
		seed ^= (seed << 6) + (seed >> 2) + 0x2AD90237 + obj.cameraFile_.size();
		seed ^= (seed << 6) + (seed >> 2) + 0x5BF7AD8C + obj.calibrationOutput_.size();
		seed ^= (seed << 6) + (seed >> 2) + 0x7B8590B3 + obj.testSuite_.size();
		seed ^= (seed << 6) + (seed >> 2) + 0x35795544 + static_cast<std::size_t>(obj.frames_);
		seed ^= (seed << 6) + (seed >> 2) + 0x096C9A55 + static_cast<std::size_t>(obj.testMax_);
		seed ^= (seed << 6) + (seed >> 2) + 0x734629BD + static_cast<std::size_t>(obj.testInterval_);
		seed ^= (seed << 6) + (seed >> 2) + 0x734629BD + static_cast<std::size_t>(obj.numOpenCVThreads_);
		return seed;
	}


	friend std::ostream& operator<<(std::ostream& os, const CommandLineOptions& obj) {
		return os
			<< " demoMode_: " << obj.demoMode_
			<< "\nbuildInfoMode: " << obj.buildInfoMode_
			<< "\ntestMode: " << obj.testMode_
			<< "\ntestSuite: " << obj.testSuite_
			<< "\ncalibrationMode_: " << obj.calibrationMode_
			<< "\ncalibrationOutput: " << obj.calibrationOutput_
			<< "\ncameraFile: " << obj.cameraFile_
			<< "\nshowWindows_: " << obj.showWindows_
			<< "\nrecordVideo_: " << obj.recordVideo_
			<< "\nframes: " << obj.frames_
			<< "\ntestMax: " << obj.testMax_
			<< "\ntestInterval: " << obj.testInterval_
			<< "\nnumOpenCVThreads_: " << obj.numOpenCVThreads_;
	}

private:
	bool buildInfoMode_;
	bool testMode_;
	bool demoMode_;
	bool calibrationMode_;

	bool showWindows_;
	bool recordVideo_;

	std::string cameraFile_;
	std::string calibrationOutput_;
	std::string testSuite_;

	int frames_;
	int testMax_;
	int testInterval_;

	int numOpenCVThreads_;


public:


	const int& getNumOpenCvThreads() const {
		return numOpenCVThreads_;
	}

	void setNumOpenCvThreads(int numOpenCvThreads) {
		numOpenCVThreads_ = numOpenCvThreads;
	}

	const std::string& getTestSuite() const {
		return testSuite_;
	}

	void setTestSuite(const std::string& testSuite) {
		testSuite_ = testSuite;
	}

	const std::string& getCalibrationOutput() const {
		return calibrationOutput_;
	}

	void setCalibrationOutput(const std::string& calibrationOutput) {
		calibrationOutput_ = calibrationOutput;
	}

	const bool& isBuildInfoMode() const {
		return buildInfoMode_;
	}

	void setBuildInfoMode(bool buildInfoMode) {
		buildInfoMode_ = buildInfoMode;
	}

	const bool& isTestMode() const {
		return testMode_;
	}

	void setTestMode(bool testMode) {
		testMode_ = testMode;
	}

	const bool& isDemoMode() const {
		return demoMode_;
	}

	void setDemoMode(bool demoMode) {
		demoMode_ = demoMode;
	}

	const bool& isCalibrationMode() const {
		return calibrationMode_;
	}

	void setCalibrationMode(bool calibrationMode) {
		calibrationMode_ = calibrationMode;
	}

	const bool& isShowWindows() const {
		return showWindows_;
	}

	void setShowWindows(bool showWindows) {
		showWindows_ = showWindows;
	}

	const bool& isRecordVideo() const {
		return recordVideo_;
	}

	void setRecordVideo(bool recordVideo) {
		recordVideo_ = recordVideo;
	}

	const std::string& getCameraFile() const {
		return cameraFile_;
	}

	void setCameraFile(const std::string& cameraFile) {
		cameraFile_ = cameraFile;
	}

	const int& getFrames() const {
		return frames_;
	}

	void setFrames(int frames) {
		frames_ = frames;
	}

	const int& getTestMax() const {
		return testMax_;
	}

	void setTestMax(int testMax) {
		testMax_ = testMax;
	}

	const int& getTestInterval() const {
		return testInterval_;
	}

	void setTestInterval(int testInterval) {
		testInterval_ = testInterval;
	}
};
