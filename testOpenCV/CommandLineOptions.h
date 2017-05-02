#pragma once

class CommandLineOptions {
	
private:

	// modes (xor : build -> test -> calib -> demo
	bool buildInfoMode;
	bool testMode;
	bool demoMode;
	bool calibrationMode;
	bool showWindows;

	bool recordVideo;

	std::string cameraFile;
	std::string calibrationOutput;

private:
	int frames;
	int testMax;
	int testInterval;


public:

	const std::string& CalibrationOutput() const {
		return calibrationOutput;
	}

	void setCalibrationOutput(const std::string& calibrationOutput) {
		this->calibrationOutput = calibrationOutput;
	}

	const bool& BuildInfoMode() const {
		return buildInfoMode;
	}

	void setBuildInfoMode(bool buildInfoMode) {
		this->buildInfoMode = buildInfoMode;
	}

	const bool& TestMode() const {
		return testMode;
	}

	void setTestMode(bool testMode) {
		this->testMode = testMode;
	}

	const bool& DemoMode() const {
		return demoMode;
	}

	void setDemoMode(bool demoMode) {
		this->demoMode = demoMode;
	}

	const bool& CalibrationMode() const {
		return calibrationMode;
	}

	void setCalibrationMode(bool calibrationMode) {
		this->calibrationMode = calibrationMode;
	}

	const bool& ShowWindows() const {
		return showWindows;
	}

	void setShowWindows(bool showWindows) {
		this->showWindows = showWindows;
	}

	const bool& RecordVideo() const {
		return recordVideo;
	}

	void setRecordVideo(bool recordVideo) {
		this->recordVideo = recordVideo;
	}

	const std::string& CameraFile() const {
		return cameraFile;
	}

	void setCameraFile(const std::string& cameraFile) {
		this->cameraFile = cameraFile;
	}

	const int& Frames() const {
		return frames;
	}

	void set_frames(int frames) {
		this->frames = frames;
	}

	const int& TestMax() const {
		return testMax;
	}

	void setTestMax(int testMax) {
		this->testMax = testMax;
	}

	const int& TestInterval() const {
		return testInterval;
	}

	void setTestInterval(int testInterval) {
		this->testInterval = testInterval;
	}
};