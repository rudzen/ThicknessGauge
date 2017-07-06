#pragma once
#include <ostream>
#include <opencv2/core/types.hpp>

class CommandLineOptions {

public:

    CommandLineOptions(const bool buildInfoMode, const bool testMode, const bool demoMode, const bool calibrationMode, const bool globMode, const bool zero_measuring,  const bool showWindows, const bool recordVideo, const std::string cameraFile, const std::string calibrationOutput, const int frames, const int testMax, const int testInterval, const int numOpenCVThreads)
        : build_info_mode_(buildInfoMode),
          test_mode_(testMode),
          demo_mode_(demoMode),
          calibration_mode_(calibrationMode),
          glob_mode_(globMode),
          zero_measurering_(zero_measuring),
          show_windows_(showWindows),
          record_video_(recordVideo),
          camera_file_(cameraFile),
          calibration_output_(calibrationOutput),
          frames_(frames),
          test_max_(testMax),
          test_interval_(testInterval),
          num_open_cv_threads_(numOpenCVThreads) { }

    CommandLineOptions()
        : build_info_mode_(false), test_mode_(false), demo_mode_(true), calibration_mode_(false), glob_mode_(false), zero_measurering_(false), show_windows_(true), record_video_(false), frames_(25), test_max_(0), test_interval_(0), num_open_cv_threads_(4) { }

    friend bool operator==(const CommandLineOptions& lhs, const CommandLineOptions& rhs) {
        return lhs.build_info_mode_ == rhs.build_info_mode_
            && lhs.test_mode_ == rhs.test_mode_
            && lhs.demo_mode_ == rhs.demo_mode_
            && lhs.calibration_mode_ == rhs.calibration_mode_
            && lhs.show_windows_ == rhs.show_windows_
            && lhs.record_video_ == rhs.record_video_
            && lhs.camera_file_ == rhs.camera_file_
            && lhs.glob_folder_ == rhs.glob_folder_
            && lhs.calibration_output_ == rhs.calibration_output_
            && lhs.test_suite_ == rhs.test_suite_
            && lhs.frames_ == rhs.frames_
            && lhs.test_max_ == rhs.test_max_
            && lhs.test_interval_ == rhs.test_interval_
            && lhs.num_open_cv_threads_ == rhs.num_open_cv_threads_;
    }

    friend bool operator!=(const CommandLineOptions& lhs, const CommandLineOptions& rhs) {
        return !(lhs == rhs);
    }

    friend std::ostream& operator<<(std::ostream& os, const CommandLineOptions& obj) {
        return os
            << " demoMode_: " << obj.demo_mode_
            << "\nbuildInfoMode: " << obj.build_info_mode_
            << "\ntestMode: " << obj.test_mode_
            << "\ntestSuite: " << obj.test_suite_
            << "\ncalibrationMode_: " << obj.calibration_mode_
            << "\ncalibrationOutput: " << obj.calibration_output_
            << "\nglobMode_: " << obj.glob_mode_
            << "\nglobFolder_: " << obj.glob_folder_
            << "\ncameraFile: " << obj.camera_file_
            << "\nshowWindows_: " << obj.show_windows_
            << "\nrecordVideo_: " << obj.record_video_
            << "\nframes: " << obj.frames_
            << "\ntestMax: " << obj.test_max_
            << "\ntestInterval: " << obj.test_interval_
            << "\nnumOpenCVThreads_: " << obj.num_open_cv_threads_;
    }

private:

    cv::Rect_<unsigned long> zero_marking_values_;

    bool build_info_mode_;
    bool test_mode_;
    bool demo_mode_;
    bool calibration_mode_;
    bool glob_mode_;

    bool zero_measurering_;
    bool show_windows_;
    bool record_video_;

    std::string camera_file_;
    std::string calibration_output_;
    std::string test_suite_;
    std::string glob_folder_;

    int frames_;
    int test_max_;
    int test_interval_;

    int num_open_cv_threads_;

public:


    bool zero_measurering() const {
        return zero_measurering_;
    }

    void zero_measurering(bool zero_measurering) {
        zero_measurering_ = zero_measurering;
    }

    cv::Rect_<unsigned long> zero_marking_values() const {
        return zero_marking_values_;
    }

    void zero_marking_values(cv::Rect_<unsigned long> zero_marking_values) {
        zero_marking_values_ = zero_marking_values;
    }

    const int& num_open_cv_threads() const {
        return num_open_cv_threads_;
    }

    void num_open_cv_threads(int numOpenCvThreads) {
        num_open_cv_threads_ = numOpenCvThreads;
    }

    const std::string& test_suite() const {
        return test_suite_;
    }

    void test_suite(const std::string& testSuite) {
        test_suite_ = testSuite;
    }

    const std::string& calibration_output() const {
        return calibration_output_;
    }

    void calibration_output(const std::string& calibrationOutput) {
        calibration_output_ = calibrationOutput;
    }

    const bool& build_info_mode() const {
        return build_info_mode_;
    }

    void build_info_mode(bool buildInfoMode) {
        build_info_mode_ = buildInfoMode;
    }

    const bool& test_mode() const {
        return test_mode_;
    }

    void test_mode(bool testMode) {
        test_mode_ = testMode;
    }

    const bool& demo_mode() const {
        return demo_mode_;
    }

    void demo_mode(bool demoMode) {
        demo_mode_ = demoMode;
    }

    const bool& calibration_mode() const {
        return calibration_mode_;
    }

    void calibration_mode(bool calibrationMode) {
        calibration_mode_ = calibrationMode;
    }

    bool glob_mode() const {
        return glob_mode_;
    }

    void glob_mode(bool globMode) {
        glob_mode_ = globMode;
    }

    const std::string& glob_folder() const {
        return glob_folder_;
    }

    void glob_folder(const std::string& globFolder) {
        glob_folder_ = globFolder;
    }

    const bool& show_windows() const {
        return show_windows_;
    }

    void show_windows(bool showWindows) {
        show_windows_ = showWindows;
    }

    const bool& record_video() const {
        return record_video_;
    }

    void record_video(bool recordVideo) {
        record_video_ = recordVideo;
    }

    const std::string& camera_file() const {
        return camera_file_;
    }

    void camera_file(const std::string& cameraFile) {
        camera_file_ = cameraFile;
    }

    const int& frames() const {
        return frames_;
    }

    void frames(int frames) {
        frames_ = frames;
    }

    int test_max() const {
        return test_max_;
    }

    void test_max(int new_test_max) {
        test_max_ = new_test_max;
    }

    const int& test_interval() const {
        return test_interval_;
    }

    void test_interval(int new_test_interval) {
        test_interval_ = new_test_interval;
    }
};
