#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

enum class VideoCodec {
    Mjpeg
};

enum class VideoColour {
    Colour, Grey
};

class VideoInfo {

public:

    VideoInfo()
        : fps_(25.0), codec_(0), colour_(false) {
    }

    int codec() const;

    void codec(VideoCodec codec);

    double fps() const;

    void fps(float FPS);

    cv::Size size() const;

    void size(int x, int y);

    void size(cv::Size size);

    bool colour() const;

    void colour(VideoColour colour);

private:

    const std::map<VideoCodec, int> video_codecs_ = {{VideoCodec::Mjpeg, CV_FOURCC('M', 'J', 'P', 'G')}};

    const std::map<VideoColour, bool> video_colour_ = {{VideoColour::Colour, true},{VideoColour::Grey, false}};

    cv::Size video_size_;

    double fps_;

    int codec_;

    bool colour_;

};
