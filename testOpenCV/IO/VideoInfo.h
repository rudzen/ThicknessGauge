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
        : Fps_(25.0), codec_(0), colour_(false) {
    }

    int GetCodec() const;

    void SetCodec(VideoCodec codec);

    double GetFPS() const;

    void SetFPS(float FPS);

    cv::Size GetSize() const;

    void SetSize(int x, int y);

    void SetSize(cv::Size size);

    bool IsColour() const;

    void SetColour(VideoColour colour);

private:

    const std::map<VideoCodec, int> m_VideoCodecs = {{VideoCodec::Mjpeg, CV_FOURCC('M', 'J', 'P', 'G')}};

    const std::map<VideoColour, bool> m_VideoColour = {{VideoColour::Colour, true},{VideoColour::Grey, false}};

    cv::Size videoSize_;

    double Fps_;

    int codec_;

    bool colour_;

};
