#include "VideoInfo.h"

int VideoInfo::GetCodec() const {
    return codec_;
}

double VideoInfo::GetFPS() const {
    return Fps_;
}

void VideoInfo::SetCodec(VideoCodec codec) {
    codec_ = m_VideoCodecs.at(codec);
}

void VideoInfo::SetFPS(float FPS) {
    Fps_ = FPS;
}

cv::Size VideoInfo::GetSize() const {
    return videoSize_;
}

void VideoInfo::SetSize(int x, int y) {
    videoSize_.width = x;
    videoSize_.height = y;
}

void VideoInfo::SetSize(cv::Size size) {
    videoSize_.width = size.width;
    videoSize_.height = size.width;
}

bool VideoInfo::IsColour() const {
    return colour_;
}

void VideoInfo::SetColour(VideoColour colour) {
    colour_ = m_VideoColour.at(colour);
}
