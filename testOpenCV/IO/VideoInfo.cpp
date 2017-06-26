#include "VideoInfo.h"

double VideoInfo::fps() const {
    return fps_;
}

VideoInfo::VideoInfo()
    : fps_(25.0)
    , codec_(0)
    , colour_(false) { }

int VideoInfo::codec() const {
    return codec_;
}

void VideoInfo::codec(VideoCodec codec) {
    codec_ = video_codecs_.at(codec);
}

void VideoInfo::fps(float new_fps) {
    fps_ = new_fps;
}

cv::Size VideoInfo::size() const {
    return video_size_;
}

void VideoInfo::size(int x, int y) {
    video_size_.width = x;
    video_size_.height = y;
}

void VideoInfo::size(cv::Size size) {
    video_size_.width = size.width;
    video_size_.height = size.width;
}

bool VideoInfo::colour() const {
    return colour_;
}

void VideoInfo::colour(VideoColour colour) {
    colour_ = video_colour_.at(colour);
}
