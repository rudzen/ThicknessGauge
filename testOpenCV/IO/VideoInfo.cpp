#include "VideoInfo.h"

double VideoInfo::fps() const {
    return fps_;
}

int VideoInfo::codec() const {
    return codec_;
}

void VideoInfo::codec(VideoCodec codec) {
    codec_ = video_codecs_.at(codec);
}

void VideoInfo::fps(float FPS) {
    fps_ = FPS;
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
