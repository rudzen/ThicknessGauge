#include "VideoInfo.h"

int VideoInfo::GetCodec() const {
	return m_Codec;
}

double VideoInfo::GetFPS() const {
	return m_FPS;
}

void VideoInfo::SetCodec(VideoCodec codec) {
	m_Codec = m_VideoCodecs.at(codec);
}

void VideoInfo::SetFPS(float FPS) {
	m_FPS = FPS;
}

cv::Size VideoInfo::GetSize() const {
	return m_VideoSize;
}

void VideoInfo::SetSize(int x, int y) {
	m_VideoSize.width = x;
	m_VideoSize.height = y;
}

void VideoInfo::SetSize(cv::Size size) {
	m_VideoSize.width = size.width;
	m_VideoSize.height = size.width;
}

bool VideoInfo::IsColour() const {
	return m_Colour;
}

void VideoInfo::SetColour(VideoColour colour) {
	m_Colour = m_VideoColour.at(colour);
}
