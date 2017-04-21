#include "ImageSave.h"
#include "ThicknessGauge.h"
#include "Stringtools.h"
#include "Util.h"
#include "InvalidFileException.h"

using namespace utils;

void ImageSave::SaveImage(cv::Mat* image) const {
	SaveImage(image, fileName_);
}

void ImageSave::SaveImage(cv::Mat* image, string filename) const {

	replace(filename.begin(), filename.end(), ' ', '_');

	auto outFile = "./images/_" + to_string(timeStamp_) + '-' + filename + '.' + (!StringTools::endsWith(filename, m_FileExtensions.at(saveType_)) ? m_FileExtensions.at(saveType_) : "");

	if (!Util::validFileName(outFile)) {
		throw InvalidFileException("Invalid filename for " + outFile);
	}

	if (information_ == Information::None) {
		imwrite(outFile, *image);
		return;
	}

	string info;
	info.append("rows: ").append(to_string(image->rows));
	info.append(" cols: ").append(to_string(image->cols));
	putText(*image, info.c_str(), cvPoint(image->cols >> 3, image->rows >> 2), 1, 1.0, CV_RGB(0, 0, 0), 2);

	if (information_ == Information::Basic) {
		imwrite(outFile, *image);
		return;
	}

	imwrite(outFile, *image);
}

void ImageSave::OpenVideo() {
	videoWriter_.open("./images/" + fileName_ + '.' + m_FileExtensions.at(saveType_), GetCodec(), GetFPS(), GetSize(), IsColour());
}

void ImageSave::CloseVideo() {
	cout << m_FileExtensions.at(saveType_) << GetCodec() << GetFPS() << GetSize() << IsColour();
	videoWriter_.release();
}

void ImageSave::SaveVideoFrame(cv::Mat& image) {
	videoWriter_.write(image);
}

void ImageSave::SetSaveType(const SaveType new_type) {
	saveType_ = new_type;
}

SaveType ImageSave::GetSaveMode() const {
	return saveType_;
}

void ImageSave::SetInformation(const Information information) {
	information_ = information;
}

Information ImageSave::GetInformation() const {
	return information_;
}

void ImageSave::UpdateTimeStamp() {
	timeStamp_ = cvGetTickCount();
}

void ImageSave::SetFileName(string FileName) {
	fileName_ = FileName;
}


