#include "ImageSave.h"
#include "ThicknessGauge.h"

void ImageSave::SaveImage(cv::Mat* image) const {
	SaveImage(image, m_FileName);
}

void ImageSave::SaveImage(cv::Mat* image, string filename) const {

	replace(filename.begin(), filename.end(), ' ', '_');

	// TODO check extension !!!
	auto outFile = "./images/_" + to_string(m_TimeStamp) + '-' + filename + '.' + m_FileExtensions.at(m_SaveType);

	if (m_Information == Information::None) {
		imwrite(outFile, *image);
		return;
	}

	string info;
	info.append("rows: ").append(to_string(image->rows));
	info.append(" cols: ").append(to_string(image->cols));
	putText(*image, info.c_str(), cvPoint(image->cols >> 3, image->rows >> 2), 1, 1.0, CV_RGB(0, 0, 0), 2);

	if (m_Information == Information::Basic) {
		imwrite(outFile, *image);
		return;
	}

	imwrite(outFile, *image);
}

void ImageSave::OpenVideo() {
	m_VideoWriter.open("./images/" + m_FileName + '.' + m_FileExtensions.at(m_SaveType), GetCodec(), GetFPS(), GetSize(), IsColour());
}

void ImageSave::CloseVideo() {
	cout << m_FileExtensions.at(m_SaveType) << GetCodec() << GetFPS() << GetSize() << IsColour();
	m_VideoWriter.release();
}

void ImageSave::SaveVideoFrame(cv::Mat& image) {
	m_VideoWriter.write(image);
}

void ImageSave::SetSaveType(const SaveType new_type) {
	m_SaveType = new_type;
}

SaveType ImageSave::GetSaveMode() const {
	return m_SaveType;
}

void ImageSave::SetInformation(const Information information) {
	m_Information = information;
}

Information ImageSave::GetInformation() const {
	return m_Information;
}

void ImageSave::UpdateTimeStamp() {
	m_TimeStamp = cvGetTickCount();
}

void ImageSave::SetFileName(string FileName) {
	m_FileName = FileName;
}


