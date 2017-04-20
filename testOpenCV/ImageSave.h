#pragma once

#include <opencv2\opencv.hpp>
#include <opencv2/core/core.hpp>
#include "VideoInfo.h"
#include "_cv.h"

using namespace _cv;


/*! \brief Simple wrapper class for image and/or video saving
*
* Wraps OpenCV image/video save functionality in a simple typesafe structure.
*/
class ImageSave : public VideoInfo {

public:

	ImageSave() : ImageSave("", SaveType::Image_Png, Information::Basic) { }

	explicit ImageSave(std::string FileName) : ImageSave(FileName, SaveType::Image_Png, Information::Basic) { }

	ImageSave(std::string FileName, SaveType SaveType) : ImageSave(FileName, SaveType, Information::Basic) { }

	ImageSave(ImageSave &imageSave) : ImageSave(imageSave.fileName_, imageSave.saveType_, imageSave.information_) { }

	ImageSave(std::string FileName, SaveType save_type, Information information) : saveType_(save_type), information_(information), fileName_(FileName) {
		UpdateTimeStamp();
	}

	~ImageSave() {
		videoWriter_.~VideoWriter();
	}

	void SaveImage(cv::Mat *image) const;

	void SaveImage(cv::Mat *image, std::string filename) const;

	void OpenVideo();

	void CloseVideo();

	void SaveVideoFrame(cv::Mat &image);

	void SetSaveType(const SaveType type);

	SaveType GetSaveMode() const;

	void SetInformation(const Information information);

	Information GetInformation() const;

	void UpdateTimeStamp();

	void SetFileName(std::string FileName);

private:

	cv::VideoWriter videoWriter_;

	const std::map<SaveType, std::string> m_FileExtensions = { { SaveType::Image_Jpeg, "jpg" },{ SaveType::Image_Png, "png" },{ SaveType::Video, "avi" } };

	SaveType saveType_;

	Information information_;

	std::string fileName_;

	int64 timeStamp_;

};

