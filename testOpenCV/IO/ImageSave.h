#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "VideoInfo.h"
#include "../namespaces/tg.h"

using namespace tg;

/*! \brief Simple wrapper class for image and/or video saving
*
* Wraps OpenCV image/video save functionality in a simple typesafe structure.
*/
class ImageSave : public VideoInfo {

public:

    ImageSave() : ImageSave("", SaveType::Image_Png, Information::Basic) { }

    explicit ImageSave(std::string FileName) : ImageSave(FileName, SaveType::Image_Png, Information::Basic) { }

    ImageSave(std::string FileName, SaveType SaveType) : ImageSave(FileName, SaveType, Information::Basic) { }

    ImageSave(ImageSave& imageSave) : ImageSave(imageSave.file_name_, imageSave.save_type_, imageSave.information_) { }

    ImageSave(std::string FileName, SaveType save_type, Information information) : save_type_(save_type)
                                                                                 , information_(information)
                                                                                 , file_name_(FileName) {
        update_time_stamp();
    }

    ~ImageSave() {
        video_writer_.~VideoWriter();
    }

    void save_image(cv::Mat* image) const;

    void save_image(cv::Mat* image, std::string filename) const;

    void open_video();

    void close_video();

    void save_video_frame(cv::Mat& image);

    void set_save_type(const SaveType type);

    SaveType save_mode() const;

    void information(const Information information);

    Information information() const;

    void update_time_stamp();

    void file_name(std::string FileName);

private:

    cv::VideoWriter video_writer_;

    const std::map<SaveType, std::string> file_extensions_ = {{SaveType::Image_Jpeg, "jpg"},{SaveType::Image_Png, "png"},{SaveType::Video, "avi"}};

    SaveType save_type_;

    Information information_;

    std::string file_name_;

    int64 time_stamp_;

};
