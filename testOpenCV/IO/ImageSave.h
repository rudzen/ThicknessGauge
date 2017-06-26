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

    ImageSave();

    explicit ImageSave(std::string filename);

    ImageSave(std::string filename, SaveType save_type);

    ImageSave(ImageSave& image_save);

    ImageSave(std::string filename, SaveType save_type, Information information);

    ~ImageSave();

    void save_image(cv::Mat* image) const;

    void save_image(cv::Mat* image, std::string file_name) const;

    void open_video();

    void close_video();

    void save_video_frame(cv::Mat& image);

    void savetype(const SaveType type);

    SaveType save_mode() const;

    void information(const Information information);

    Information information() const;

    void update_time_stamp();

    void file_name(std::string new_filename);

private:

    cv::VideoWriter video_writer_;

    const std::map<SaveType, std::string> file_extensions_ = {{SaveType::Image_Jpeg, "jpg"},{SaveType::Image_Png, "png"},{SaveType::Video, "avi"}};

    SaveType save_type_;

    Information information_;

    std::string file_name_;

    int64 time_stamp_;

};
