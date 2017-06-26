#include "IO/ImageSave.h"
#include "ThicknessGauge.h"
#include "Util/Stringtools.h"
#include "Exceptions/InvalidFileException.h"

#include "../namespaces/filesystem.h"

ImageSave::ImageSave()
    : ImageSave("", SaveType::Image_Png, Information::Basic) { }

ImageSave::ImageSave(std::string FileName)
    : ImageSave(FileName, SaveType::Image_Png, Information::Basic) { }

ImageSave::ImageSave(std::string FileName, SaveType SaveType)
    : ImageSave(FileName, SaveType, Information::Basic) { }

ImageSave::ImageSave(ImageSave& imageSave)
    : ImageSave(imageSave.file_name_, imageSave.save_type_, imageSave.information_) { }

ImageSave::ImageSave(std::string FileName, SaveType save_type, Information information)
    : save_type_(save_type)
    , information_(information)
    , file_name_(FileName) {
    update_time_stamp();
}

ImageSave::~ImageSave() {
    video_writer_.~VideoWriter();
}

void ImageSave::save_image(cv::Mat* image) const {
    save_image(image, file_name_);
}

void ImageSave::save_image(cv::Mat* image, string filename) const {

    replace(filename.begin(), filename.end(), ' ', '_');

    auto outFile = "./images/_" + to_string(time_stamp_) + '-' + filename + '.' + (!utils::StringTools::endsWith(filename, file_extensions_.at(save_type_)) ? file_extensions_.at(save_type_) : "");

    if (!file::is_name_legal(outFile)) {
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

void ImageSave::open_video() {
    video_writer_.open("./images/" + file_name_ + '.' + file_extensions_.at(save_type_), codec(), fps(), size(), colour());
}

void ImageSave::close_video() {
    log_time << file_extensions_.at(save_type_) << codec() << fps() << size() << colour();
    video_writer_.release();
}

void ImageSave::save_video_frame(cv::Mat& image) {
    video_writer_.write(image);
}

void ImageSave::set_save_type(const SaveType new_type) {
    save_type_ = new_type;
}

SaveType ImageSave::save_mode() const {
    return save_type_;
}

void ImageSave::information(const Information information) {
    information_ = information;
}

Information ImageSave::information() const {
    return information_;
}

void ImageSave::update_time_stamp() {
    time_stamp_ = cvGetTickCount();
}

void ImageSave::file_name(string FileName) {
    file_name_ = FileName;
}
