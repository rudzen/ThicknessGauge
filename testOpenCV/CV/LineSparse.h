#pragma once
#include <vector>
#include <map>
#include <opencv2/opencv.hpp>
#include <opencv2/stitching/detail/warpers.hpp>

#include "namespaces/sort.h"

/**
 * \brief Line class, contains information from a singular captured frame.
 * Including defined sections for 3 sides (right/center/left), frame reference and test output matrix.
 * Considered obsolete and still here for reference as it contains some interesting differentiation functions
 */
class Line {
public:
    enum class Location {
        base_one, base_two, heigth_one, heigth_two
    };

private:

    enum class SortMethod {
        none, x, y
    };

    const std::map<Location, int> location_base_map_ = {{Location::base_one , 0}, {Location::base_two, 1}};

    const std::map<Location, int> location_heigth_map_ = {{Location::heigth_one , 0}, {Location::heigth_two, 1}};

    std::map<int, int> intensity_;

public:
    const std::map<int, int>& intensity() const {
        return intensity_;
    }

    void intensity(const std::map<int, int>& intensity) {
        intensity_ = intensity;
    }

private:
    /**
    * \brief The frame for which the data set in the class is based
    */
    cv::Mat frame_;

    /**
    * \brief Matrix representation of the data vectors in the class
    */
    cv::Mat output_;

    /**
    * \brief Area of interest based on the 4 "focus" areas
    */
    cv::Rect roi[4];

    /**
     * \brief All the complete work from the class. Diffirentiated Y values + differentiated intensity values.
     */
    std::vector<cv::Point2i> all_complete_;

    /**
    * \brief All the sparse elements
    */
    std::vector<cv::Point2i> all_sparse_;

    /**
    * \brief All the raw pixel locations from frame
    */
    std::vector<cv::Point2i> all_total_;

    /**
     * \brief All the sparse elements differentiated
     */
    std::vector<cv::Point2i> all_differentiated_;

    // temporary bastard
    std::vector<cv::Point2i> tmp;

    /**
    * \brief Left side of the elements
    */
    std::vector<cv::Point2i> left_one_;

    /**
    * \brief Left side #2 of the elements
    */
    std::vector<cv::Point2i> left_two_;

    /**
    * \brief Right side #1 of the elements
    */
    std::vector<cv::Point2i> right_one_;

    /**
    * \brief Right side #2 of the elements
    */
    std::vector<cv::Point2i> right_two_;

    /**
    * \brief The calculated baseline for all 3 sides
    */
    double base_line_[2] = {0.0, 0.0};

    double heigth_line_[2] = {0.0, 0.0};

public:

    /**
     * \brief Differentiates points in a vector
     * \param input The vector to be differentiated
     * \param output The results
     */
    static void differentiate_y(std::vector<cv::Point2i>& input, std::vector<cv::Point2i>& output);

    /**
     * \brief Diffirentiates twice :>
     */
    void differentiate_y();

    void save_all_data(std::string& file_prefix);

    /**
     * \brief Differentiates the intensity levels in X direction
     */
    void differentiate_intensity();

    bool point_search_x(cv::Point2i i, cv::Point2i j);

    /**
     * \brief Merges the differentiated values of height and intensity into Y
     */
    void merge_intensity();

    /**
     * \brief Combines two vectors into a third
     * \param source_one The first vector
     * \param source_two The second vector
     * \param target The target vector with sourceOne and sourceTwo data
     * \param sort_x Sorting method to be used when combining is done
     */
    static void combine(std::vector<cv::Point2d>& source_one, std::vector<cv::Point2d>& source_two, std::vector<cv::Point2d> target, SortMethod sort_x);

    /**
     * \brief Get the pixel intensity of a location from the current frame
     * \param location The point location to get the intensity from
     * \return The grey scale pixel intensity
     */
    unsigned char get_pixel_intensity(cv::Point2d& location);

    /**
     * \brief Generates the output matrix based on the current elements
     */
    void generate_output();

    bool generate_sparse();

public:

    /**
     * \brief Splits the elements based on values in X,
     * <rigth> < rightX, <center> < leftX, the rest in <left>
     * \param right_one The right section border in X
     * \param right_two The left section border in X
     */
    void split(double left_one, double left_two, double right_one, double right_two);

    void draw_poly();

public: // getters and setter + minor functions

    /**
     * \brief Reset the default output matrix
     */
    void reset_output() {
        reset_output(frame_);
    }

    /**
     * \brief Reset the default output matrix using custom matrix as template
     * \param template_frame The template to base the configuration of the output matrix on
     */
    void reset_output(cv::Mat& template_frame) {
        output_ = cv::Mat::zeros(template_frame.rows, template_frame.cols, template_frame.type());
    }

    /**
     * \brief Set the class main frame reference (no pun)
     * \param frame_to_set The frame t
     */
    void frame(const cv::Mat& frame_to_set) {
        frame_to_set.copyTo(frame_); // copy
    }

    /**
     * \brief Get output matrix reference
     * \return The output matrix reference
     */
    const cv::Mat& output() const {
        return output_;
    }

    /**
     * \brief Get the baseline (Y)
     * \param location For which location
     * \return The baseline (Y)
     */
    double line(Location location) {
        switch (location) {
        case Location::base_one:
        case Location::base_two:
            return base_line_[location_base_map_.at(location)];
        case Location::heigth_one:
        case Location::heigth_two:
        default:
            return heigth_line_[location_heigth_map_.at(location)];
        }
    }

};

inline void Line::differentiate_y(std::vector<cv::Point2i>& input, std::vector<cv::Point2i>& output) {

    output.clear();

    if (input.empty()) {
        return;
    }

    auto size = input.size();

    if (size == 1) {
        output.emplace_back(cv::Point(input.front().x, -input.front().y));
        return;
    }

    output.reserve(input.size() - 1);

    for (auto i = 1; i < size; ++i) {
        output.emplace_back(cv::Point(input[i].x, input[i].y - input[i - 1].y));
    }
}

inline void Line::differentiate_y() {

    auto size = all_sparse_.size();

    if (size == 1) {
        all_differentiated_.emplace_back(cv::Point(all_sparse_.front().x, -all_sparse_.front().y));
        return;
    }

    tmp.reserve(all_sparse_.size() - 1);

    for (auto i = 1; i < size; ++i) {
        tmp.emplace_back(cv::Point(all_sparse_[i].x, all_sparse_[i].y - all_sparse_[i - 1].y));
    }

    differentiate_y(tmp, all_differentiated_);
}

inline void Line::save_all_data(std::string& file_prefix) {
    std::ofstream all(file_prefix + "_d_0_all.txt");
    std::ofstream sparse(file_prefix + "_d_1_sparse.txt");
    std::ofstream inten(file_prefix + "_d_2_intensity.txt");
    std::ofstream diff1(file_prefix + "_d_3_diff1.txt");
    std::ofstream diff2(file_prefix + "_d_4_diff2.txt");
    std::ofstream full(file_prefix + "_d_5_full.txt");

    for (auto& p : all_total_) {
        all << p << '\n';
    }

    for (auto& p : all_sparse_) {
        sparse << p << '\n';
    }

    for (auto& p : intensity_) {
        inten << '[' << p.first << ", " << p.second << "]\n";
    }

    for (auto& p : tmp) {
        diff1 << p << '\n';
    }

    for (auto& p : all_differentiated_) {
        diff2 << p << '\n';
    }

    for (auto& p : all_complete_) {
        full << p << '\n';
    }

    diff2.close();
    diff1.close();
    inten.close();
    sparse.close();
    all.close();
    full.close();

}

inline void Line::differentiate_intensity() {

    if (frame_.empty()) {
        return;
    }

    if (all_sparse_.empty()) {
        return;
    }

    auto size = all_sparse_.size();

    intensity_.clear();

    if (size == 1) {
        auto front = all_sparse_.front();
        intensity_[front.x] = frame_.at<unsigned char>(front);
    }

    for (auto i = 1; i < size; ++i) {
        intensity_[all_sparse_[i].x] = frame_.at<unsigned char>(all_sparse_[i]) - frame_.at<unsigned char>(all_sparse_[i - 1]);
        //auto val = intensity_[allSparse_[i].x];
        //log_time << val << endl;
    }

}

inline void Line::merge_intensity() {

    if (all_differentiated_.empty()) {
        return;
    }

    if (intensity_.empty()) {
        return;
    }

    for (auto& incent : all_differentiated_) {
        if (intensity_.count(incent.x)) {
            all_complete_.emplace_back(cv::Point(incent.x, intensity_[incent.x] + incent.y));
        } else {
            all_complete_.emplace_back(incent);
        }
    }

    sorter::sort_pixels_x_ascending(all_complete_);
}

inline void Line::combine(std::vector<cv::Point2d>& source_one, std::vector<cv::Point2d>& source_two, std::vector<cv::Point2d> target, SortMethod sort_x) {
    target.reserve(source_one.size() + source_two.size());
    target.insert(target.begin(), source_one.begin(), source_one.end());
    target.insert(target.end(), source_two.begin(), source_two.end());
    if (sort_x == SortMethod::x) {
        sorter::sort_pixels_x_ascending(target);
    } else if (sort_x == SortMethod::y) {
        sorter::sort_pixels_y_ascending(target);
    }
}

inline unsigned char Line::get_pixel_intensity(cv::Point2d& location) {
    if (frame_.empty()) {
        return 0;
    }

    if (frame_.cols > location.x) {
        return 0;
    }

    if (frame_.rows > location.y) {
        return 0;
    }

    return frame_.at<uchar>(location);
}

inline void Line::generate_output() {
    // just basic method, can be optimized.
    for (auto& e : all_sparse_) {
        output_.at<unsigned char>(e) = 255;
    }
}

inline bool Line::generate_sparse() {

    if (!all_sparse_.empty()) {
        all_sparse_.clear();
    }

    all_sparse_.reserve(frame_.cols);

    all_total_.clear();
    all_total_.reserve(frame_.cols * frame_.rows);

    findNonZero(frame_, all_total_);

    if (all_total_.empty()) {
        return false;
    }

    // sort the list in X
    sorter::sort_pixels_x_ascending(all_total_);

    auto y = 0;
    auto count = 0;
    auto highest = 0;

    auto x = all_total_.front().x;

    for (auto& p : all_total_) {
        if (p.x != x) {
            if (count > 0) {
                all_sparse_.emplace_back(cv::Point(x, frame_.rows - y));
                count = 0;
            }
            highest = 0;
        }
        auto intensity = frame_.at<unsigned char>(p);
        if (intensity >= highest) {
            highest = p.y;
            x = p.x;
            y = p.y;
        }
        count++;
    }

    return all_sparse_.empty() ^ true;

}

inline void Line::split(double left_one, double left_two, double right_one, double right_two) {
    if (all_sparse_.empty()) {
        return;
    }

    // TODO : Validate input.

    auto size = all_sparse_.size();

    right_one_.clear();
    left_one_.clear();

    right_one_.reserve(size);
    left_one_.reserve(size);

    for (auto& p : all_sparse_) {
        if (p.x < left_one) {
            left_one_.emplace_back(p);
        } else if (p.x < left_two) {
            left_two_.emplace_back(p);
        } else if (p.x < right_one) {
            right_one_.emplace_back(p);
        } else {
            right_two_.emplace_back(p);
        }
    }
}

inline void Line::draw_poly() {
    polylines(frame_, all_sparse_, false, cv::Scalar(255, 255, 255));
}
