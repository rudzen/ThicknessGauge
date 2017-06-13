#include "Calc/MiniCalc.h"

MiniCalc::MiniCalc() {
}

MiniCalc::~MiniCalc() {
}

int MiniCalc::highestPixelInLine(cv::Mat& image) const {
    auto const halfImageY = image.size().height / 2;

    vector<vector<cv::Point>> contours;
    findContours(image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    cv::Rect highestRect(0, 0, 0, 0);

    for (auto i = 0; i < contours.size(); ++i) {
        auto r = boundingRect(contours[i]);
        if (/* halfImageY < r.y && */ r.y > highestRect.y) {
            highestRect = r;
        }
        //cout << r << endl;
    }

    return highestRect.y;
}

[[deprecated("not really used anymore, but could still prove useful in the future")]]
bool MiniCalc::generatePlanarPixels(cv::Mat& input, cv::Mat& output, vector<cv::Point2f>& pixels, vector<cv::Point2f>& gradient_pixels) const {

    vector<cv::Point> pix;

    pix.reserve(input.cols);

    findNonZero(input, pix);

    gradient_pixels.clear();
    gradient_pixels.reserve(input.cols);
    for (auto x = input.cols; x--;)
        gradient_pixels.emplace_back(cv::Point2d(0.0, 0.0));

    pixels.clear();
    pixels.reserve(input.cols);

    // sort the list in X
    sort::sort_pixels_x_ascending(pix);

    auto x = pix.front().x;
    auto count = 0;
    auto y_sum = 0;
    auto y_mean = 0.0;
    auto gradient_sum = 0.0;

    for (const auto& p : pix) {
        if (p.x != x) {
            if (count > 0) {
                pixels.emplace_back(cv::Point(x, static_cast<int>(cvRound(y_sum / static_cast<double>(count)))));
                auto gradient = static_cast<unsigned char>(cvRound(gradient_sum / count));
                output.at<unsigned char>(pixels.back()) = gradient;
                gradient_pixels[x].y = gradient;
                count = 0;
            }
            y_sum = 0;
        }
        x = p.x;
        y_sum += p.y;
        y_mean += p.y;
        gradient_sum += input.at<unsigned char>(p);
        count++;
    }

    if (pixels.empty())
        return false;

    y_mean /= pixels.size();

    return true;
}

bool MiniCalc::getActualPixels(cv::Mat& image, vi& output, int y_limit) {
    vi result;
    cv::findNonZero(image, result);
    y_limit = abs(image.rows - y_limit);
    for (auto& p : result) {
        if (p.y <= y_limit)
            output.emplace_back(p);
    }
    return !output.empty();
}

bool MiniCalc::getActualPixels(vi& pixels, vi& target, double y_limit, double image_height) {
    if (!target.empty())
        target.clear();

    y_limit = abs(image_height - y_limit);

    target.reserve(cvRound(y_limit));

    for (auto& p : pixels) {
        if (p.y <= y_limit)
            target.emplace_back(p);
    }

    return !target.empty();
}
