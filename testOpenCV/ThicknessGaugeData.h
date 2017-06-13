#pragma once

#include <array>
#include <ostream>

class ThicknessGaugeData {

protected:

    /**
     * \brief The "main" frame container.
     * Has all the information which is required for the entire process.
     * This structure is contained within the vector "frameset", and is accessed
     * easily by pointing.. like..
     * auto frames = frameset[frameset_index].get();
     */
    typedef struct Frames {
        std::vector<cv::Mat> frames;
        std::vector<double> means;
        std::vector<double> stddevs;
        std::string exp_ext;
        int exp_ms;

        Frames(const std::string& expExt, int expMs)
            : exp_ext(expExt),
              exp_ms(expMs) {
        }

        /**
         * \brief Clear the entire structure
         */
        void clear() {
            frames.clear();
            means.clear();
            stddevs.clear();
            exp_ext = "";
            exp_ms = 0;
        }

        /**
         * \brief Computes mean and stddev based on the contained frames.
         */
        void compute() {
            cv::Scalar mean;
            cv::Scalar stddev;
            means.clear();
            means.shrink_to_fit();
            means.reserve(frames.size());
            stddevs.clear();
            stddevs.shrink_to_fit();
            stddevs.reserve(frames.size());
            for (auto& frame : frames) {
                // TODO : replace with miniCalc function(s)
                cv::meanStdDev(frame, mean, stddev);
                means.emplace_back(mean[0]);
                stddevs.emplace_back(stddev[0]);
            }
        }
    } Frames;

    std::array<int, 3> exposures = {5000, 20000, 40000};
    std::array<std::string, 3> expusures_short = {"_5k", "_20k", "_40k"};
    std::vector<std::unique_ptr<Frames>> frameset;

    std::vector<cv::Mat> nulls_;

    cv::Size imageSize_;

    void initFrames() {
        frameset.clear();
        frameset.reserve(3);
        for (auto i = 0; i < exposures.size(); i++) {
            auto fra = std::make_unique<Frames>(expusures_short[i], exposures[i]);
            frameset.emplace_back(std::move(fra));
        }
        frameset.shrink_to_fit();

    }

    friend std::ostream& operator<<(std::ostream& os, const std::unique_ptr<Frames> const& obj) {
        os << "[Frame Structure]\n{\n";
        if (obj->frames.empty()) {
            return os << "\t\"frameCount\": null,\n}";
        }
        os << cv::format("\t\"frameCount\": %i,\n", obj->frames.size());
        os << "\t\"frameDimensions\": " << obj->frames.front().size() << ",\n"; // , obj->frames.front().rows);
        os << "\t\"frameType\": " << obj->frames.front().type() << ",\n"; // , obj->frames.front().rows);
        os << cv::format("\t\"frameExposureDefault\": %i,\n", obj->exp_ms);
        os << cv::format("\t\"frameExposureExtension\": %s,\n", obj->exp_ext);

        // means
        os << "\t\"means\":[ ";
        auto size = obj->means.size();
        for (auto i = 0; i < size; i++) {
            os << cv::format("\"%f\"", obj->means[i]);
            if (i != size - 1)
                os << ",";
        }
        os << " ],\n";

        os << "\t\"stddevs\":[ ";
        size = obj->stddevs.size();
        for (auto i = 0; i < size; i++) {
            os << cv::format("\"%f\"", obj->stddevs[i]);
            if (i != size - 1)
                os << ",";
        }
        return os << " ]\n}\n";
    }

public:

    ThicknessGaugeData() {
        initFrames();
    }

    void setImageSize(cv::Size size);

    void setImageSize(const int width, const int height);

};

inline void ThicknessGaugeData::setImageSize(cv::Size size) {
    imageSize_ = size;
}

inline void ThicknessGaugeData::setImageSize(const int width, const int height) {
    imageSize_.width = width;
    imageSize_.height = height;
}
