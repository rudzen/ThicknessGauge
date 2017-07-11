#pragma once
#include <opencv2/imgproc.hpp>

/**
* \brief LineConfig data holder
* Contains the meta for which to use when computing lines
* For more information:
* http://docs.opencv.org/3.0-beta/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html#fitline
*/
class LineConfig {

    /**
     * \brief Distance used by the M-estimator, see cv::DistanceTypes
     * 
     * DIST_USER	= User defined distance.
     * DIST_L1		= distance = |x1-x2| + |y1-y2|
     * DIST_L2		= the simple euclidean distance
     * DIST_C		= distance = max(|x1-x2|,|y1-y2|)
     * DIST_L12		= L1-L2 metric: distance = 2(sqrt(1+x*x/2) - 1))
     * DIST_FAIR	= distance = c^2(|x|/c-log(1+|x|/c)), c = 1.3998
     * DIST_WELSCH	= distance = c^2/2(1-exp(-(x/c)^2)), c = 2.9846
     * DIST_HUBER	= distance = |x|<c ? x^2/2 : c(|x|-c/2), c=1.345 
     */
    cv::DistanceTypes dist_type_;

    /**
     * \brief Numerical parameter ( C ) for some types of distances. If it is 0, an optimal value is chosen.
     */
    double params_;

    /**
     * \brief Sufficient accuracy for the radius (distance between the coordinate origin and the line).
     */
    double reps_;

    /**
     * \brief Sufficient accuracy for the angle. 0.01 would be a good default value for reps and aeps.
     */
    double aeps_;

public:
    LineConfig(const cv::DistanceTypes distType, const double params, const double reps, const double aepa)
        : dist_type_(distType)
          , params_(params)
          , reps_(reps)
          , aeps_(aepa) { }

    LineConfig(const LineConfig& other)
        : dist_type_{other.dist_type_},
          params_{other.params_},
          reps_{other.reps_},
          aeps_{other.aeps_} {}

    LineConfig(LineConfig&& other) = delete;

    LineConfig& operator=(const LineConfig& other) {
        if (this == &other)
            return *this;
        dist_type_ = other.dist_type_;
        params_ = other.params_;
        reps_ = other.reps_;
        aeps_ = other.aeps_;
        return *this;
    }

    LineConfig& operator=(LineConfig&& other) = delete;

    /**
     * \brief Default configuration
     */
    LineConfig()
        : dist_type_(cv::DIST_L12)
          , params_(0.0)
          , reps_(0.01)
          , aeps_(0.01) { }

    cv::DistanceTypes dist_type() const {
        return dist_type_;
    }

    void dist_type(cv::DistanceTypes distType) {
        dist_type_ = distType;
    }

    double params() const {
        return params_;
    }

    void params(double params) {
        params_ = params;
    }

    double reps() const {
        return reps_;
    }

    void reps(double reps) {
        reps_ = reps;
    }

    double aeps() const {
        return aeps_;
    }

    void aeps(double aepa) {
        aeps_ = aepa;
    }
};
