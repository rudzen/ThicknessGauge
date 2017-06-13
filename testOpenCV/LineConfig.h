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
    int distType_;

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
    double aepa_;

public:
    LineConfig(const int distType, const double params, const double reps, const double aepa)
        : distType_(distType),
          params_(params),
          reps_(reps),
          aepa_(aepa) {
    }

    /**
     * \brief Default configuration
     */
    LineConfig()
        : distType_(cv::DIST_L12), params_(0.0), reps_(0.01), aepa_(0.01) {
    }

    int getDistType() const {
        return distType_;
    }

    void setDistType(int distType) {
        distType_ = distType;
    }

    double getParams() const {
        return params_;
    }

    void setParams(double params) {
        params_ = params;
    }

    double getReps() const {
        return reps_;
    }

    void setReps(double reps) {
        reps_ = reps;
    }

    double getAepa() const {
        return aepa_;
    }

    void setAepa(double aepa) {
        aepa_ = aepa;
    }
};
