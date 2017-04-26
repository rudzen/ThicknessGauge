#pragma once

#include "_cv.h"
#include <opencv2/core/mat.hpp>

using namespace _cv;

class PointData {


public:
	PointData(const p center, const p lowerCenter, const p upperCenter)
		: center_(center),
		  lowerCenter_(lowerCenter),
		  upperCenter_(upperCenter), centerDist(0) {
	}

private:

	p center_;
	p lowerCenter_;
	p upperCenter_;

	double centerDist;

public:
	const p& Center() const;
	void set_center(p center);
	const p& LowerCenter() const;
	void set_lower_center(p lowerCenter);
	const p& UpperCenter() const;
	void set_upper_center(p upperCenter);
	const double& CenterDist() const;
	void set_center_dist(double centerDist);
};

inline const p& PointData::Center() const {
	return center_;
}

inline void PointData::set_center(p center) {
	center_ = center;
}

inline const p& PointData::LowerCenter() const {
	return lowerCenter_;
}

inline void PointData::set_lower_center(p lowerCenter) {
	lowerCenter_ = lowerCenter;
}

inline const p& PointData::UpperCenter() const {
	return upperCenter_;
}

inline void PointData::set_upper_center(p upperCenter) {
	upperCenter_ = upperCenter;
}

inline const double& PointData::CenterDist() const {
	return centerDist;
}

inline void PointData::set_center_dist(double centerDist) {
	this->centerDist = centerDist;
}
