#pragma once
#include <ostream>
#include "Util/Util.h"

namespace tg {

	typedef std::vector<cv::Point2d> vd;
	typedef std::vector<cv::Point2i> vi;

	typedef std::pair<cv::Point2f, cv::Point2f> linePair;

	enum Side { Left = 0, Right = 1 };

	enum class Quantile { Q0, Q25, Q50, Q75, Q100 };

	enum class SortBy { X, Y };

	enum class TextDrawPosition { UpperLeft, UpperRight, LowerLeft, LowerRight };

	enum class WindowType { Input, Output, Temp };

	enum class SaveType { Image_Jpeg, Image_Png, Video };

	enum class Information { None, Basic, Full };

	enum class GlobType { Test, Calibration, Sequence };

	enum class FilterKernels { Simple3x3, Horizontal10x10 };

}
