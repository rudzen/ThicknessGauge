#pragma once

namespace _cv {

	typedef std::vector<cv::Point2d> vd;
	typedef std::vector<cv::Point2i> vi;

	typedef std::pair<cv::Point2i, cv::Point2i> linePair;

	enum class Quantile { Q0, Q25, Q50, Q75, Q100 };

	enum class SortBy { X, Y };

	enum class TextDrawPosition { UpperLeft, UpperRight, LowerLeft, LowerRight };

	enum class WindowType { Input, Output, Temp };

	enum class SaveType { Image_Jpeg, Image_Png, Video };

	enum class Information { None, Basic, Full };

	enum class GlobType { Test, Calibration, Sequence };

	enum class FilterKernels { Simple3x3, Horizontal10x10 };

}
