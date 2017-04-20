#pragma once

namespace _cv {

	enum class Quantile { Q0, Q25, Q50, Q75, Q100 };

	enum class SortBy { X, Y };

	enum class TextDrawPosition { UpperLeft, UpperRight, LowerLeft, LowerRight };

	enum class WindowType { Input, Output, Temp };

	enum class SaveType { Image_Jpeg, Image_Png, Video };

	enum class Information { None, Basic, Full };

}