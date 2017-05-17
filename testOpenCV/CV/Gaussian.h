#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/shape/hist_cost.hpp>

   /*
	|  __
	| /__\
	| X~~|			"The eternal code god
	|-\|//-.		 watches over this mess."
   /|`.|'.' \			- R.A.Kohn, 2017
  |,|.\~~ /||
  |:||   ';||
  ||||   | ||
  \ \|     |`.
  |\X|     | |
  | .'     |||
  | |   .  |||
  |||   |  `.| JS
  ||||  |   ||
  ||||  |   ||
  `+.__._._+*/

class Gaussian {
	
	cv::Mat original_;

	cv::Mat image_;

	cv::Mat result_;

	cv::Size kernel_;

	double sigmaX_;

	double sigmaY_;

	int border_;

public:

	Gaussian(const cv::Mat& image, const cv::Size& kernel, double sigmaX, double sigmaY, int border) : image_(image)
	                                                                                                 , kernel_(kernel)
	                                                                                                 , sigmaX_(sigmaX)
	                                                                                                 , sigmaY_(sigmaY)
	                                                                                                 , border_(border) {}

	Gaussian(const cv::Mat& image, const cv::Size& kernel, double sigmaX, double sigmaY) : image_(image)
	                                                                                     , kernel_(kernel)
	                                                                                     , sigmaX_(sigmaX)
	                                                                                     , sigmaY_(sigmaY)
	                                                                                     , border_(cv::BORDER_DEFAULT) {}

	Gaussian() : sigmaX_(0.0)
		, sigmaY_(0.0)
		, border_(0) { }

	const cv::Mat& getOriginal() const { return original_; }

	void setOriginal(const cv::Mat& original) { original_ = original; }

	const cv::Mat& getImage() const { return image_; }

	void setImage(const cv::Mat& image) { image_ = image; }

	const cv::Mat& getResult() const { return result_; }

	void setResult(const cv::Mat& result) { result_ = result; }

	cv::Size& getKernel() { return kernel_; }

	void setKernel(const cv::Size& kernel) { kernel_ = kernel; }

	double getSigmaX() const { return sigmaX_; }

	void setSigmaX(double sigmaX) { sigmaX_ = sigmaX; }

	double getSigmaY() const { return sigmaY_; }

	void setSigmaY(double sigmaY) { sigmaY_ = sigmaY; }

	int getBorder() const { return border_; }

	void setBorder(int border) { border_ = border; }

public:

	void doGaussian();

	void doGaussian(cv::Size& kernel, double sigmaX, double sigmaY);

};

inline void Gaussian::doGaussian() {
	doGaussian(kernel_, sigmaX_, sigmaY_);
}

inline void Gaussian::doGaussian(cv::Size& kernel, double sigmaX, double sigmaY) {
	cv::GaussianBlur(image_, result_, kernel, sigmaX, sigmaY, border_);
}

