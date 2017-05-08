#pragma once
#include <ostream>

typedef pair<float, float> floatBoundry;
typedef pair<int, int> intBoundry;
/**
 * \brief Contains meta for a singular test.
 */
class TestConfig {


private:
	
	int testNum_;

	float alpha_;

	int sigma_;

	cv::Size kernel_;

public:

	TestConfig(): alpha_(0.1f), sigma_(5), testNum_(1), kernel_(0, 0) { }

	TestConfig(const float alpha, const int sigma, const int testNum, const cv::Size kernel)
		: testNum_(testNum), alpha_(alpha),
		  sigma_(sigma), kernel_(kernel) {
	}


	const cv::Size& kernel() const {
		return kernel_;
	}

	void kernel(const cv::Size& kernel) {
		kernel_ = kernel;
	}

	const float& alpha() const {
		return alpha_;
	}

	void alpha(float alpha) {
		alpha_ = alpha;
	}

	const int& sigma() const {
		return sigma_;
	}

	void sigma(int sigma) {
		sigma_ = sigma;
	}

	const int& testNum() const {
		return testNum_;
	}

	void testNum(int testNum) {
		testNum_ = testNum;
	}


	friend std::ostream& operator<<(std::ostream& os, const TestConfig& obj) {
		return os
			<< "testNum: " << obj.testNum_
			<< " alpha: " << obj.alpha_
			<< " sigma: " << obj.sigma_
			<< " kernel: " << obj.kernel_;
	}

};