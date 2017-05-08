#pragma once
#include <opencv2/core/matx.hpp>
#include "Util/Vec.h"

template <typename T>
class Bezier {

public:

	/**
	 * \brief Bezier 2d interpolation for 3 points
	 * \param p1 First point
	 * \param p2 Second point
	 * \param p3 Third point
	 * \param mu mu
	 * \return bezier point
	 */
	static v2<T> bezier3(v2<T>& p1, v2<T>& p2, v2<T>& p3, double mu);

	// testing for opencv points.. meh
	static cv::Point2d bezier3(cv::Point2d& p1, cv::Point2d& p2, cv::Point2d& p3, double mu);


	/**
	 * \brief Bezier 2d interpolation for 4 points
	 * \param p1 First point
	 * \param p2 Second point
	 * \param p3 Third Point
	 * \param p4 Fourth point
	 * \param mu mu, 0 <= mu < 1 ranges from 0.0 = start- towards 1.0 = end of curve
	 * \return Bezier point
	 */
	static v2<T> bezier4(v2<T>& p1, v3<T>& p2, v2<T>& p3, v2<T>& p4, double mu);

	/**
	 * \brief Generic bezier curve, n+1 control points, last point is not computed
	 * \param p Vector of points
	 * \param n Base amount of control points
	 * \param mu mu, 0 <= mu < 1 ranges from 0.0 = start- towards 1.0 = end of curve
	 * \return Bezier point
	 */
	v2<T> bezier(std::vector<v2<T>>& p, int n, double mu);

};


template <typename T>
class Bezier3 {

public:

	/**
	* \brief Bezier 3d interpolation for 3 points
	* \param p1 First point
	* \param p2 Second point
	* \param p3 Third point
	* \param mu mu
	* \return bezier point
	*/
	static v3<T> bezier3(v3<T>& p1, v3<T>& p2, v3<T>& p3, double mu);

	/**
	* \brief Bezier 3d interpolation for 4 points
	* \param p1 First point
	* \param p2 Second point
	* \param p3 Third Point
	* \param p4 Fourth point
	* \param mu mu, 0 <= mu < 1 ranges from 0.0 = start- towards 1.0 = end of curve
	* \return Bezier point
	*/
	static v3<T> bezier4(v3<T>& p1, v3<T>& p2, v3<T>& p3, v3<T>& p4, double mu);

	/**
	* \brief Generic bezier curve, n+1 control points, last point is not computed
	* \param p Vector of points
	* \param n Base amount of control points
	* \param mu mu, 0 <= mu < 1 ranges from 0.0 = start- towards 1.0 = end of curve
	* \return Bezier point
	*/
	v3<T> bezier(std::vector<v3<T>>& p, int n, double mu);

};

template <typename T>
v2<T> Bezier<T>::bezier3(v2<T>& p1, v2<T>& p2, v2<T>& p3, double mu) {
	auto mu2 = mu * mu;
	auto mum1 = 1 - mu;
	auto mum12 = mum1 * mum1;
	return p(p1.x * mum12 + 2 * p2.x * mum1 * mu + p3.x * mu2,
			 p1.y * mum12 + 2 * p2.y * mum1 * mu + p3.y * mu2);
}

template <typename T>
cv::Point2d Bezier<T>::bezier3(cv::Point2d& p1, cv::Point2d& p2, cv::Point2d& p3, double mu) {
	auto mu2 = mu * mu;
	auto mum1 = 1 - mu;
	auto mum12 = mum1 * mum1;
	return cv::Point2d(p1.x * mum12 + 2 * p2.x * mum1 * mu + p3.x * mu2,
					   p1.y * mum12 + 2 * p2.y * mum1 * mu + p3.y * mu2);
}

template <typename T>
v2<T> Bezier<T>::bezier4(v2<T>& p1, v3<T>& p2, v2<T>& p3, v2<T>& p4, double mu) {
	auto mum1 = 1 - mu;
	auto mum12 = mum1 * mum1;
	auto mum13 = mum12 * mum1;
	auto mu2 = mu * mu;
	auto mu3 = mu2 * mu;

	auto _3mumum12 = 3 * mu * mum12;
	auto _3mu2mum1 = 3 * mu2 * mum1;

	return p(mum13 * p1.x + _3mumum12 * p2.x + _3mu2mum1 * p3.x + mu3 * p4.x,
			 mum13 * p1.y + _3mumum12 * p2.y + _3mu2mum1 * p3.y + mu3 * p4.y);
}

template <typename T>
v2<T> Bezier<T>::bezier(std::vector<v2<T>>& p, int n, double mu) {
	if (n < 1)
		return nullptr;

	v2<T> b;

	double muk = 1;
	auto munk = pow(1 - mu, static_cast<double>(n));

	for (auto k = 0; k <= n; k++) {
		auto nn = n;
		auto kn = k;
		auto nkn = nn - kn;
		auto blend = muk * munk;
		muk *= mu;
		munk /= (1 - mu);
		while (nn >= 1) {
			blend *= nn--;
			if (kn > 1)
				blend /= static_cast<double>(kn--);
			if (nkn > 1)
				blend /= static_cast<double>(nkn--);
		}
		b += p[k] * blend;
	}
	return (b);
}

template <typename T>
v3<T> Bezier3<T>::bezier3(v3<T>& p1, v3<T>& p2, v3<T>& p3, double mu) {

	auto mu2 = mu * mu;
	auto mum1 = 1 - mu;
	auto mum12 = mum1 * mum1;
	return p(p1.x * mum12 + 2 * p2.x * mum1 * mu + p3.x * mu2,
	         p1.y * mum12 + 2 * p2.y * mum1 * mu + p3.y * mu2,
	         p1.z * mum12 + 2 * p2.z * mum1 * mu + p3.z * mu2);
}

template <typename T>
v3<T> Bezier3<T>::bezier4(v3<T>& p1, v3<T>& p2, v3<T>& p3, v3<T>& p4, double mu) {
	auto mum1 = 1 - mu;
	auto mum12 = mum1 * mum1;
	auto mum13 = mum12 * mum1;
	auto mu2 = mu * mu;
	auto mu3 = mu2 * mu;

	auto _3mumum12 = 3 * mu * mum12;
	auto _3mu2mum1 = 3 * mu2 * mum1;

	return p(mum13 * p1.x + _3mumum12 * p2.x + _3mu2mum1 * p3.x + mu3 * p4.x,
	         mum13 * p1.y + _3mumum12 * p2.y + _3mu2mum1 * p3.y + mu3 * p4.y,
	         mum13 * p1.z + _3mumum12 * p2.z + _3mu2mum1 * p3.z + mu3 * p4.z);
}

template <typename T>
v3<T> Bezier3<T>::bezier(std::vector<v3<T>>& p, int n, double mu) {

	if (n < 1)
		return nullptr;

	v3<T> b;

	double muk = 1;
	auto munk = pow(1 - mu, static_cast<double>(n));

	for (auto k = 0; k <= n; k++) {
		auto nn = n;
		auto kn = k;
		auto nkn = nn - kn;
		auto blend = muk * munk;
		muk *= mu;
		munk /= (1 - mu);
		while (nn >= 1) {
			blend *= nn--;
			if (kn > 1)
				blend /= static_cast<double>(kn--);
			if (nkn > 1)
				blend /= static_cast<double>(nkn--);
		}
		b += p[k] * blend;
	}
	return (b);
}
