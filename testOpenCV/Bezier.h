#pragma once
#include <opencv2/core/matx.hpp>
#include "PointData.h"

template <typename T>
class Bezier {

	static v3<T> bezier3(v3<T>& p1, v3<T>& p2, v3<T>& p3, double mu);

	/*
	Four control point Bezier interpolation
	mu ranges from 0 to 1, start to end of curve
	*/
	static v3<T> bezier4(v3<T>& p1, v3<T>& p2, v3<T>& p3, v3<T>& p4, double mu);


	/*
	General Bezier curve
	Number of control points is n+1
	0 <= mu < 1    IMPORTANT, the last point is not computed
	*/
	v3<T> bezier(std::vector<v3<T>>& p, int n, double mu);

};

template <typename T>
v3<T> Bezier<T>::bezier3(v3<T>& p1, v3<T>& p2, v3<T>& p3, double mu) {

	auto mu2 = mu * mu;
	auto mum1 = 1 - mu;
	auto mum12 = mum1 * mum1;
	return p(p1.x * mum12 + 2 * p2.x * mum1 * mu + p3.x * mu2,
	         p1.y * mum12 + 2 * p2.y * mum1 * mu + p3.y * mu2,
	         p1.z * mum12 + 2 * p2.z * mum1 * mu + p3.z * mu2);
}

template <typename T>
v3<T> Bezier<T>::bezier4(v3<T>& p1, v3<T>& p2, v3<T>& p3, v3<T>& p4, double mu) {
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
v3<T> Bezier<T>::bezier(std::vector<v3<T>>& p, int n, double mu) {

	if (n < 1)
		return nullptr;

	v3<T> b;

	double muk = 1;
	auto munk = pow(1 - mu, static_cast<double>(n));

	for (auto k = 0; k <= n; k++) {
		auto nn = n;
		auto kn = k;
		auto nkn = n - k;
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
