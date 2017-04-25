/*
(      -4QQQQQQQQQQQQQQQQQQ: jQQQQQQQQQQ
(        4QQQQQQQQQQQQQQQQQ: jQQQQQQQQQQ
( )QQQm. ]QQQQQQQQQQQQQQQQQ: jQQQQQQQQQQ
( )WQQQ; ]Qf =QQQ  dQQ@^ -4: jQ(      QQ
( )WQQD  jQf =QQQ  dQW`  .   jQc___   QQ
(       jWQf =QQQ  dQf .mQc  jQQQQF  jQQ
(       ?WQf =QQQ  dQ; ]QQQ  jQQQP  jWQQ
( )WQQL  WWf =QQQ  dQ: jQQQ. jQQD  <QWQQ
( )WQQW  dQf :QQW  dQ; )QQ@  jQ@` _QQQQQ
( )WQQm  3Qk  ??'  dQL  "T'  jQ'  TTTTQQ
( )WQQQ  3QQ,   <  dQQ,   _. jQ       WW
wawWQQQwaaQQQwawWaamQQmc_wmwayQaaaaaaamQ
QWWQQQQWWQQQQQWQQQWQQQQQQQQWWQQQWQWQWQQQ
*/

/*
Short and precise description from http://stackoverflow.com/a/27448008

You have the number of positions between the points, and then you have the current position.
Think of mu as a percentage of the linear distance between the first point and the second
that is determined by the current position, and the total number of positions.

That is: mu = (double)current_position / number_of_positions_between_the_points;

That will give you values between 0 and 1, in fixed increments,
determined by how many positions you want to have between the points.
Hint: In your loop, j is the current position.
The other thing that you have to deal with is that you are calling a function named
Interpolate(point_a, point_b, 2048, j) but you haven't shown the implementation for that function.
Instead, you have the CosineInterpolate function.
Presumably you wanted to abstract the interpolation method by invoking CosineInterpolate from Interpolate.
The first part of the answer tells you how to do that. I hope this helps!
 */


#pragma once
#include <cmath>

class Interpolate {

	const long double PI = 3.141592653589793238L;

public:

	template <typename T>
	static T Linear(T y1, T y2, T mu);

	template <typename T>
	T Cosine(T y1, T y2, T mu);

	template <typename T>
	T Cubic(T y0, T y1, T y2, T y3, T mu);

	template <typename T>
	T CubicSmooth(T y0, T y1, T y2, T y3, T mu);

	/*
	Tension: 1 is high, 0 normal, -1 is low
	Bias: 0 is even, positive is towards first segment, negative towards the other
	*/
	template <typename T>
	T HermiteInterpolate(T y0, T y1, T y2, T y3, T mu, T tension, T bias);


};

template <typename T>
T Interpolate::Linear(T y1, T y2, T mu) {
	return (y1 * (1 - mu) + y2 * mu);
}

template <typename T>
T Interpolate::Cosine(T y1, T y2, T mu) {
	double mu2 = (1 - cos(mu * PI)) / 2;
	return (y1 * (1 - mu2) + y2 * mu2);
}

template <typename T>
T Interpolate::Cubic(T y0, T y1, T y2, T y3, T mu) {
	auto mu2 = mu * mu;
	auto a0 = y3 - y2 - y0 + y1;
	auto a1 = y0 - y1 - a0;
	auto a2 = y2 - y0;
	auto a3 = y1;
	return (a0 * mu * mu2 + a1 * mu2 + a2 * mu + a3);
}

template <typename T>
T Interpolate::CubicSmooth(T y0, T y1, T y2, T y3, T mu) {
	auto mu2 = mu * mu;
	auto a0 = -0.5 * y0 + 1.5 * y1 - 1.5 * y2 + 0.5 * y3;
	auto a1 = y0 - 2.5 * y1 + 2 * y2 - 0.5 * y3;
	auto a2 = -0.5 * y0 + 0.5 * y2;
	auto a3 = y1;
	return (a0 * mu * mu2 + a1 * mu2 + a2 * mu + a3);
}

template <typename T>
T Interpolate::HermiteInterpolate(T y0, T y1, T y2, T y3, T mu, T tension, T bias) {
	auto mu2 = mu * mu;
	auto mu3 = mu2 * mu;
	auto m0 = (y1 - y0) * (1 + bias) * (1 - tension) / 2;
	m0 += (y2 - y1) * (1 - bias) * (1 - tension) / 2;
	auto m1 = (y2 - y1) * (1 + bias) * (1 - tension) / 2;
	m1 += (y3 - y2) * (1 - bias) * (1 - tension) / 2;
	auto a0 = 2 * mu3 - 3 * mu2 + 1;
	auto a1 = mu3 - 2 * mu2 + mu;
	auto a2 = mu3 - mu2;
	auto a3 = -2 * mu3 + 3 * mu2;
	return (a0 * y1 + a1 * m0 + a2 * m1 + a3 * y2);
}
