/*
* The MIT License
*
* Copyright 2015-7 Rudy Alex Kohn (s133235@student.dtu.dk).
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/

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

>> s133235 [ at ] student [ dot ] dtu [ dot ] dk
>> rudzen [ at ] gmail [ dot ] com
*/


#pragma once
#include <cmath>

// geometric (and close by) functions for the masses.
// None of http://www.cplusplus.com/reference/cmath/ is included whatsoever!

// Catching exceptions is for communists.

namespace utils {

	class Compute {

	public:

		template<typename T>
		double triangleArea(T a, T b, T c);

		template<typename T>
		static double triangleArea(T width, T height);

		static double sineA(double *__restrict opposite, double *__restrict hypotenuse) {
			return *opposite / *hypotenuse;
		}

		static double cosineA(double *__restrict adjacent, double *__restrict hypotenuse) {
			return *adjacent / *hypotenuse;
		}

		static double tangentA(double *__restrict opposite, double *__restrict adjacent) {
			return *opposite / *adjacent;
		}

		static double angleA(double *__restrict a, double *__restrict b, double *__restrict c) {
			return acos(((*b * *b) + (*c * *c) - (*a * *a)) / (2 * *b * *c));
		}

		static double angleB(double *__restrict a, double *__restrict b, double *__restrict c) {
			return acos(((*a * *a) + (*c * *c) - (*b * *b)) / (2 * *a * *c));
		}

		static double angleC(double *__restrict a, double *__restrict b, double *__restrict c) {
			return acos(((*a * *a) + (*b * *b) - (*c * *c)) / (2 * *a * *b));
		}

	};

	template <typename T>
	double Compute::triangleArea(T a, T b, T c) {
		return sqrt((a + b - c) * (a - b + c) * (-a + b + c) * (a + b + c)) * 0.25;
	}

	template <typename T>
	double Compute::triangleArea(T width, T height) {
		return (static_cast<double>(width) * static_cast<double>(height)) * 0.5;
	}

}
