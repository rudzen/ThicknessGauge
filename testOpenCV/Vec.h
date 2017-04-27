/*
* The MIT License
*
* Copyright 2016 Rudy Alex Kohn (s133235@student.dtu.dk).
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
//
// Class: Full v2 & v3 classes.
// Description : With ..
// - Functionality.
// - Operational overload.
// - Output stream overload.
//
// Created by rudz on 6/9/16.
//
#ifndef VEC_H
#define VEC_H
#include <ostream>

using namespace std;

template<class T>
class v2 {
public:
	virtual ~v2() = default;

	friend std::size_t hash_value(const v2& obj) {
		std::size_t seed = 0x379D1D39;
		seed ^= (seed << 6) + (seed >> 2) + 0x6689D690 + hash_value(obj.x);
		seed ^= (seed << 6) + (seed >> 2) + 0x4ABF5DE9 + hash_value(obj.y);
		return seed;
	}

	v2(T x1, T y1, T x2, T y2) {
		x = x2 - x1;
		y = y2 - y1;
	}

	v2(T x_, T y_) {
		x = x_;
		y = y_;
	}

	v2() {
		x = 0;
		y = 0;
	}

	T x;
	T y;

	v2 operator+(const v2 &that) {
		return v2<T>(x + that.x, y + that.y);
	}

	v2 operator+=(const v2 &that) {
		return v2<T>(x + that.x, y + that.y);
	}

	v2 operator-(const v2 &that) {
		return v2<T>(x - that.x, y - that.y);
	}

	v2 operator-=(const v2 &that) {
		return v2<T>(x - that.x, y - that.y);
	}

	// Operator : Scalarproduct (dotproduct)
	T operator*(const v2 &that) {
		return (x * that.x) + (y * that.y);
	}

	virtual v2 operator*(const T &k) {
		return v2<T>(k * x, k * y);
	}

	v2 operator*=(const v2 &that) {
		return v2<T>(x, y) * that;
	}

	v2 operator*=(const double k) {
		return v2<T>(x, y) * k;
	}

	int operator/(const v2 &that) {
		return v2(x, y) * that == 0 ? 1 : 0;
	}

	int operator<(const v2 &that) {
		return len() < that.len();
	}

	int operator>(const v2 &that) {
		return len() > that.len();
	}

	int operator==(const v2 &that) {
		return x == that.x & y == that.y;
	}

	v2 operator!() {
		return v2<T>(-x, -y);
	}

	v2 project_onto(const v2 &that) {
		return this * (v2(x, y) * that / pow(len(), 2));
	}

	T project_len(const v2 &that) {
		return abs(v2(x, y) * that / len());
	}

	T angle(const v2 &that) {
		return (v2(x, y) * that) / (len() * that.len());
	}

	virtual T len() const {
		return abs(sqrt((x * x) + (y * y)));
	}

	v2 cross() {
		return v2<T>(-y, x);
	}

	T det(const v2 &that) {
		return cross() * that;
	}

	int parallel(const v2 &that) {
		return det(that) == 0 ? 1 : 0;
	}

	virtual bool hasValue() {
		return this->x != 0 || this->y != 0;
	}

};

template<class T>
ostream &operator<<(ostream &stream, v2<T> v) {
	stream << '[' << v.x << ',' << v.y << ']';
	return stream;
}

template<class T>
class v3 : public v2<T> {
public:

	v3(T x1, T y1, T z1, T x2, T y2, T z2) : v2<T>(x1, y1, x2, y2) {
		z = z2 - z1;
	}

	v3(T x_, T y_, T z_) : v2<T>(x_, y_) {
		z = z_;
	}

	v3() : v2<T>(0, 0) {
		z = 0;
	}

	T z;

	friend std::size_t hash_value(const v3& obj) {
		std::size_t seed = 0x021F39B2;
		seed ^= (seed << 6) + (seed >> 2) + 0x0AB3178C + hash_value(static_cast<const v2<T>&>(obj));
		seed ^= (seed << 6) + (seed >> 2) + 0x41691B84 + hash_value(obj.z);
		return seed;
	}

	v3 operator+(const v3 &that) {
		return v3<T>(this->x + that.x, this->y + that.y, this->z + that.z);
	}

	v3 operator-(const v3 &that) {
		return v3<T>(this->x - that.x, this->y - that.y, this->z - that.z);
	}

	// Operator : Scalarproduct (dotproduct)
	T operator*(const v3 &that) override {
		return (this->x * that.x) + (this->y * that.y) + (this->z * that.z);
	}

	v3 operator*(const double k) {
		return v3<T>(k * this->x, k * this->y, k * this->z);
	}

	T len() const override {
		return abs(sqrt(this->x * this->x + this->y * this->y + this->z * this->z));
	}

	v3 cross(const v3 &that) {
		return v3<T>(this->y * that.z - this->z * this->y, this->z * that.x - this->x * that.z, this->x * that.y - that.y - this->x);
	}

	T parallelogram_area(const v3 &that) {
		return abs(cross(that).len());
	}

	T angle(const v3 &that) {
		return v3<T>(this->x, this->y, this->z) * that / (len() * that.len());
	}

	bool hasValue() override {
		return this->x != 0 || this->y != 0 || this->z != 0;
	}

};

template<class T>
ostream &operator<<(ostream &stream, v3<T> v) {
	stream << '[' << v.x << ',' << v.y << ',' << v.z << ']';
	return stream;
}


#endif //VEC_H
