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
// Can still be optimized quite a bit

#pragma once

#include <ostream>

using namespace std;

template <typename T>
class v2 {
public:
    virtual ~v2() = default;

    v2(T x1, T y1, T x2, T y2) {
        x = x2 - x1;
        y = y2 - y1;
    }

    v2(T x_, T y_) {
        x = x_;
        y = y_;
    }

    v2(const v2& other)
        : x(other.x)
        , y(other.y) { }

    v2(v2&& other) noexcept
        : x(std::move(other.x))
        , y(std::move(other.y)) { }

    v2& operator=(const v2& other) {
        if (this == &other)
            return *this;
        x = other.x;
        y = other.y;
        return *this;
    }

    v2& operator=(v2&& other) noexcept {
        if (this == &other)
            return *this;
        x = std::move(other.x);
        y = std::move(other.y);
        return *this;
    }

    v2() {
        x = 0;
        y = 0;
    }

    // OpenCV copy constructor
#ifdef CV_VERSION
    explicit v2(cv::Point_<T>& p) {
        x = p.x;
        y = p.y;
    }

    template <typename Y>
    explicit v2(cv::Point_<Y>& p) {
        static_assert(std::is_convertible<Y, T>::value, "Invalid type.");
        x = static_cast<T>(p.x);
        y = static_cast<T>(p.y);
    }

    explicit v2(cv::Vec<T, 2>& p) {
        x = p[0];
        y = p[1];
    }

    template <typename Y>
    explicit v2(cv::Vec<T, 2>& p) {
        static_assert(std::is_convertible<Y, T>::value, "Invalid type.");
        x = static_cast<T>(p.x);
        y = static_cast<T>(p.y);
    }

    v2 operator+(const cv::Point_<T>& that) {
        return v2<T>(x + that.x, y + that.y);
    }

    v2 operator+(const cv::Vec<T, 2>& that) {
        return v2<T>(x + that.x, y + that.y);
    }

#endif
    T x;

    T y;

    friend bool operator==(const v2& lhs, const v2& rhs) {
        return lhs.x == rhs.x
                && lhs.y == rhs.y;
    }

    friend bool operator!=(const v2& lhs, const v2& rhs) {
        return !(lhs == rhs);
    }

    v2 operator+(const v2& that) {
        return v2<T>(x + that.x, y + that.y);
    }

    void operator+=(const v2& that) {
        this->x += that.x;
        this->y += that.y;
    }

    v2 operator-(const v2& that) {
        return v2<T>(x - that.x, y - that.y);
    }

    void operator-=(const v2& that) {
        this->x -= that.x;
        this->y -= that.y;
    }

    T operator*(const v2& that) {
        // Operator : Scalarproduct (dotproduct)
        return (x * that.x) + (y * that.y);
    }

    virtual v2 operator*(const T& k) {
        return v2<T>(k * x, k * y);
    }

    void operator*=(const v2& that) {
        this->x *= that->x;
        this->y *= that->y;
    }

    template <typename Y>
    void operator*=(Y k) {
        static_assert(std::is_arithmetic<Y>::value, "Not a valid type");
        this->x *= k;
        this->y *= k;
    }

    template <typename Y>
    bool operator/(const v2<Y>& that) {
        static_assert(std::is_arithmetic<Y>::value, "Not a valid type");
        return this * that == 0 ? true : false;
    }

    template <typename Y>
    bool operator<(const v2& that) {
        static_assert(std::is_arithmetic<Y>::value, "Not a valid type");
        return len() < that.len();
    }

    template <typename Y>
    bool operator>(const v2& that) {
        return len() > that.len();
    }

    bool operator==(const v2& that) {
        return x == that.x & y == that.y;
    }

    void operator!() {
        x = -x;
        y = -y;
    }

    v2 project_onto(const v2& that) {
        return this * (v2(x, y) * that / pow(len(), 2));
    }

    T project_len(const v2& that) {
        return abs(v2(x, y) * that / len());
    }

    T angle(const v2& that) {
        return (this * that) / (len() * that.len());
    }

    virtual T len() const {
        return static_cast<T>(abs(sqrt((x * x) + (y * y))));
    }

    v2 cross() {
        return v2<T>(-y, x);
    }

    void crossTo(v2& that) {
        that.x = -this->y;
        that.y = this->x;
    }

    T det(const v2& that) {
        return cross() * that;
    }

    int parallel(const v2& that) {
        return det(that) == 0 ? 1 : 0;
    }

    virtual bool real() {
        return this->x + this->y == 0;
    }

    friend std::size_t hash_value(const v2& obj) {
        std::size_t seed = 0x379D1D39;
        seed ^= (seed << 6) + (seed >> 2) + 0x6689D690 + hash_value(obj.x);
        seed ^= (seed << 6) + (seed >> 2) + 0x4ABF5DE9 + hash_value(obj.y);
        return seed;
    }

};

template <class T>
ostream& operator<<(ostream& stream, v2<T> v) {
    stream << '[' << v.x << ',' << v.y << ']';
    return stream;
}

template <class T>
class v3 : public v2<T> {
public:

    v3(T x1, T y1, T z1, T x2, T y2, T z2)
        : v2<T>(x1, y1, x2, y2) {
        z = z2 - z1;
    }

    v3(T x_, T y_, T z_)
        : v2<T>(x_, y_) {
        z = z_;
    }

    v3()
        : v2<T>(0, 0) {
        z = 0;
    }

    v3(const v3& other)
        : v2<T>(other)
        , z(other.z) { }

    v3(v3&& other) noexcept
        : v2<T>(std::move(other))
        , z(std::move(other.z)) { }

    v3& operator=(const v3& other) {
        if (this == &other)
            return *this;
        v2<T>::operator =(other);
        z = other.z;
        return *this;
    }

    v3& operator=(v3&& other) noexcept {
        if (this == &other)
            return *this;
        v2<T>::operator =(std::move(other));
        z = std::move(other.z);
        return *this;
    }

    T z;

    friend bool operator==(const v3& lhs, const v3& rhs) {
        return static_cast<const v2<T>&>(lhs) == static_cast<const v2<T>&>(rhs)
                && lhs.z == rhs.z;
    }

    friend bool operator!=(const v3& lhs, const v3& rhs) {
        return !(lhs == rhs);
    }

    virtual v3 operator+(const v3& that) {
        return v3<T>(this->x + that.x, this->y + that.y, this->z + that.z);
    }

    v3 operator-(const v3& that) {
        return v3<T>(this->x - that.x, this->y - that.y, this->z - that.z);
    }

    // Operator : Scalarproduct (dotproduct)
    //T operator*(const v3 &that) override {
    //	return (this->x * that.x) + (this->y * that.y) + (this->z * that.z);
    //}

    v3 operator*(const double k) {
        return v3<T>(static_cast<T>(k * this->x), static_cast<T>(k * this->y), static_cast<T>(k * this->z));
    }

    T len() const override {
        return abs(sqrt(this->x * this->x + this->y * this->y + this->z * this->z));
    }

    v3 cross(const v3& that) {
        return v3<T>(this->y * that.z - this->z * this->y, this->z * that.x - this->x * that.z, this->x * that.y - that.y - this->x);
    }

    T parallelogram_area(const v3& that) {
        return abs(cross(that).len());
    }

    T angle(const v3& that) {
        return v3<T>(this->x, this->y, this->z) * that / (len() * that.len());
    }

    bool real() override {
        return this->x + this->y + this->z == 0;
    }

    friend std::size_t hash_value(const v3& obj) {
        std::size_t seed = 0x021F39B2;
        seed ^= (seed << 6) + (seed >> 2) + 0x0AB3178C + hash_value(static_cast<const v2<T>&>(obj));
        seed ^= (seed << 6) + (seed >> 2) + 0x41691B84 + hash_value(obj.z);
        return seed;
    }
};

template <class T>
ostream& operator<<(ostream& stream, v3<T> v) {
    stream << '[' << v.x << ',' << v.y << ',' << v.z << ']';
    return stream;
}

/////////////////////////////////////

template <class T>
class v4 : public v3<T> {
public:

    v4(T x1, T y1, T z1, T w1, T x2, T y2, T z2, T w2)
        : v3<T>(x1, y1, z1, x2, y2, z2) {
        w = z2 - z1;
    }

    v4(T x_, T y_, T z_, T w_)
        : v3<T>(x_, y_, z_) {
        w = w_;
    }

    v4()
        : v3<T>(0, 0, 0) {
        w = 0;
    }

    v4(const v4& other)
        : v3<T>(other)
        , w(other.w) { }

    v4(v4&& other) noexcept
        : v3<T>(std::move(other))
        , w(std::move(other.w)) { }

    v4& operator=(const v4& other) {
        if (this == &other)
            return *this;
        v3<T>::operator =(other);
        w = other.w;
        return *this;
    }

    v4& operator=(v4&& other) noexcept {
        if (this == &other)
            return *this;
        v3<T>::operator =(std::move(other));
        w = std::move(other.w);
        return *this;
    }

    T w;

    friend bool operator==(const v4& lhs, const v4& rhs) {
        return static_cast<const v3<T>&>(lhs) == static_cast<const v3<T>&>(rhs)
                && lhs.w == rhs.w;
    }

    friend bool operator!=(const v4& lhs, const v4& rhs) {
        return !(lhs == rhs);
    }

    v4 operator+(const v4& that) {
        return v4<T>(this->x + that.x, this->y + that.y, this->z + that.z, this->w + that.w);
    }

    v4 operator-(const v4& that) {
        return v4<T>(this->x - that.x, this->y - that.y, this->z - that.z, this->w - that.w);
    }

    // Operator : Scalarproduct (dotproduct)
    T operator*(const v4& that) override {
        return (this->x * that.x) + (this->y * that.y) + (this->z * that.z) + (this->w * that->w);
    }

    v4 operator*(const double k) {
        return v4<T>(static_cast<T>(k * this->x), static_cast<T>(k * this->y), static_cast<T>(k * this->z), static_cast<T>(k * this->w));
    }

    T len() const override {
        return abs(sqrt(this->x * this->x + this->y * this->y + this->z * this->z));
    }

    v4 cross(const v4& that) {
        return v4<T>(this->y * that.z - this->z * this->y, this->z * that.x - this->x * that.z, this->x * that.y - that.y - this->x);
    }

    T parallelogram_area(const v4& that) {
        return abs(cross(that).len());
    }

    T angle(const v4& that) {
        return v4<T>(this->x, this->y, this->z) * that / (len() * that.len());
    }

    bool real() override {
        return this->x == 0 || this->y == 0 || this->z == 0 || this->w == 0;
    }

    friend std::size_t hash_value(const v4& obj) {
        std::size_t seed = 0x021F39B2;
        seed ^= (seed << 6) + (seed >> 2) + 0x0AB3178C + hash_value(static_cast<const v3<T>&>(obj));
        seed ^= (seed << 6) + (seed >> 2) + 0x41691B84 + hash_value(obj.w);
        return seed;
    }
};

template <class T>
ostream& operator<<(ostream& stream, v4<T> v) {
    stream << '[' << v.x << ',' << v.y << ',' << v.z << ',' << v.w << ']';
    return stream;
}
