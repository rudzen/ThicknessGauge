/**
MIT License

Copyright (c) 2016 Rudy Alex Kohn

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#pragma once

#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>
#include <string.h>
#include <functional>

#if UINTPTR_MAX == UINT64_MAX
//64 bit
#ifndef _MSC_VER
typedef __int128_t i128;
#endif
#else
//32 bit
    typedef unsigned long long i128;
#endif

class Ztring : public std::string {
public:
    explicit Ztring(const std::string& s)
        : std::string(s) {
    }

    explicit Ztring(const char* s)
        : std::string(s) {
    }

    Ztring() { }

    bool endsWith(const std::string& ending) {
        return ending.size() <= this->size() && equal(ending.rbegin(), ending.rend(), this->rbegin());
    }

    Ztring& trim() {
        trimLeft();
        trimRight();
        return *this;
    }

    Ztring& trimLeft() {
        this->erase(this->begin(), find_if(this->begin(), this->end(), not1(std::ptr_fun<int, int>(static_cast<int (*)(int)>(isspace)))));
        return *this;

    }

    Ztring& trimRight() {
        this->erase(find_if(this->rbegin(), this->rend(), not1(std::ptr_fun<int, int>(static_cast<int (*)(int)>(isspace)))).base(), this->end());
        return *this;
    }

    Ztring& replace(const char c1, const char c2) {
        for (size_t i = 0; i < size(); i++)
            if (at(i) == c1)
                at(i) = c2;
        return *this;
    }

    Ztring& replace(const std::string& s1, const std::string& s2) {
        size_t a;
        while ((a = find(s1)) != npos)
            std::string::replace(a, s1.size(), s2);
        return *this;
    }

    Ztring& toUpper() {
        transform(begin(), end(), begin(), ::toupper);
        return *this;
    }

    Ztring& toLower() {
        transform(begin(), end(), begin(), ::tolower);
        return *this;
    }

    template <class T>
    explicit Ztring(T d, const std::string tohex = "") {
        std::stringstream ss;
        if (tohex == "int64tohex") {
            std::stringstream ss2;
            ss2 << std::hex << d;
            ss << "0x";
            for (size_t i = 0; i < 16 - ss2.str().length(); i++)
                ss << "0";
            ss << std::hex << d << "ULL";
        } else if (tohex == "int32tohex") {
            std::stringstream ss2;
            ss2 << std::hex << d;
            ss << "0x";
            for (size_t i = 0; i < 8 - ss2.str().length(); i++)
                ss << "0";
            ss << std::hex << d;
        } else {
            ss << d;
        }
        assign(ss.str());
    }

    static int stoi(const std::string& s) {
        if (s.empty())
            return 0;
        return std::stoi(s);
    }

    /**
 * Creates a string containing spaces
 *
 * @param amount : how many spaces to make
 * @return the string containing amount spaces
 */
    static std::string space(size_t amount) {
        return replicate(' ', amount);
    }

    /**
     * Replicate a char
     *
     * @param c : the char to replicate
     * @param amount : how many time to replicate
     * @return the string containing amount char
     */
    static std::string replicate(char c, size_t amount) {
        return std::string(amount, c);
    }

    /**
     * Replaces a piece of an existing string with another string.<br>
     *
     * @param into : The string to overwrite in
     * @param toInsert : The string which is put in the into string.
     * @param startPos : Start position where it should be inserted at.
     * @return The resulting string.
     */
    template <typename T>
    static std::string overwrite(std::string into, std::string toInsert, T startPos) {
        static_assert(std::is_integral<T>::value, "Wrong type.");
        auto len = into.length();

        if (len == 0)
            return toInsert;

        auto lenInsert = toInsert.length();

        if (lenInsert == 0)
            return into;

        std::ostringstream ss;

        // no fault check from here!

        auto start = static_cast<size_t>(startPos);

        ss << into.substr(0, startPos - 1);
        ss << toInsert;

        if (startPos - 1 + lenInsert <= len - 1)
            ss << into.substr(ss.str().length(), into.length());

        return ss.str();
    }

};
