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
*/
#pragma once
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <locale>
#include <functional>

/*
 * The final StringTools library, no need to look any further!!!
 */

namespace utils {

    using namespace std;

    /**
    * A collection of useful string functions based on std::string
    */
    class StringTools : public std::string {
    private:

        // singleton stuff	
        StringTools() {
        }

        StringTools(const StringTools&) = delete;
        StringTools& operator=(const StringTools&) = delete;

    public: // singleton stuff

        static StringTools& instance() {
            static StringTools s;
            return s;
        }

        // end of singleton stuff


        /**
        * Replaces first occurrence of from in str with to
        *
        * Taken from http://stackoverflow.com/questions/3418231/c-replace-part-of-a-string-with-another-string
        */
        static bool replace(string& str, const string& from, const string& to) {
            auto start_pos = str.find(from);
            if (start_pos == string::npos)
                return false;
            str.replace(start_pos, from.length(), to);
            return true;
        }

        /**
        * Replaces last occurrence of from in str with to
        */
        static bool replaceLast(string& str, const string& from, const string& to) {
            auto start_pos = str.rfind(from);
            if (start_pos == string::npos)
                return false;
            str.replace(start_pos, from.length(), to);
            return true;
        }

        static bool startsWith(const string& str, const string& prefix) {
            return str.size() >= prefix.size() && str.compare(0, prefix.size(), prefix) == 0;
        }

        /**
        * Checks a string for specific suffix
        *
        * Taken from: http://stackoverflow.com/questions/20446201/how-to-check-if-string-ends-with-txt
        */
        static bool endsWith(const string& str, const string& suffix) {
            return str.size() >= suffix.size() && str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
        }

        /**
        * Converts arbitrary datatypes (all datatypes which support the << stream
        * operator) into std::string
        */
        template <typename T>
        static string toString(T value) {
            ostringstream ss;
            ss << value;
            return ss.str();
        }

        /**
        * Converts strings to arbitrary datatypes (using the << stream operator)
        *
        * @param str The string that should be converted
        */
        template <typename T>
        static T parse(const string& str) {
            T result;
            std::istringstream(str) >> result;
            return result;
        }

        /**
        * Converts strings to arbitrary datatypes
        *
        * @param str The string that should be converted
        * @param advanced True of advanced conversions should be enabled
        */
        template <typename T>
        static T parse(const string& str, bool advanced) {
            // By default the advanced mode is disabled for all datatypes
            return parse<T>(str);
        }

        template <typename T>
        inline
        static vector<T> parseArray(const string& str) {
            vector<T> elems;
            std::istringstream f(str);
            string s;
            while (getline(f, s, ':'))
                elems.emplace_back(parse<T>(s));

            return elems;
        }

        /**
        * Converts null terminated string to upper case
        */
        static void toUpper(char* s) {
            for (size_t i = 0; s[i]; i++)
                s[i] = toupper(s[i]);
        }

        /**
        * Converts std::string to upper case
        */
        static void toUpper(string& str) {
            for (size_t i = 0; str[i]; i++)
                str[i] = toupper(str[i]);
        }

        /**
        * Converts null terminated string to lower case
        */
        static void toLower(char* s) {
            for (size_t i = 0; s[i]; i++)
                s[i] = tolower(s[i]);
        }

        /**
        * Converts std::string to lower case
        */
        static void toLower(string& str) {
            for (size_t i = 0; str[i]; i++)
                str[i] = tolower(str[i]);
        }

        /**
        * Trims from start
        *
        * Taken from http://stackoverflow.com/questions/216823/whats-the-best-way-to-trim-stdstring
        */
        static string& ltrim(string& s) {
            s.erase(s.begin(), find_if(s.begin(), s.end(), not1(ptr_fun<int, int>(isspace))));
            return s;
        }

        /**
        * Trims from end
        *
        * Taken from http://stackoverflow.com/questions/216823/whats-the-best-way-to-trim-stdstring
        */
        static string& rtrim(std::string& s) {
            s.erase(find_if(s.rbegin(), s.rend(), not1(ptr_fun<int, int>(isspace))).base(), s.end());
            return s;
        }

        /**
        * Trims from both ends
        *
        * Taken from http://stackoverflow.com/questions/216823/whats-the-best-way-to-trim-stdstring
        */
        static string& trim(string& s) {
            return ltrim(rtrim(s));
        }

        /**
        * Join vector
        *
        * Taken from http://dracoater.blogspot.com/2010/03/joining-vector-and-splitting-string-in.html
        */
        template <typename T>
        static string join(const vector<T>& v, const string& token) {
            std::ostringstream result;
            for (typename vector<T>::const_iterator i = v.begin(); i != v.end(); ++i) {
                if (i != v.begin())
                    result << token;
                result << *i;
            }

            return result.str();
        }

        /**
        * Split a string
        *
        * Taken from http://stackoverflow.com/questions/236129/how-to-split-a-string-in-c
        */
        static vector<string> split(const string& str, char delim) {
            vector<string> elems;
            stringstream ss(str);

            string item;
            while (getline(ss, item, delim))
                elems.emplace_back(item);

            return elems;
        }
    };

    template <>
    inline
    string StringTools::parse(const string& str) {
        return str;
    }

    template <>
    inline
    const char* StringTools::parse(const string& str) {
        return str.c_str();
    }

    template <>
    inline
    bool StringTools::parse(const string& str, bool advanced) {
        auto s = str;
        toLower(s);

        if (s == "on" || s == "yes")
            return true;

        return parse<bool>(str);
    }

}
