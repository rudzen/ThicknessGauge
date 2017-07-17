#pragma once
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <functional>
#include <cctype>

/**
 * \brief Contains helper functionality for string manipulation
 */
namespace str {

    /**
     * \brief Replaces first occurrence of from in str with to
     * \param str The string to check for occurrence
     * \param from The string to be replaced in str
     * \param to The string to replace from in str
     * \return true if replaced, otherwise false
     */
    inline bool replace(std::string& str, const std::string& from, const std::string& to) {
        auto start_pos = str.find(from);
        if (start_pos == std::string::npos) {
            return false;
        }
        str.replace(start_pos, from.length(), to);
        return true;
    }

    /**
     * \brief Replaces last occurrence of from in str with to
     * \param str The string to check for occurrence
     * \param from The string to be replaced in str
     * \param to The string to replace from in str
     * \return true if replaced, otherwise false
     */
    inline bool replace_last(std::string& str, const std::string& from, const std::string& to) {
        auto start_pos = str.rfind(from);
        if (start_pos == std::string::npos) {
            return false;
        }
        str.replace(start_pos, from.length(), to);
        return true;
    }

    /**
     * \brief Check a string for specific prefix
     * \param str The string to check
     * \param prefix The prefix to check for
     * \return true if prefix was found, otherwise false
     */
    inline bool begin_with(const std::string& str, const std::string& prefix) {
        return str.size() >= prefix.size() && str.compare(0, prefix.size(), prefix) == 0;
    }

    /**
     * \brief Check a string for specific suffix
     * \param str The string to check
     * \param suffix The suffix to check for
     * \return true if suffix was found, otherwise false
     */
    inline bool end_with(const std::string& str, const std::string& suffix) {
        return str.size() >= suffix.size() && str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
    }

    /**
    * Converts arbitrary datatypes (all datatypes which support the << streamoperator) into std::string
    */
    template <typename T>
    std::string to_string(T value) {
        std::ostringstream ss;
        ss << value;
        return ss.str();
    }

    /**
    * Converts strings to arbitrary datatypes (using the << stream operator)
    * @param str The std::string that should be converted
    */
    template <typename T>
    static T parse(const std::string& str) {
        T result;
        std::istringstream(str) >> result;
        return result;
    }

    /**
    * Converts std::strings to arbitrary datatypes
    * @param str The std::string that should be converted
    */
    template <typename T>
    T parse(const std::string& str, bool na) {
        return parse<T>(str);
    }

    template <typename T>
    std::vector<T> parse_array(const std::string& str) {
        std::vector<T> elems;
        std::istringstream f(str);
        std::string s;
        while (getline(f, s, ':')) {
            elems.emplace_back(parse<T>(s));
        }
        return elems;
    }

    /**
    * Converts null terminated std::string to upper case
    */
    inline void to_upper(char* s) {
        for (size_t i = 0; s[i]; i++) {
            s[i] = toupper(s[i]);
        }
    }

    /**
    * Converts std::std::string to upper case
    */
    inline void to_upper(std::string& str) {
        for (size_t i = 0; str[i]; i++) {
            str[i] = toupper(str[i]);
        }
    }

    /**
    * Converts null terminated std::string to lower case
    */
    inline void to_lower(char* s) {
        for (size_t i = 0; s[i]; i++) {
            s[i] = tolower(s[i]);
        }
    }

    /**
    * Converts std::std::string to lower case
    */
    inline void to_lower(std::string& str) {
        for (size_t i = 0; str[i]; i++) {
            str[i] = tolower(str[i]);
        }
    }

    /**
    * Trims from start
    */
    inline std::string& ltrim(std::string& s) {
        s.erase(s.begin(), find_if(s.begin(), s.end(), not1(std::ptr_fun<int, int>(isspace))));
        return s;
    }

    /**
    * Trims from end
    */
    inline std::string& rtrim(std::string& s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), not1(std::ptr_fun<int, int>(isspace))).base(), s.end());
        return s;
    }

    /**
    * Trims from both ends
    */
    inline std::string& trim(std::string& s) {
        return ltrim(rtrim(s));
    }

    /**
    * Join vector
    */
    template <typename T>
    std::string join(const std::vector<T>& v, const std::string& token) {
        std::ostringstream result;
        std::for_each(v.begin(), v.end(), [&](const T& i) {
                  result << i;
                  if (i != v.end()) {
                      result << token;
                  }
              });
        return result.str();
    }

    /**
    * Split a std::string
    */
    inline std::vector<std::string> split(const std::string& str, char delim) {
        std::vector<std::string> elems;
        elems.reserve(str.size());

        std::stringstream ss(str);

        std::string item;
        item.reserve(str.size());

        while (getline(ss, item, delim)) {
            elems.emplace_back(std::move(item));
        }

        elems.shrink_to_fit();
        return elems;
    }

    template <>
    inline std::string parse(const std::string& str) {
        return str;
    }

    template <>
    inline const char* parse(const std::string& str) {
        return str.c_str();
    }

    template <>
    inline bool parse(const std::string& str, bool na) {
        auto s = str;
        to_lower(s);

        if (s == "on" || s == "yes") {
            return true;
        }

        return parse<bool>(str);
    }

}
