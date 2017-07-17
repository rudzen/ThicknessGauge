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
     * \brief Converts arbitrary datatypes (all datatypes which support the << streamoperator) into std::string
     * \tparam T The type of value to convert
     * \param value The value to convert
     * \return The converted value as string
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

    /**
     * \brief Parse a string to array using ':' as seperator
     * \tparam T The return type of vector
     * \param str The string to parse
     * \return The string as array with ':' seperation
     */
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
     * \brief Converts a null terminated string to upper case
     * \param s The null terminated string to convert
     */
    inline void to_upper(char* s) {
        for (size_t i = 0; s[i]; i++) {
            s[i] = toupper(s[i]);
        }
    }

    /**
     * \brief Converts a string to upper case
     * \param str The string to convert
     */
    inline void to_upper(std::string& str) {
        for (size_t i = 0; str[i]; i++) {
            str[i] = toupper(str[i]);
        }
    }

    /**
     * \brief Converts a null terminated string to lower case
     * \param s The null terminated string to convert
     */
    inline void to_lower(char* s) {
        for (size_t i = 0; s[i]; i++) {
            s[i] = tolower(s[i]);
        }
    }

    /**
     * \brief Converts a string to lower case
     * \param str The string to convert
     */
    inline void to_lower(std::string& str) {
        for (size_t i = 0; str[i]; i++) {
            str[i] = tolower(str[i]);
        }
    }

    /**
     * \brief Trims empty spaces from a string from the beginning
     * \param s The string to trim
     * \return The trimmed string
     */
    inline std::string& ltrim(std::string& s) {
        s.erase(s.begin(), find_if(s.begin(), s.end(), not1(std::ptr_fun<int, int>(isspace))));
        return s;
    }

    /**
     * \brief Trims empty spaces from a string from the end
     * \param s The string to trim
     * \return The trimmed string
     */
    inline std::string& rtrim(std::string& s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), not1(std::ptr_fun<int, int>(isspace))).base(), s.end());
        return s;
    }

    /**
     * \brief Trims empty spaces from a string from both ends
     * \param s The string to trim
     * \return The trimmed string
     */
    inline std::string& trim(std::string& s) {
        return ltrim(rtrim(s));
    }

    /**
     * \brief Joins a vector to a string with a specified delimiter
     * \tparam T The type of vector
     * \param v The vector to join
     * \param token The seperator for each part of the vector
     * \return The resulting string
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
     * \brief Splits a string into a vector with defined delimiter
     * \param str The string to split
     * \param delim The delimier to split with
     * \return The vector of the string that was split
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

    /**
     * \brief Copy a string from reference to value
     * \param str The string to copy
     * \return The copy of the string
     */
    template <>
    inline std::string parse(const std::string& str) {
        return str;
    }

    /**
     * \brief Copy a string to null terminated value
     * \param str The string to copy
     * \return The null terminated copy
     */
    template <>
    inline const char* parse(const std::string& str) {
        return str.c_str();
    }

    /**
     * \brief Parse a boolean string to bool value
     * \param str The string to parse
     * \param na Not used atm
     * \return true if string contained "on", "yes" or "true", otherwise false
     */
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
