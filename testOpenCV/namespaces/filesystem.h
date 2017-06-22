//          Copyright Rudy Alex Kohn 2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once
#include <string>
#ifdef __unix__
#include <direct.h>
#else
#include <windows.h>
#endif

namespace file {

    /**
     * \brief The current known illegal chars
     */
    const std::string illegal_chars = "\\/:?\"<>|";

    /**
     * \brief Creates a directory from specified name
     * Note, this is non-recursive!
     * \param pathname The name of the directory to create
     * \return true if the name is invalid or the directory could not be created
     */
    bool create_directory(const std::string& pathname);

    /**
     * \brief Check is a path is actually an existing directory
     * \param path The path to check
     * \return true if it exists, otherwise false
     */
    bool is_directory(const std::string& path);

    /**
     * \brief Checks if a file exists.
     * \param name The filename to check
     * \return true if exists, otherwise false
     */
    bool is_file(const std::string& name);

    /**
     * \brief Checks if a string can be used to create a file or a folder with that name
     * \param name The name string to check
     * \return true if it seems legal, otherwise false
     */
    bool is_name_legal(const std::string& name);

}
