//          Copyright Rudy Alex Kohn 2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once
#include <string>
#include <vector>
#include <memory>
#ifdef __unix__
#include <direct.h>
#else
#include <windows.h>
#endif

namespace file {

    using path_legal = struct path {
        std::vector<std::string> legal_part;
        std::vector<std::string> complete_path;
        std::string org;
        bool any_legal;

        explicit path(std::string org)
            : org(org), any_legal(false) { }
    };

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
     * \brief Relaxed method to check if two files are identical.
     * Note: This is aimed towards minor file and low file count.
     * \param file_one The first filename
     * \param file_two The second filename
     * \return true if files are identical, otherwise false
     */
    bool are_files_equal(std::string& file_one, std::string& file_two);

    /**
     * \brief Checks if a string can be used to create a file or a folder with that name
     * \param name The name string to check
     * \return true if it seems legal, otherwise false
     */
    bool is_name_legal(const std::string& name);

    /**
     * \brief Checks a entire path for legal parts, will split the path into seperate sub folder
     * entities in seperate vector, while keeping a corresponding "full" path for each as well,
     * as long as the path is legal. Path seperation is also handled gracefully.
     * \param output The construct containing the original string, and will be populated with
     * the resulting data.
     * \return true if _any_ part of the parsed data is legal, otherwise false.
     */
    bool is_path_legal(const std::shared_ptr<path_legal> &output);
}
