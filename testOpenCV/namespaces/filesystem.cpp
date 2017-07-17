//          Copyright Rudy Alex Kohn 2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include "filesystem.h"
#include <algorithm>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include "calc.h"

namespace file {

    constexpr char path_seperator =
#ifdef _WIN32
            '\\';
#else
        '/';
#endif

    bool create_directory(const std::string& pathname) {
#ifdef __unix__
        if (!is_name_legal(std::string(r.begin(), r.end()))) {
            return false;
        }
		mkdir(pathname.c_str());
        return true;
#else
        auto slength = static_cast<int>(pathname.length()) + 1;
        auto len = MultiByteToWideChar(CP_ACP, 0, pathname.c_str(), slength, nullptr, 0);
        auto buf = new wchar_t[len];

        MultiByteToWideChar(CP_ACP, 0, pathname.c_str(), slength, buf, len);
        std::wstring r(buf);
        delete[] buf;

        auto done = is_name_legal(std::string(r.begin(), r.end()));

        if (!done) {
            return false;
        }

        done = CreateDirectory(r.c_str(), nullptr);
        return done;
#endif
    }

    bool is_directory(const std::string& pathname) {
        struct stat info;
        if (stat(pathname.c_str(), &info) != 0) {
            return false;
        }
        if (info.st_mode & S_IFDIR) {
            return true;
        }
        return false;
    }

    bool is_file(const std::string& name) {
#ifdef _MSC_VER
        struct stat buffer;
        return (stat(name.c_str(), &buffer) == 0);
#else
		if (auto file = fopen(name.c_str(), "r")) {
			fclose(file);
			return true;
		}
		return false;
#endif
    }

    bool are_files_equal(std::string& file_one, std::string& file_two) {

        std::ifstream in1(file_one, std::ios::binary);
        std::ifstream in2(file_two, std::ios::binary);

        auto size1 = in1.seekg(0, std::ifstream::end).tellg();
        auto size2 = in2.seekg(0, std::ifstream::end).tellg();

        if (size1 != size2) {
            in1.close();
            in2.close();
            return false;
        }

        in1.seekg(0, std::ifstream::beg);
        in2.seekg(0, std::ifstream::beg);

        static const size_t BLOCKSIZE = 4096;
        size_t remaining = size1;

        auto identical = true;

        while (remaining) {
            char buffer1[BLOCKSIZE], buffer2[BLOCKSIZE];
            auto size = static_cast<size_t>(std::min(BLOCKSIZE, remaining));

            in1.read(buffer1, size);
            in2.read(buffer2, size);

            if (memcmp(buffer1, buffer2, size) != 0) {
                identical = false;
                break;
            }

            remaining -= size;
        }

        in1.close();
        in2.close();

        return identical;
    }

    bool is_path_legal(const std::shared_ptr<file::path_legal>& output) {

        output->any_legal = false;

        if (output->org.empty())
            return false;

        std::istringstream ss(output->org);
        std::string token;

        token.reserve(output->org.size());

        while (getline(ss, token, path_seperator)) {
            output->complete_path.emplace_back(std::move(token));
        }

        token.clear();

        for (const auto& p : output->complete_path) {
            token += p;
            token += path_seperator;
            if (!is_directory(token)) {
                break;
            }
            output->legal_part.emplace_back(token);
        }

        output->any_legal = output->legal_part.empty() ^ true;

        return output->any_legal;
    }

    inline
    bool is_name_legal(const std::string& name) {
        for (const auto& c : name) {
            if (illegal_chars.find(c) != std::string::npos) {
                return false;
            }
        }
        return true;
    }
}
