//          Copyright Rudy Alex Kohn 2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include "filesystem.h"
#include <vector>
#include <sstream>
#include "stl.h"

namespace file {

    constexpr char path_seperator =
#ifdef _WIN32
            '\\';
#else
                            '/';
#endif

    using path_legal = struct path {
        std::vector<std::string> legal_part;

        std::vector<std::string> complete_path;

        bool any_legal;
    };

    bool create_directory(const std::string& pathname) {
#ifdef __unix__
        if (!is_name_legal(std::string(r.begin(), r.end())))
            return false;
		mkdir(pathname.c_str());
        return true;
#else
        auto slength = static_cast<int>(pathname.length()) + 1;
        auto len = MultiByteToWideChar(CP_ACP, 0, pathname.c_str(), slength, nullptr, 0);
        auto buf = new wchar_t[len];
        MultiByteToWideChar(CP_ACP, 0, pathname.c_str(), slength, buf, len);
        std::wstring r(buf);
        delete[] buf;
        if (!is_name_legal(std::string(r.begin(), r.end())))
            return false;
        auto done = CreateDirectory(r.c_str(), nullptr);
        return done;
#endif
    }

    bool is_directory(const std::string& pathname) {
        struct stat info;
        if (stat(pathname.c_str(), &info) != 0)
            return false;
        if (info.st_mode & S_IFDIR)
            return true;
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

    inline
    path_legal is_path_legal(const std::string& full_path) {
        path_legal current;

        current.any_legal = false;

        if (full_path.empty())
            return current;

        std::istringstream ss(full_path);
        std::string token;

        token.reserve(full_path.size());

        while (getline(ss, token, path_seperator))
            current.complete_path.emplace_back(std::move(token));

        token.clear();

        for (const auto& p : current.complete_path) {
            token += p;
            token += path_seperator;
            if (!is_directory(token))
                break;
            current.legal_part.emplace_back(token);
            current.any_legal = true;
        }

        return current;
    }

    inline
    bool is_name_legal(const std::string& name) {
        for (const auto& c : name)
            if (illegal_chars.find(c) != std::string::npos)
                return false;
        return true;
    }
}
