#include "filesystem.h"

namespace file {

#if defined(_MSC_VER) && !defined(inline)
#define inline __forceinline
#endif

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
    bool is_name_legal(const std::string& name) {
        for (auto& c : name) {
            if (illegal_chars.find(c) != std::string::npos)
                return false;
        }
        return true;
    }
}
