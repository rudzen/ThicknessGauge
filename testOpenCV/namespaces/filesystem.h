#pragma once
#include <string>
#ifdef __unix__
#include <direct.h>
#else
#include <windows.h>
#endif

namespace file {
	
	const std::string illegal_chars = "\\/:?\"<>|";

	bool createDirectory(const std::string& pathname);

	bool isDirectory(const std::string& path);

	/**
	 * \brief Checks if a file exists.
	 * \param name The filename to check
	 * \return true if exists, otherwise false
	 */
	bool isFile(const std::string& name);

	bool isNameLegal(const std::string& name);

}
