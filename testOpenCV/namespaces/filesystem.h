#pragma once
#include <string>

namespace file {
	
	/**
	 * \brief Checks if a file exists.
	 * \param name The filename to check
	 * \return true if exists, otherwise false
	 */
	bool isFile(const std::string& name) {
#ifdef _MSC_VER
		
		// should only be POSIX, but M$ is weird..

		struct stat buffer;
		return (stat(name.c_str(), &buffer) == 0);

#else
		// this was originaly only for _MSC_VER, but alas!
		if (auto file = fopen(name.c_str(), "r")) {
			fclose(file);
			return true;
		}
		return false;

#endif
	}

	bool isNameLegal(const std::string& name);

}
