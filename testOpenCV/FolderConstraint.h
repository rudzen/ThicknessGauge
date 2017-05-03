#pragma once
#include <tclap/Constraint.h>

class FolderConstraint : public TCLAP::Constraint<std::string> {

	const std::string illegalChars = "\\/:?\"<>|";

	/**
	* \brief Checks if a file exists.
	* \param name The filename to check
	* \return true if exists, otherwise false
	*/
	static bool isFolder(const std::string& name) {
#ifdef _MSC_VER

		// should only be POSIX, but M$ is weird..

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

	bool isNameLegal(const std::string& name) const {
		for (auto it = name.begin(); it < name.end(); ++it) {
			auto found = illegalChars.find(*it) != std::string::npos;
			if (found)
				return false;
		}
		return true;
	}

public:

	/**
	* Returns a description of the Constraint.
	*/
	std::string description() const override {
		return "File does not exist or it contains " + illegalChars + " in the name.";
	}

	/**
	* Returns the short ID for the Constraint.
	*/
	std::string shortID() const override {
		return "filename";
	}

	/**
	* The method used to verify that the value parsed from the command
	* line meets the constraint.
	* \param value - The value that will be checked.
	*/
	bool check(const std::string& value) const override {
		return isNameLegal(value) && isFile(value);
	}


};
