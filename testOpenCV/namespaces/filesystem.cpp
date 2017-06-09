#include "filesystem.h"


namespace file {

	bool isNameLegal(const std::string& name) {
		const std::string illegal_chars = "\\/:?\"<>|";
		for (auto& c : name) {
			if (illegal_chars.find(c) != std::string::npos)
				return false;
		}
		return true;
	}
}
