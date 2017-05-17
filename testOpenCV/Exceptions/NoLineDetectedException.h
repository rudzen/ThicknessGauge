#pragma once

#include <stdexcept>
#include <sstream>

class NoLineDetectedException : public std::exception {

public:

	explicit NoLineDetectedException(std::string message) : message_(message) { }

	const char* what() const throw() override {
		return message_.c_str();
	}

private:

	std::string message_;

};