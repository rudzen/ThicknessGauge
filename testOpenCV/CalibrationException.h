#pragma once

#include <stdexcept>

class CalibrationException : public std::exception {

public:

	explicit CalibrationException(std::string message) : message_(message) { }

	const char* what() const throw() override {
		return message_.c_str();
	}

private:

	std::string message_;

};