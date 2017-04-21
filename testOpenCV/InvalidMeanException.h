#pragma once

#include <stdexcept>
#include <sstream>

class InvalidMeanException : public std::exception {

public:

	explicit InvalidMeanException(std::string message) : message_(message) { }

	const char* what() const throw() override {
		return message_.c_str();
	}

private:

	std::string message_;

};