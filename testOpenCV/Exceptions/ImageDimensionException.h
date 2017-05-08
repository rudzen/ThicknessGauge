#pragma once

#include <exception>
#include <stdexcept>

class ImageDimensionException : public std::exception {

public:

	explicit ImageDimensionException(std::string message) : message_(message) { }

	const char* what() const throw() override {
		return message_.c_str();
	}

private:

	std::string message_;

};