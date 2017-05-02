#pragma once

#include <stdexcept>

class TestException : public std::exception {

public:

	explicit TestException(std::string message) : message_(message) { }

	const char* what() const throw() override {
		return message_.c_str();
	}

private:

	std::string message_;

};