#pragma once

#include <stdexcept>

class CaptureFailException : public exception {

public:

	explicit CaptureFailException(string message) : message_(message) { }

	const char* what() const throw() override {
		return message_.c_str();
	}

private:

	string message_;

};