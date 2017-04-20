#pragma once

#include <stdexcept>

class CaptureFailException : public exception {

public:

	explicit CaptureFailException(string message) : m_message(message) { }

	const char* what() const throw() override {
		return m_message.c_str();
	}

private:

	string m_message;

};