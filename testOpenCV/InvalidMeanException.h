#pragma once

#include <stdexcept>
#include <sstream>

class InvalidMeanException : public std::exception {

public:

	explicit InvalidMeanException(std::string message) : m_message(message) { }

	const char* what() const throw() override
	{
		return m_message.c_str();
	}

private:

	std::string m_message;

};