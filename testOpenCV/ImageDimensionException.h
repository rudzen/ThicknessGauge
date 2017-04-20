#pragma once

#include <exception>
#include <stdexcept>

class ImageDimensionException : public std::exception {

public:

	explicit ImageDimensionException(std::string message) : m_message(message) { }

	const char* what() const throw() override
	{
		return m_message.c_str();
	}

private:

	std::string m_message;

};