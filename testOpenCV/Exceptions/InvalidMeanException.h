#pragma once
#include <stdexcept>

   /*
	|  __
	| /__\
	| X~~|			"The eternal code god
	|-\|//-.		 watches over this mess."
   /|`.|'.' \			- R.A.Kohn, 2017
  |,|.\~~ /||
  |:||   ';||
  ||||   | ||
  \ \|     |`.
  |\X|     | |
  | .'     |||
  | |   .  |||
  |||   |  `.| JS
  ||||  |   ||
  ||||  |   ||
  `+.__._._+*/

/**
 * \brief Take a guess:
 * a) It exists because i figured the means could be controlled
 * b) The exception is just too mean, and therefore it might not be valid :)
 */
class InvalidMeanException : public std::exception {

    std::string message_;

public:

    explicit InvalidMeanException(std::string message)
        : message_(message) {
    }

    const char* what() const throw() override { return message_.c_str(); }

};
