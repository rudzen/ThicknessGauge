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

class TestException : public std::exception {

    std::string message_;

public:

    explicit TestException(std::string message)
        : message_(message) {
    }

    const char* what() const throw() override { return message_.c_str(); }

};
