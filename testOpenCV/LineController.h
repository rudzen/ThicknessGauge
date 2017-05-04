// Rudy Alex Kohn

#pragma once
#include "Line.h"

class LineController {

	Line lines;

public:

	const Line& lines1() const {
		return lines;
	}

	void lines1(const Line& lines) {
		this->lines = lines;
	}
};
