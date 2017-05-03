#pragma once
#include <tclap/Visitor.h>
#include <iostream>

class CalibrationModeVisitor : public TCLAP::Visitor {

public:

	void visit() override {
		std::cout << "Showing build info." << std::endl;
	}

};
