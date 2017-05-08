#pragma once
#include <tclap/Visitor.h>
#include <iostream>

class TestModeVisitor : public TCLAP::Visitor {

public:

	void visit() override {
		std::cout << "Test mode set." << std::endl;
	}

};
