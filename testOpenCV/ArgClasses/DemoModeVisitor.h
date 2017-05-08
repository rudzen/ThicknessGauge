#pragma once
#include <tclap/Visitor.h>
#include <iostream>

class DemoModeVisitor : public TCLAP::Visitor {
	
public:

	void visit() override {
		std::cout << "Demo mode set." << std::endl;
	}

};
