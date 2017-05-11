#pragma once
#include <tclap/Visitor.h>
#include <iostream>

class GlobModeVisitor : public TCLAP::Visitor {

public:

	void visit() override {
		std::cout << "Glob mode set." << std::endl;
	}

};
