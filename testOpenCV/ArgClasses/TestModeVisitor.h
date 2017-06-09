#pragma once
#include <tclap/Visitor.h>
#include <iostream>

#include "../namespaces/tg.h"

using namespace tg;

class TestModeVisitor : public TCLAP::Visitor {

public:

	void visit() override {
		log_time << "Test mode set." << std::endl;
	}

};
