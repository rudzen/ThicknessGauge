#pragma once
#include <tclap/Visitor.h>
#include <iostream>

#include "../namespaces/tg.h"

using namespace tg;

class DemoModeVisitor : public TCLAP::Visitor {
	
public:

	void visit() override {
		log_time << "Demo mode set." << std::endl;
	}

};
