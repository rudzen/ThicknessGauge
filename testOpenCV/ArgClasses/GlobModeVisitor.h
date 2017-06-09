#pragma once
#include <tclap/Visitor.h>
#include <iostream>

#include "../namespaces/tg.h"

using namespace tg;

class GlobModeVisitor : public TCLAP::Visitor {

public:

	void visit() override {
		log_time << "Glob mode set." << std::endl;
	}

};
