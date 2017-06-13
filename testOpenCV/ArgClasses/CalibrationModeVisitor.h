#pragma once
#include <tclap/Visitor.h>
#include <iostream>

class CalibrationModeVisitor : public TCLAP::Visitor {

public:

    void visit() override {
        std::cout << "Calibration mode set.[NOT IMPLEMENTED FULLY YET]" << std::endl;
    }

};
