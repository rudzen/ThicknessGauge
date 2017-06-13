#pragma once
#include <tclap/Constraint.h>

class IntegerConstraint : public TCLAP::Constraint<int> {

    int min_;
    int max_;

    std::string name_;

public:

    IntegerConstraint(std::string name, const int min, const int max)
        : min_(min), max_(max), name_(name) {
    }

    IntegerConstraint()
        : min_(5), max_(200), name_("value constraint") {
    }

    /**
    * Returns a description of the Constraint.
    */
    std::string description() const override {
        return "Valid input is between " + std::to_string(min_) + " and " + std::to_string(max_) + " (inclusive).";
    }

    /**
    * Returns the short ID for the Constraint.
    */
    std::string shortID() const override {
        return name_;
    }

    /**
    * The method used to verify that the value parsed from the command
    * line meets the constraint.
    * \param value - The value that will be checked.
    */
    bool check(const int& value) const override {
        if (value < min_ || value > max_)
            return false;
        return true;
    }


};
