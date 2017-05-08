#pragma once
#include <tclap/Constraint.h>

class TestSuitConstraint : public TCLAP::Constraint<std::string> {

	const std::string illegalChars = "\\/:?\"<>|";

	bool isNameLegal(const std::string& name) const {
		for (auto it = name.begin(); it < name.end(); ++it) {
			auto found = illegalChars.find(*it) != std::string::npos;
			if (found)
				return false;
		}
		return true;
	}

public:

	/**
	* Returns a description of the Constraint.
	*/
	std::string description() const override {
		return "Input contains " + illegalChars + " in the name.";
	}

	/**
	* Returns the short ID for the Constraint.
	*/
	std::string shortID() const override {
		return "suit name";
	}

	/**
	* The method used to verify that the value parsed from the command
	* line meets the constraint.
	* \param value - The value that will be checked.
	*/
	bool check(const std::string& value) const override {
		return isNameLegal(value) && value != "input";
	}


};
