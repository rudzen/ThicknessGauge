#include "PrettyMenu.h"
#include <opencv2/core/version.hpp>
#include <sstream>
#include <iostream>

vector<string>& PrettyMenu::split(const string& s, char delim, vector<string>& elems) const {
	stringstream ss(s);
	string item;

	while (getline(ss, item, delim))
		elems.push_back(item);

	return elems;
}

vector<string> PrettyMenu::split(const string& s, char delim) const {
	vector<string> elems;
	split(s, delim, elems);
	return elems;
}

void PrettyMenu::show() {
	for (auto& me : menu_)
		std::cout << me.second << '\n';
	std::cout << endl;
}

void PrettyMenu::generateTop() {
	menu_.push_back(menuentry(++pos_, seperator_));
	ostringstream ss;
	ss << title_ << version_;
	menu_.push_back(menuentry(++pos_, makeFilledBorder(ss.str())));
	menu_.push_back(menuentry(++pos_, makeFilledBorder(title2_)));
	menu_.push_back(menuentry(++pos_, seperator_));
	menu_.push_back(menuentry(++pos_, makeSingleBorderCentered(info_)));
}

void PrettyMenu::generateStatus() {
	ostringstream ss;
	menu_.push_back(menuentry(++pos_, makeSingleBorderCentered("s133235@student.dtu")));
	menu_.push_back(menuentry(++pos_, makeFilledBorder("Status")));
	ss << " Date/Time build : ";
	std::string data = __DATE__;
	std::string time = __TIME__;
	ss << trim(data) << " / " << trim(time);
	menu_.push_back(menuentry(++pos_, makeFilledBorder(ss.str())));

	ss.str(std::string());

	ss << getOpenCVInfo();
	menu_.push_back(menuentry(++pos_, makeFilledBorder(ss.str())));

	menu_.push_back(menuentry(++pos_, dot + space(MAX_LEN - 2) + dot));

}

void PrettyMenu::generateButtom() {
	//while (pos < MAX_HEI)
	//	menu.push_back(menuentry(++pos, dot + space(MAX_LEN - 2) + dot));
	menu_.push_back(menuentry(++pos_, overwrite(seperator_, " < powered by con-dynmenu v0.4 by rudz > ", 30)));
}

string PrettyMenu::makeFilledBorder(string s) const {
	auto len = s.length();
	if (len >= MAX_LEN - 2) {
		return s;
	}
	ostringstream ss;
	if (len == MAX_LEN - 4)
		ss << dot << ' ' << s << ' ' << dot;
	else {
		auto lenMod2 = len % 2;
		auto dots = replicate(dot, static_cast<int>((MAX_LEN - lenMod2 - 2 - len) >> 1));
		ss << dots << ' ' << s << ' ' << dots;
		if (len % 2 != 0)
			ss << replicate(dot, static_cast<int>(lenMod2));
	}
	return ss.str();
}

string PrettyMenu::makeSingleBorderCentered(string s) const {
	auto len = s.length();

	if (len > MAX_LEN - 2)
		return s;

	ostringstream ss;
	if (len == MAX_LEN - 4)
		ss << dot << ' ' << s << ' ' << dot;
	else if (len < MAX_LEN - 4) {
		ss << dot;
		auto lenMod2 = len % 2;
		auto dots = space(static_cast<int>((MAX_LEN - lenMod2 - 4 - len) >> 1));
		ss << dots << ' ' << s << ' ';
		if (len % 2 > 0) ss << space(lenMod2);
		ss << dots << dot;
	}
	else ss << s;
	return ss.str();
}

string PrettyMenu::makeSingleBordered(string s) const {
	auto len = s.length();

	if (len > MAX_LEN - 2)
		return s;

	ostringstream ss;
	ss << dot << s << space((MAX_LEN - 3 - len)) << dot;
	return ss.str();
}

string PrettyMenu::overwrite(string into, string toInsert, unsigned long startPos) {
	auto len = into.length();
	if (len == 0) return toInsert;
	auto lenInsert = toInsert.length();
	if (lenInsert == 0) return into;
	ostringstream ss;
	// no fault check from here!
	ss << into.substr(0, startPos - 1);
	ss << toInsert;
	if (startPos - 1 + lenInsert <= len - 1)
		ss << into.substr(ss.str().length(), into.length());
	return ss.str();
}

string PrettyMenu::generate_top_sections() {
	ostringstream ss;
	const auto s = 5;

	for (auto i = 0; i < 6; ++i)
		ss << '|' << (i + 1) << space(s);

	ss << '|';
	return ss.str();
}

string PrettyMenu::getOpenCVInfo() {
	string out("OpenCV: ");
	out.append(CV_VERSION);
	return out;
}

template <typename T>
string PrettyMenu::generate_regular_sections(T c, int val) {
	ostringstream ss;
	const auto s = 6;
	ss << '|';
	if (val > 0)
		ss << c << ':' << val;
	else
		ss << '|' << space(s);
	ss << '|';
	return ss.str();
}
