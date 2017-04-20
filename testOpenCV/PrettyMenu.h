/*
* The MIT License
*
* Copyright 2015-7 Rudy Alex Kohn (s133235@student.dtu.dk).
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/

/*
(      -4QQQQQQQQQQQQQQQQQQ: jQQQQQQQQQQ
(        4QQQQQQQQQQQQQQQQQ: jQQQQQQQQQQ
( )QQQm. ]QQQQQQQQQQQQQQQQQ: jQQQQQQQQQQ
( )WQQQ; ]Qf =QQQ  dQQ@^ -4: jQ(      QQ
( )WQQD  jQf =QQQ  dQW`  .   jQc___   QQ
(       jWQf =QQQ  dQf .mQc  jQQQQF  jQQ
(       ?WQf =QQQ  dQ; ]QQQ  jQQQP  jWQQ
( )WQQL  WWf =QQQ  dQ: jQQQ. jQQD  <QWQQ
( )WQQW  dQf :QQW  dQ; )QQ@  jQ@` _QQQQQ
( )WQQm  3Qk  ??'  dQL  "T'  jQ'  TTTTQQ
( )WQQQ  3QQ,   <  dQQ,   _. jQ       WW
wawWQQQwaaQQQwawWaamQQmc_wmwayQaaaaaaamQ
QWWQQQQWWQQQQQWQQQWQQQQQQQQWWQQQWQWQWQQQ
*/

/*
	: 2017.19.April
	. Updated to C++11
	. Added typedef stuff
	. Non-inlined some functions

	: 2016.December.06
	. Updated to C++03

	: 2015
	. First java version

 */

#pragma once

#include <string>
#include <vector>
#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>

using namespace std;

/**
* Simple class to print out stuff to console that looks nice!
* @author rudz
*/
class PrettyMenu {

	typedef pair<int, string> menuentry;

	const int MAX_LEN = 78;
	const int MAX_HEI = 20;

	const char dot = '*';
	//const char data_sep = '|';


	//const int TITLE_LEN = (int) (TITEL.length() + TITEL2.length());

	//int port;
	int pos_ = -1;
	string version_;

	vector<menuentry> menu_;

	vector<string>& split(const string& s, char delim, vector<string>& elems) const;

	vector<string> split(const string& s, char delim) const;

	// trim from start
	static string& ltrim(string& s) {
		s.erase(s.begin(), find_if(s.begin(), s.end(), not1(ptr_fun<int, int>(static_cast<int(*)(int)>(isspace)))));
		return s;
	}

	// trim from end
	static string& rtrim(string& s) {
		s.erase(find_if(s.rbegin(), s.rend(), not1(ptr_fun<int, int>(static_cast<int(*)(int)>(isspace)))).base(), s.end());
		return s;
	}

	// trim from both ends
	static string& trim(string& s) {
		return ltrim(rtrim(s));
	}

	void initialize() {
		seperator_ = replicate(dot, MAX_LEN);
		generateTop();
		generateStatus();
		generateButtom();
		menu_.shrink_to_fit();
	}

	string seperator_;

	string title_ = "Heigth measuring software - v.Ongoing";
	string title2_ = "<< !-X-! >>";
	string info_ = "R.A.Kohn";

public:

	vector<menuentry> data;

	explicit PrettyMenu(const string& version) : version_(version) {
		initialize();
	}

	PrettyMenu() {
		initialize();
	}

	/**
	* Prints out the current menu to console.
	*/
	void show();

	/**
	* Generate the top part of all menus
	*/
	void generateTop();

	/**
	* Generates the status part of the menu screen.
	*/
	void generateStatus();

	/**
	* Makes the last bit of the menu screen.
	*/
	void generateButtom();

	/**
	* Encapsulates a specified string with filled "border" of current
	* 'dot'.<br>
	* The parsed string will be centred.
	*
	* @param s : the string to encapsulate
	* @return the completed string
	*/
	string makeFilledBorder(string s) const;


	/**
	* Generates a line containing a single 'dot' at the beginning and end.<b>
	* The rest is filled with spaces and the parsed string is centred.
	*
	* @param s : the string to centre
	* @return : the resulting string
	*/
	string makeSingleBorderCentered(string s) const;

	/**
	* Generates a line containing a single 'dot' at the beginning and end.<b>
	* The string will follow after a space from the left, filled with spaces up
	* until the last 'dot'.
	*
	* @param s : the string to encapsulate.
	* @return : the resulting string
	*/
	string makeSingleBordered(string s) const;

	/**
	* Creates a string containing spaces
	*
	* @param amount : how many spaces to make
	* @return the string containing amount spaces
	*/
	template<typename T>
	static string space(T amount) {
		return replicate(' ', amount);
	}

	/**
	* Replicate a char
	*
	* @param c : the char to replicate
	* @param amount : how many time to replicate
	* @return the string containing amount char
	*/
	//std::string replicate(string c, int amount) const {
	//	ostringstream ss;
	//	for (auto i = 1; i <= amount; ++i)
	//		ss << c;
	//	return ss.str();
	//}

	template<typename T, typename A>
	static string replicate(T c, A amount) {
		return string(amount, c);
	}

	/**
	* Replaces a piece of an existing string with another string.<br>
	*
	* @param into : The string to overwrite in
	* @param toInsert : The string which is put in the into string.
	* @param startPos : Start position where it should be inserted at.
	* @return The resulting string.
	*/
	string overwrite(string into, string toInsert, unsigned long startPos);

	static string generate_top_sections();

	/**
	 * \brief Constructs regular menu section
	 * \param c The base text the menu section contains
	 * \return 
	 */
	template<typename T>
	static string generate_regular_sections(T c, int val);


public: // info functions (temporary)

	static string getOpenCVInfo();

};
