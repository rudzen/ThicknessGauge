#include "ProgressBar.h"

ProgressBar::ProgressBar() { }

ProgressBar::ProgressBar(unsigned long n_, const char* description_, std::ostream& out_) {

	n = n_;
	frequency_update = n_;
	description = description_;
	out = &out_;

	unit_bar = "=";
	unit_space = " ";
	desc_width = static_cast<unsigned int>(std::strlen(description));	// character width of description field

}

void ProgressBar::SetFrequencyUpdate(unsigned long frequency_update_) {

	frequency_update = frequency_update_ > n ? n : frequency_update_;
}

void ProgressBar::SetStyle(const char* unit_bar_, const char* unit_space_) {

	unit_bar = unit_bar_;
	unit_space = unit_space_;
}

int ProgressBar::GetConsoleWidth() {

	//#ifdef _WINDOWS
	CONSOLE_SCREEN_BUFFER_INFO csbi;
	GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &csbi);
	auto width = csbi.srWindow.Right - csbi.srWindow.Left;
//#else
//	struct winsize win;
//	ioctl(0, TIOCGWINSZ, &win);
//	auto width = win.ws_col;
//#endif

	return width;
}

int ProgressBar::GetBarLength() const {

	// get console width and according adjust the length of the progress bar

	auto bar_length = static_cast<int>((GetConsoleWidth() - desc_width - CHARACTER_WIDTH_PERCENTAGE) / 2.);

	return bar_length;
}

void ProgressBar::ClearBarField() const {

	for (auto i = 0; i<GetConsoleWidth(); ++i)
		*out << ' ';
	*out << '\r' << std::flush;
}

void ProgressBar::Progressed(unsigned long idx_) {
	try {
		if (idx_ > n) throw idx_;

		// determines whether to update the progress bar from frequency_update
		if ((idx_ != n) && (idx_ % (n / frequency_update) != 0)) return;

		// calculate the size of the progress bar
		auto bar_size = GetBarLength();

		// calculate percentage of progress
		auto progress_percent = idx_* TOTAL_PERCENTAGE / n;

		// calculate the percentage value of a unit bar 
		auto percent_per_unit_bar = TOTAL_PERCENTAGE / bar_size;

		// display progress bar
		*out << " " << description << " [";

		for (auto bar_length = 0; bar_length <= bar_size - 1; ++bar_length) {
			*out << (bar_length*percent_per_unit_bar<progress_percent ? unit_bar : unit_space);
		}

		*out << ']' << std::setw(CHARACTER_WIDTH_PERCENTAGE + 1) << std::setprecision(1) << std::fixed << progress_percent << "%\r" << std::flush;
	} catch (unsigned long e) {
		ClearBarField();
		std::cerr << "PROGRESS_BAR_EXCEPTION: _idx (" << e << ") went out of bounds, greater than n (" << n << ")." << std::endl << std::flush;
	}

}
