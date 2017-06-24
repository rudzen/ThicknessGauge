#include <string>
#include "ProgressBar.h"

ProgressBar::ProgressBar() {}

ProgressBar::ProgressBar(unsigned long n, const char* description, std::ostream& out) {

    n_ = n;
    frequency_update_ = n;
    description_ = description;
    out_ = &out;

    unit_bar_ = "=";
    unit_space_ = " ";
    desc_width_ = static_cast<unsigned int>(std::strlen(description)); // character width of description field

}

void ProgressBar::set_frequency_update(unsigned long frequency_update) {

    frequency_update_ = frequency_update > n_ ? n_ : frequency_update;
}

void ProgressBar::set_style(const char* unit_bar, const char* unit_space) {

    unit_bar_ = unit_bar;
    unit_space_ = unit_space;
}

int ProgressBar::get_console_width() {

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

int ProgressBar::bar_length() const {

    // get console width and according adjust the length of the progress bar

    auto bar_length = static_cast<int>((get_console_width() - desc_width_ - CHARACTER_WIDTH_PERCENTAGE) / 2.);

    return bar_length;
}

void ProgressBar::clear_bar_field() const {
    *out_ << std::string(" ", get_console_width()) << '\r' << std::flush;
}

void ProgressBar::progressed(unsigned long idx) {
    try {
        if (idx > n_)
            throw idx;

        // determines whether to update the progress bar from frequency_update
        if ((idx != n_) && (idx % (n_ / frequency_update_) != 0))
            return;

        // calculate the size of the progress bar
        auto bar_size = bar_length();

        // calculate percentage of progress
        auto progress_percent = idx * TOTAL_PERCENTAGE / n_;

        // calculate the percentage value of a unit bar 
        auto percent_per_unit_bar = TOTAL_PERCENTAGE / bar_size;

        // display progress bar
        *out_ << ' ' << description_ << " [";

        for (auto bar_length = 0; bar_length <= bar_size - 1; ++bar_length) {
            *out_ << (bar_length * percent_per_unit_bar < progress_percent ? unit_bar_ : unit_space_);
        }

        *out_ << ']' << std::setw(CHARACTER_WIDTH_PERCENTAGE + 1) << std::setprecision(1) << std::fixed << progress_percent << "%\r" << std::flush;
    } catch (unsigned long e) {
        clear_bar_field();
        std::cerr << "PROGRESS_BAR_EXCEPTION: _idx (" << e << ") went out of bounds, greater than n (" << n_ << ")." << std::endl << std::flush;
    }

}

unsigned long ProgressBar::n() const {
    return n_;
}

void ProgressBar::n(unsigned long new_n) {
    n_ = new_n;
}
