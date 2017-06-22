// Rudy Alex Kohn

#ifndef _PROGRESS_BAR_
#define _PROGRESS_BAR_

//#ifdef _WINDOWS
#include <windows.h>
//#else
//#include <sys/ioctl.h>
//#endif

#include <iostream>
#include <iomanip>
#include <cstring>

#define TOTAL_PERCENTAGE 100.0
#define CHARACTER_WIDTH_PERCENTAGE 4

class ProgressBar {

public:

    ProgressBar();

    explicit ProgressBar(unsigned long n, const char* description = "", std::ostream& out = std::cerr);

    void set_frequency_update(unsigned long frequency_update);

    void set_style(const char* unit_bar, const char* unit_space);

    void progressed(unsigned long idx);

public:

    unsigned long n() const;

    void n(unsigned long n);

private:
    unsigned long n_;

    unsigned int desc_width_;

    unsigned long frequency_update_;

    std::ostream* out_;

    const char* description_;

    const char* unit_bar_;

    const char* unit_space_;

    void clear_bar_field() const;

    static int get_console_width();

    int bar_length() const;

};

#endif
