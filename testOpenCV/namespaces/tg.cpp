//          Copyright Rudy Alex Kohn 2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once
#include "tg.h"
#include <mutex>
#include <type_traits>
#include "UI/rlutil.h"

namespace tg {

    std::ostream& operator<<(std::ostream& os, SyncCout sc) {

        static std::mutex m;

        switch (sc) {
        case SyncCout::IO_LOCK:
            m.lock();
            break;
        case SyncCout::IO_UNLOCK:
            m.unlock();
        default: ;
        }
        return os;
    }

    std::ostream& operator<<(std::ostream& os, const LogTime lt) {

        switch (lt) {
        case LogTime::LOG_TIME:
            rlutil::setColor(rlutil::GREEN);
            os << '[' << get_time();
            break;
        case LogTime::LOG_DATE:
            rlutil::setColor(rlutil::GREEN);
            os << '[' << get_date();
            break;
        case LogTime::LOG_TIME_DATE:
            rlutil::setColor(rlutil::GREEN);
            os << '[' << get_time_date();
            break;
        case LogTime::LOG_OK_TIME:
            rlutil::setColor(rlutil::GREEN);
            os << '[' << get_time();
            break;
        case LogTime::LOG_ERR_TIME:
            rlutil::setColor(rlutil::RED);
            os << '[' << get_time();
            break;
        default: ;
        }

        os << ']';

        rlutil::setColor(rlutil::GREY);

        return os << ": ";


    }

    std::string get_time_date(const std::string format) {
        struct tm newtime;
        auto now = time(nullptr);
        localtime_s(&newtime, &now);
        char buf[1024];
        strftime(buf, sizeof(buf) - 1, format.c_str(), &newtime);
        return std::string(buf);
    }

    std::string get_time_date() {
        return get_time_date("%c");
    }

    std::string get_time() {
        return get_time_date("%T");
    }

    std::string get_date() {
        return get_time_date("%F");
    }

}
