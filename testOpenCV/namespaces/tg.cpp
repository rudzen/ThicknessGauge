#pragma once

#include "tg.h"
#include <mutex>
#include <type_traits>

namespace tg {

	std::ostream& operator<<(std::ostream& os, SyncCout sc) {

		static std::mutex m;

		switch (sc) {
		case SyncCout::IO_LOCK:
			m.lock();
			break;
		case SyncCout::IO_UNLOCK:
			m.unlock();
		}
		return os;

	}

	std::ostream& operator<<(std::ostream& os, LogTime lt) {

		switch (lt) {
		case LogTime::LOG_TIME_NONE:
			return os;
		case LogTime::LOG_TIME_TIME:
			return os << get_time();
		case LogTime::LOG_TIME_DATE:
			return os << get_date();
		default: return os;
		}

	}

	template <class T>
	T alignMinValue(T value) {
		static_assert(std::is_fundamental<T>::value, "alignment is only possible for fundamental types.");
		if (value < 0)
			value = 0;
		return value;
	}

	std::string get_time_date() {
		struct tm newtime;
		auto now = time(nullptr);
		localtime_s(&newtime, &now);
		char buf[1024];
		strftime(buf, sizeof(buf) - 1, "%c", &newtime);
		return std::string(buf);
	}

	std::string get_time() {
		struct tm newtime;
		auto now = time(nullptr);
		localtime_s(&newtime, &now);
		char buf[1024];
		strftime(buf, sizeof(buf) - 1, "%T", &newtime);
		return std::string(buf);
	}

	std::string get_date() {
		struct tm newtime;
		auto now = time(nullptr);
		localtime_s(&newtime, &now);
		char buf[1024];
		strftime(buf, sizeof(buf) - 1, "%F", &newtime);
		return std::string(buf);
	}

}
