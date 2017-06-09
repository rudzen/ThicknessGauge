#pragma once

#include "tg.h"
#include <mutex>
#include <type_traits>

namespace tg {

	std::ostream& operator<<(std::ostream& os, SyncCout sc) {

		static std::mutex m;

		if (sc == IO_LOCK)
			m.lock();

		if (sc == IO_UNLOCK)
			m.unlock();

		return os;
	}

	template <class T>
	T alignValue(T value) {
		static_assert(std::is_fundamental<T>::value, "alignment is only possible for fundamental types.");
		if (value < 0)
			value = 0;
		return value;
	}

}
