#pragma once

namespace stl {
	
	template <typename T>
	void copyVector(T& source, T& destination) {
		destination.reserve(source.size() + destination.size());
		destination.insert(destination.begin(), source.begin(), source.end());
	}

	template <typename T>
	void copyVector(const T& source, T& destination) {
		destination.reserve(source.size() + destination.size());
		destination.insert(destination.begin(), source.begin(), source.end());
	}

}