#pragma once
#include <cmath>


/**
 * \brief Calculation utility functionality
 */
namespace calc {
	
	//int dist_manhattan(int x1, int x2, int y1, int y2) {
	//	return abs(x2 - x1 + y2 - y1);
	//}

	/** Brief Calculates the manhattan distance
	* Manhattan distance between two points
	* @param x1 Point #1 x
	* @param x2 Point #2 x
	* @param y1 Point #1 y
	* @param y2 Point #2 y
	* @return The manhattan distance between the two points
	*/
	template <class T>
	__forceinline T dis_manhattan(T x1, T x2, T y1, T y2) {
		return abs(x2 - x1 + y2 - y1);
	}


}
