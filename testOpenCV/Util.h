#pragma once
#include <opencv2/core/mat.hpp>
#include <iostream>
#include <fstream>
#include "Stringtools.h"
#include "IsValidFileName.h"

// generic utility functions...

class Util {

public:

	static char* getTime();

	static std::string type2str(int type);

	static bool isFile(const std::string& name) {
#ifdef _MSC_VER // should only be POSIX, but M$ is weird..
		struct stat buffer;
		return (stat(name.c_str(), &buffer) == 0);
#else // this has no effect
		if (auto file = fopen(name.c_str(), "r")) {
			fclose(file);
			return true;
		}
		return false;
#endif
	}


	/** Brief Print message to console
	* Outputs message to console
	* @param message The message to output
	*/
	static void log(char *message) {
		std::cout << message << std::endl;
	}

	/** Brief Print message to console
	* Outputs message to console
	* @param message The message to output
	*/
	static void log(std::string message) {
		std::cout << message << std::endl;
	}

	/** Brief Determines the highest of two values
	* Max of two ints
	* @param a operand #1
	* @param b operand #2
	* @return the highest of the two operands, defaults to operand #1
	*/
	//static int max(int a, int b);

	///** Brief Determines the highest of three values
	//* Max of three ints
	//* @param a Operand #1
	//* @param b Operand #2
	//* @param c Operand #3
	//* @return The highest of the three operands, defaults to operand #1
	//*/
	//static int max(int a, int b, int c) {
	//	return max(a, max(b, c));
	//}

	///** Brief Determines the highest of four values
	//* Max of four ints
	//* @param a Operand #1
	//* @param b Operand #2
	//* @param c Operand #3
	//* @param d Operand #4
	//* @return The highest of the four operands, defaults to operand #1
	//*/
	//static int max(int a, int b, int c, int d) {
	//	return max(max(a, max(b, c)), d);
	//}

	///** Brief Determines the lowest of two values
	//* The min of two ints
	//* @param a Operand #1
	//* @param b Operand #2
	//* @return The lowest of the operands, defaults to operand #1
	//*/
	//template<typename T>
	//static T min(T a, T b) {
	//	return a < b ? a : b;
	//}

	/** Brief Calculates the manhattan distance
	* Manhattan distance between two points
	* @param x1 Point #1 x
	* @param x2 Point #2 x
	* @param y1 Point #1 y
	* @param y2 Point #2 y
	* @return The manhattan distance between the two points
	*/
	static int dist_manhattan(int x1, int x2, int y1, int y2) {
		return abs(x2 - x1 + y2 - y1);
	}

	/** Brief Integer manhattan distance between two OpenCV points
	* Manhattan distance between two OpenCV points
	* @param p1 Point #1
	* @param p2 Point #2
	* @return The distance as integer
	*/
	static int dist_manhattan(cv::Point &p1, cv::Point &p2) {
		return dist_manhattan(p1.x, p2.x, p1.y, p1.y);
	}

	///** Brief Is a point located within an image frame
	//* Determines if a given point is located within image dimensions
	//* @param PointX The X coordinate of the point
	//* @param PointY The Y coordinate of the point
	//* @param ImageWidth The width of the image
	//* @param ImageHeight The height of the image
	//* @return true if the point is within the image, otherwise false
	//*/
	//static bool is_in_image(int PointX, int PointY, int ImageWidth, int ImageHeight) {
	//	return PointX > 0 && PointX < ImageWidth && PointY > 0 && PointY < ImageHeight;
	//}

	///** Brief Is a OpenCV point located within an image
	//* Determines if a given OpenCV point is located within an image
	//* @param point The point
	//* @param Image The image
	//* @return true if the point is within the image borders, otherwise false
	//*/
	//static bool is_in_image(cv::Point &point, cv::Mat &Image) {
	//	return is_in_image(point.x, point.y, Image.cols, Image.rows);
	//}

	/**
	* Round to the nearest integer
	* @param d The value to round
	* @return Nearest integer as double
	*/
	static int round(double d) {
		return static_cast<int>(floor(d + 0.5));
	}


	static bool validFileName(const std::string& filename) {
		return IsValidFileName(filename.c_str());
	}

};

