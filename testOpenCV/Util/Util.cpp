#include "Util.h"
#include <ctime>
#include <opencv2/core/hal/interface.h>

//char* Util::getTime() {
//	struct tm newtime;
//	auto now = time(nullptr);
//	localtime_s(&newtime, &now);
//	char buf[1024];
//	strftime(buf, sizeof(buf) - 1, "%c", &newtime);
//	return buf;
//
//	//	time_t tm;
//	//	time(&tm);
//	//#ifdef _WIN64
//	//	struct tm *t2 = localtime(&tm);
//	//#else
//	//	struct tm *t2 = localtime(&tm);
//	//#endif // _WIN64
//	//
//	//	char buf[1024];
//	//	strftime(buf, sizeof(buf) - 1, "%c", t2);
//	//	return buf;
//}

std::string Util::getTime() {
	struct tm newtime;
	auto now = time(nullptr);
	localtime_s(&newtime, &now);
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", &newtime);
	return std::string(buf);
}

std::string Util::type2str(int type) {
	std::string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth) {
	case CV_8U: r = "8U";
		break;
	case CV_8S: r = "8S";
		break;
	case CV_16U: r = "16U";
		break;
	case CV_16S: r = "16S";
		break;
	case CV_32S: r = "32S";
		break;
	case CV_32F: r = "32F";
		break;
	case CV_64F: r = "64F";
		break;
	default: r = "User";
		break;
	}

	r += "C";
	r += (chans + '0');

	return r;
}

//int Util::max(int a, int b) {
//	return a > b ? a : b;
//}
