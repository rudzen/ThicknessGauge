// ThicknessGaugeNullSaver.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"
#include <iostream>
#include <string>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <opencv2/imgcodecs.hpp>

void save_null() {

    std::string filename;

    std::cout << "Enter filename to capture null file as.>";
    std::cin >> filename;

    if (filename.empty()) {
        std::cout << "You must give a filename.\n";
        return;
    }

    // quick and dirty hack to save null image quickly
    std::cout << "Enter delay in seconds before capture to " << filename << "\n>";
    int t;
    std::cin >> t;
    cv::Mat nullImage;
    cv::VideoCapture cap;
    cap.open(CV_CAP_PVAPI);
    cap >> nullImage;
    cv::imwrite(filename, nullImage);
    cap.release();
}

// simple application to generate null image files.
int main() {

    save_null();

    return 0;
}
