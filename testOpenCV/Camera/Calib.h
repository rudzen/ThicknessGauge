#pragma once
#include <cstdio>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include "namespaces/draw.h"

class Calib {

    // simple manuel calibration of the camera..

    int numBoards = 0;
    int numCornersHor;
    int numCornersVer;


public:


    void run_calib() {
        printf("Enter number of corners along width: ");
        scanf("%d", &numCornersHor);
        printf("Enter number of corners along height: ");
        scanf("%d", &numCornersVer);
        printf("Enter number of boards: ");
        scanf("%d", &numBoards);

        auto numSquares = numCornersHor * numCornersVer;
        auto board_sz = cv::Size(numCornersHor, numCornersVer);

        auto capture = cv::VideoCapture(0);

        std::vector<std::vector<cv::Point3f>> object_points;
        std::vector<std::vector<cv::Point2f>> image_points;

        std::vector<cv::Point2f> corners;
        auto successes = 0;

        cv::Mat image;
        cv::Mat gray_image;
        capture >> image;


        std::vector<cv::Point3f> obj;
        for (int j = 0; j < numSquares; j++)
            obj.push_back(cv::Point3f(j / numCornersHor, j % numCornersHor, 0.0f));


        while (successes < numBoards) {
            cvtColor(image, gray_image, CV_BGR2GRAY);
            auto found = findChessboardCorners(image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
            if (found) {
                cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
                drawChessboardCorners(gray_image, board_sz, corners, found);
            }

            imshow("win1", image);
            imshow("win2", gray_image);
            capture >> image;
            auto key = cv::waitKey(1);

            if (key == 27)
                return;
            if (key == ' ' && found != 0) {
                image_points.push_back(corners);
                object_points.push_back(obj);
                printf("Snap stored!");
                successes++;
                if (successes >= numBoards)
                    break;
            }
        }


        cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
        cv::Mat distCoeffs;
        std::vector<cv::Mat> rvecs;
        std::vector<cv::Mat> tvecs;

        intrinsic.ptr<float>(0)[0] = 1;
        intrinsic.ptr<float>(1)[1] = 1;
        calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);

        cv::Mat imageUndistorted;
        while (1) {
            capture >> image;
            undistort(image, imageUndistorted, intrinsic, distCoeffs);
            imshow("win1", image);
            cv::imshow("win2", imageUndistorted);
            cv::waitKey(1);
        }

        capture.release();
        return;

    }

};
