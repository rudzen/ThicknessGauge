// Mads1.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include "../testOpenCV/namespaces/calc.h"
#include "../testOpenCV/Camera/CapturePvApi.h"
#include "../testOpenCV/namespaces/draw.h"

int main() {

    double P1[2];
    double P2[2];

    std::cout << "Tærte : " << calc::PI << '\n';
    std::cout << "---------------------------------------------------------------------\n";

    std::cout << "Indtast punkt 1 x koordinat: ";
    std::cin  >> P1[0];
    std::cout << "Indtast punkt 1 y koordinat: ";
    std::cin  >> P1[1];

    std::cout << "Indtast punkt 2 x koordinat: ";
    std::cin  >> P2[0];
    std::cout << "Indtast punkt 2 y koordinat: ";
    std::cin  >> P2[1];

    cv::Point2d P1cv(P1[0],P1[1]);
    cv::Point2d P2cv(P2[0],P2[1]);

    cv::Vec2d P1v(P1cv);
    cv::Vec2d P2v(P2cv);

    std::vector<cv::Point2d> Vector;

    Vector.emplace_back(P1cv);
    Vector.emplace_back(P2cv);

    cv::Vec4d Pvec(P1cv.x,P1cv.y, P2cv.x , P2cv.y);

    std :: cout << "\nManhatten afstanden mellem de 2 punkter: " << calc::dist_manhattan(P1cv, P2cv) << "\n";

    std :: cout << "\nNorm afstanden mellem de 2 punkter: " << calc::dist_norm(P1cv,P2cv) << "\n";

    std::cout << "\nAverage af Punkt 1 X : " << calc::avg_x(Pvec) << "\n";
    std::cout << "\nAverage af Punkt 1 Y : " << calc::avg_y(Pvec) << "\n";

    //std::cout << "\nAverage af Punkt 2 X : " << calc::avg_x(P2v) << "\n";
    //std::cout << "\nAverage af Punkt 2 Y : " << calc::avg_y(P2v) << "\n";

    std::cout << "\nAverage af Punkt 1 XY : " << calc::avg_xy(Vector) << "\n";

    std::string ABE = "ABE";

    cv::namedWindow(ABE);

    cv::Mat Image;

    CapturePvApi RealImage;

    RealImage.initialize();

    while(true) {
        RealImage.cap_single(Image);
        draw::show_image(ABE, Image);
        if(draw::is_escape_pressed(30)) {
            break;
        }
    }


    

    return 0;
}
