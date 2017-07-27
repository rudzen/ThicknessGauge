// Mads1.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include "../testOpenCV/namespaces/calc.h"

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

    std :: cout << "\nManhatten afstanden mellem de 2 punkter: " << calc::dist_manhattan(P1cv, P2cv) << "\n";

    std :: cout << "\nNorm afstanden mellem de 2 punkter: " << calc::dist_norm(P1cv,P2cv) << "\n";

    std::cout << "\nAverage af Punkt 1 X : " << calc::avg_x(P1v) << "\n";
    std::cout << "\nAverage af Punkt 1 X : " << calc::avg_y(P1v) << "\n";

    std::cout << "\nAverage af Punkt 1 X : " << calc::avg_x(P2v) << "\n";
    std::cout << "\nAverage af Punkt 1 X : " << calc::avg_y(P2v) << "\n";




    

    return 0;
}
