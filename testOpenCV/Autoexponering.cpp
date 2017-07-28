#include "Autoexponering.h"
#include "CV/HoughLinesR.h"
#include "Camera/Seeker.h"

int Autoexponering::mads() {

    //punkt = 0.0;


    double P1[2];
    double P2[2];

    std::cout << "Tærte : " << calc::PI << '\n';
    std::cout << "---------------------------------------------------------------------\n";

    std::cout << "Indtast punkt 1 x koordinat: ";
    std::cin >> P1[0];
    std::cout << "Indtast punkt 1 y koordinat: ";
    std::cin >> P1[1];

    std::cout << "Indtast punkt 2 x koordinat: ";
    std::cin >> P2[0];
    std::cout << "Indtast punkt 2 y koordinat: ";
    std::cin >> P2[1];

    cv::Point2d P1cv(P1[0], P1[1]);
    cv::Point2d P2cv(P2[0], P2[1]);

    cv::Vec2d P1v(P1cv);
    cv::Vec2d P2v(P2cv);

    std::vector<cv::Point2d> Vector;

    Vector.emplace_back(P1cv);
    Vector.emplace_back(P2cv);

    cv::Vec4d Pvec(P1cv.x, P1cv.y, P2cv.x, P2cv.y);

    std::cout << "\nManhatten afstanden mellem de 2 punkter: " << calc::dist_manhattan(P1cv, P2cv) << "\n";

    std::cout << "\nNorm afstanden mellem de 2 punkter: " << calc::dist_norm(P1cv, P2cv) << "\n";

    std::cout << "\nAverage af Punkt 1 X : " << calc::avg_x(Pvec) << "\n";
    std::cout << "\nAverage af Punkt 1 Y : " << calc::avg_y(Pvec) << "\n";

    //std::cout << "\nAverage af Punkt 2 X : " << calc::avg_x(P2v) << "\n";
    //std::cout << "\nAverage af Punkt 2 Y : " << calc::avg_y(P2v) << "\n";

    std::cout << "\nAverage af Punkt 1 XY : " << calc::avg_xy(Vector) << "\n";

    std::string ABE = "ABE";

    cv::namedWindow(ABE, cv::WINDOW_FREERATIO);

    std::vector<cv::Mat> images;

    cv::Mat Image;

    unsigned long e = RealImage.exposure();
    RealImage.exposure(50000);

    log_time << RealImage.region() << '\n';

    //RealImage.region(cv::Rect_<unsigned long>(0, 0, 2448, 2050));

    //std::unique_ptr<HoughLinesR> hough = std::make_unique<HoughLinesR>(

    cv::Mat tmp;
    cv::Mat tmp2;

    while (true) {
        images.clear();
        //RealImage.cap(1, images);
        RealImage.cap_single(tmp2);
        cv::Mat& img = tmp2;
        cv::Scalar intensity = img.at<uchar>(cv::Point(1, 2));
        
        log_time << "w: " << img.cols << " h: " << img.rows << '\n';

        log_time << std::to_string(intensity[0]) << '\n';

        cv::bilateralFilter(img, tmp, 3, 20, 10);
        cv::threshold(tmp, tmp, 100, 255, CV_THRESH_BINARY);
        //GaussianBlur(img, img, cv::Size(5, 5), 0, 10, cv::BORDER_DEFAULT);

        
        

        draw::show_image(ABE, tmp);
        if (draw::is_escape_pressed(30)) {
            break;
        }
    }

    return 0;
}
