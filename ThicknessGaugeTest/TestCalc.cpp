#include "stdafx.h"
#include "CppUnitTest.h"
#include "../testOpenCV/namespaces/calc.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace ThicknessGaugeTest {
    cv::Point p1(2, 4);
    cv::Point p2(4, 4);

    TEST_CLASS(CALC_TEST) {

    private:

    public:

        TEST_METHOD(DistNorm) {
            auto dist = calc::dist_norm(p1, p2);
            auto expected = 2.0;
            Assert::AreEqual(expected, dist, 0.01);
        }

        TEST_METHOD(DistManha) {
            auto dist = calc::dist_manhattan(p1, p2);
            auto expected = 2.0;
            Assert::AreEqual(expected, dist, 0.01);
        }

        TEST_METHOD(AvgYVectorOfPoints) {
            std::vector<cv::Point> points;
            points.reserve(100);

            const auto count = 100;

            for (auto i = count; i--;)
                points.emplace_back(cv::Point(1, count + i));

            auto avgy = calc::avg_y(points);

            const auto expected = 149.5;
            const auto tolerance = 0.001;

            Assert::AreEqual(expected, avgy, tolerance);

        }

        TEST_METHOD(AvgXVectorOfPoints) {
            
            std::vector<cv::Point> points;
            points.reserve(100);

            const auto count = 100;

            for (auto i = count; i--;)
                points.emplace_back(cv::Point(count + i, 1));

            auto avgx = calc::avg_x(points);

            const auto expected = 149.5;
            const auto tolerance = 0.001;

            Assert::AreEqual(expected, avgx, tolerance);

        }

        TEST_METHOD(AvgXYVectorOfPoints) {
            
            std::vector<cv::Point> points;
            points.reserve(100);

            const auto count = 100;

            for (auto i = count; i--;)
                points.emplace_back(cv::Point(count + i, count + i));

            auto avgxy = calc::avg_xy(points);

            const auto expected = 149.5 * 2.0;
            const auto tolerance = 0.001;

            Assert::AreEqual(expected, avgxy[0] + avgxy[1], tolerance);

        }

        TEST_METHOD(RadiansDegrees) {
            
            const auto degree = 360.0;
            const auto radians = calc::PIx2;

            auto r = calc::deg_to_rad(degree);

            Assert::AreEqual(radians, r, 0.001);

            auto d = calc::rad_to_deg(radians);

            Assert::AreEqual(degree, d, 0.001);

        }

        TEST_METHOD(AnglePoints) {
            
            cv::Point2f p1(1.0, 1.0);
            cv::Point2f p2(0.0, 0.0);
            cv::Point2f c(2.0, 2.0);

            auto angle_from_zero = calc::angle_from_zero(p1.x, p1.y);

            auto angle_from_zero2 = calc::angle_from_zero(c.x, c.y);

            auto tolerance = calc::deg_to_rad(3.0);

            Assert::AreEqual(angle_from_zero2, angle_from_zero, tolerance);

        }

        TEST_METHOD(AngleBetweenPoints) {
            
            cv::Vec4d points(0.0, 2.0, 4.0, 4.0);

            auto angle = calc::angle(points);

            auto tolerance = calc::deg_to_rad(0.5);

            auto expected = calc::deg_to_rad(45.0);

            Assert::AreEqual(expected, angle, tolerance);

        }

    };
}
