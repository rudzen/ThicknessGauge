#include "stdafx.h"
#include "CppUnitTest.h"
#include "../testOpenCV/namespaces/calc.h"
#include "../testOpenCV/namespaces/cvr.h"

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


        TEST_METHOD(LineNearFitCalc) {
            const size_t DEF_NEAR_EXTRACT = 20;
            const double RNG_DIFF = 0.1;

            std::shared_ptr<Data<double>> pdata = std::make_shared<Data<double>>();

            // fill up some values..
            std::vector<cv::Point2d> tmp_marking;
            std::vector<cv::Point2d> tmp_ground;

            const auto LIMIT = DEF_NEAR_EXTRACT * 10;

            // throw in some sugar
            stl::populate_x(tmp_ground, LIMIT);
            stl::populate_x(tmp_marking, LIMIT);

            // alter y values so they can be used for something
            for (auto i = 0; i < LIMIT; i++) {
                auto& p_m = tmp_marking[i];
                auto& p_g = tmp_ground[i];
                p_m.y = 4.0;
                p_g.y = p_m.y * calc::rngeezuz(1.45, 1.45 + RNG_DIFF);
                if (i % 2 == 0) {
                    p_m.y += calc::rngeezuz(1.0, 1.0 + RNG_DIFF);
                    p_g.y += calc::rngeezuz(0.4, 0.4 + RNG_DIFF);
                }
            }

            // copy vectors to result structure
            stl::copy_vector(tmp_marking, pdata->center_points);
            stl::copy_vector(tmp_ground, pdata->left_points);

            // copy pasta from original code, to avoid massive import #

            // line fitting outputs
            cv::Vec4f line_ground;
            cv::Vec4f line_marking;

            // the point snips
            std::vector<cv::Point2d> points_ground;
            std::vector<cv::Point2d> points_marking;

            points_ground.reserve(DEF_NEAR_EXTRACT);
            points_marking.reserve(DEF_NEAR_EXTRACT);

            cvr::extract_near<false, false>(pdata->center_points, points_marking, DEF_NEAR_EXTRACT);
            cvr::extract_near<true, false>(pdata->left_points, points_ground, DEF_NEAR_EXTRACT);

            auto marking_expected = calc::avg_y(tmp_marking);
            auto ground_expected = calc::avg_y(tmp_ground);

            auto total_excepted = abs(marking_expected - ground_expected) / 2.0;

            Assert::AreEqual(DEF_NEAR_EXTRACT, points_marking.size());
            Assert::AreEqual(DEF_NEAR_EXTRACT, points_ground.size());

            LineConfig line_config;

            line_config.dist_type(cv::DIST_L2);

            cvr::fit_line(points_ground, line_ground, line_config);
            cvr::fit_line(points_marking, line_marking, line_config);

            auto avg_ground = calc::avg_y(line_ground);
            auto avg_marking = calc::avg_y(line_marking);

            double dif = abs(avg_marking - avg_ground);

            // check with variation within the RNG_DIFF allowed, also because output is single precision
            Assert::AreEqual(total_excepted, dif, RNG_DIFF);

            //std::cout << dif << '\n';

            //return dif;


        }


    };
}
