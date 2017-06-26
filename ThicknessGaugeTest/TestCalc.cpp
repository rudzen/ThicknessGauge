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

        TEST_METHOD(TestDistNorm) {
            auto dist = calc::dist_norm(p1, p2);
            auto expected = 2.0;
            Assert::AreEqual(expected, dist, 0.01);
        }

        TEST_METHOD(TestDistManha) {
            auto dist = calc::dist_manhattan(p1, p2);
            auto expected = 2.0;
            Assert::AreEqual(expected, dist, 0.01);
        }

    };
}
