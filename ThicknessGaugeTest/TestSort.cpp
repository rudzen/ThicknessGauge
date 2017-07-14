#include "stdafx.h"
#include "CppUnitTest.h"
#include "../testOpenCV/namespaces/sort.h"
#include "namespaces/cvr.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace ThicknessGaugeTest {

    TEST_CLASS(SORT_TEST) {

    public:

        TEST_METHOD(TestBinarySearch) {
            
            const int value = 42;
            const int point_count = 2000;
            const bool expected_false = false;

            std::vector<cv::Point> points;
            points.reserve(point_count);

            for (auto i = 0; i < point_count; i++) {
                points.emplace_back(cv::Point(i + 10, i));
            }

            bool exists = sorter::search_x(points, value);

            Assert::IsFalse(exists);

            points[4].x = value;

            exists = sorter::search_x(points, value);

            Assert::IsTrue(exists);

        }

    };
}
