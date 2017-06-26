#include <vector>
#include <memory>
#include "stdafx.h"
#include "CppUnitTest.h"
#include "../testOpenCV/namespaces/filesystem.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace ThicknessGaugeTest {
    TEST_CLASS(FILE_PATH_TEST) {
    public:

        TEST_METHOD(TestPathLegality) {

            // set up data
            std::string pa = "C:\\Program Files (x86)\\Microsoft Visual Studio 14.0\\XML\\Schemas\\1033";
            const std::vector<std::string> results_complete{
                "C:", "Program Files (x86)", "Microsoft Visual Studio 14.0", "XML", "Schemas", "1033"
            };

            const std::vector<std::string> results_legal{
                "C:\\",
                "C:\\Program Files (x86)\\",
                "C:\\Program Files (x86)\\Microsoft Visual Studio 14.0\\",
                "C:\\Program Files (x86)\\Microsoft Visual Studio 14.0\\XML\\",
                "C:\\Program Files (x86)\\Microsoft Visual Studio 14.0\\XML\\Schemas\\",
                "C:\\Program Files (x86)\\Microsoft Visual Studio 14.0\\XML\\Schemas\\1033\\"
            };

            auto path_testing = std::make_shared<file::path_legal>(pa);

            auto anything = file::is_path_legal(path_testing);

            Assert::IsTrue(anything);


            // test sizes

            const size_t target_count = 6;
            Assert::AreEqual(path_testing->complete_path.size(), target_count);
            Assert::AreEqual(path_testing->legal_part.size(), target_count);

            // test content

            for (size_t i = 0; i < target_count; i++) {
                Assert::AreEqual(path_testing->complete_path[i], results_complete[i]);
                Assert::AreEqual(path_testing->legal_part[i], results_legal[i]);
            }

        }

    };
}
