#pragma once
#include <string>
#include <memory>


class CommandLineOptions;

namespace args {


    const std::string default_camera_calibration_file = "C2450.json";

    /**
     * \brief Parses the command line arguments and dissolves them into options
     * \param argc Mirrored from main()
     * \param argv Mirrored from main()
     * \param options The options class (DTO)
     * \return true if an action which doesn't allow the thicknessgauge to proceed afterwards, fx show information etc.
     * otherwise false.
     */
    bool parse_args(int argc, char** argv, std::shared_ptr<CommandLineOptions> options);


}
