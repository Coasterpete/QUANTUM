#include <quantum/engine/Application.hpp>
#include <quantum/engine/Logging.hpp>
#include <quantum/editor/ReadmeCapture.hpp>

#include <SDL3/SDL_main.h>

#include <exception>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace
{
    void configureLogLevel(const int argc, char* argv[])
    {
        quantum::logging::LogLevel level = quantum::logging::defaultLogLevel;

        for (int index = 1; index < argc; ++index)
        {
            const std::string_view argument{argv[index]};
            std::string_view value;

            if (argument == "--log-level")
            {
                if (index + 1 >= argc)
                {
                    throw std::invalid_argument(
                        "--log-level requires trace, debug, info, warning, "
                        "or error."
                    );
                }
                value = argv[++index];
            }
            else if (argument.starts_with("--log-level="))
            {
                value = argument.substr(std::string_view{"--log-level="}.size());
            }
            else
            {
                continue;
            }

            const auto parsedLevel = quantum::logging::parseLogLevel(value);
            if (!parsedLevel.has_value())
            {
                throw std::invalid_argument(
                    "Unknown log level '" + std::string(value)
                    + "'; expected trace, debug, info, warning, or error."
                );
            }
            level = *parsedLevel;
        }

        quantum::logging::setLogLevel(level);
        quantum::logging::logMessagef(
            quantum::logging::LogLevel::Info,
            "APP",
            "Starting QUANTUM with %.*s logging.",
            static_cast<int>(quantum::logging::logLevelName(level).size()),
            quantum::logging::logLevelName(level).data()
        );
    }
}

int main(int argc, char* argv[])
{
    try
    {
        configureLogLevel(argc, argv);
        std::vector<std::string_view> arguments;
        for (int index = 1; index < argc; ++index)
            arguments.emplace_back(argv[index]);
        if (const auto manifest = quantum::editor::parseReadmeCaptureArguments(arguments))
            return quantum::editor::runReadmeCapture(
                quantum::editor::loadReadmeCaptureManifest(*manifest));
        quantum::engine::Application application;
        return application.run();
    }
    catch (const std::exception& exception)
    {
        quantum::logging::logMessagef(
            quantum::logging::LogLevel::Error,
            "APP",
            "Fatal error: %s",
            exception.what()
        );
        return 1;
    }
}
