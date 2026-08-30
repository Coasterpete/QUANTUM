#include <quantum/engine/Logging.hpp>

#include <array>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace
{
    struct CapturedRecord
    {
        quantum::logging::LogLevel level{};
        std::string category;
        std::string message;
    };

    struct Capture
    {
        std::array<CapturedRecord, 8> records{};
        std::size_t recordCount = 0;
        bool overflowed = false;
    };

    void captureLog(
        const quantum::logging::LogLevel level,
        const std::string_view category,
        const std::string_view message,
        void* const userData) noexcept
    {
        auto& capture = *static_cast<Capture*>(userData);
        if (capture.recordCount >= capture.records.size())
        {
            capture.overflowed = true;
            return;
        }

        try
        {
            CapturedRecord& record = capture.records[capture.recordCount++];
            record.level = level;
            record.category = category;
            record.message = message;
        }
        catch (...)
        {
            capture.overflowed = true;
        }
    }

    void require(const bool condition, const char* const message)
    {
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    }

    void testDefaultFilteringPreservesMeaningfulEditorMessages()
    {
        Capture capture;
        quantum::logging::setLogSinkForTesting(captureLog, &capture);
        quantum::logging::setLogLevel(quantum::logging::defaultLogLevel);

        quantum::logging::logMessage(
            quantum::logging::LogLevel::Trace,
            "INP",
            "MOUSE_DOWN btn=1 pos=(25,40)"
        );
        quantum::logging::logMessage(
            quantum::logging::LogLevel::Debug,
            "SEL",
            "selected=2"
        );
        quantum::logging::logMessage(
            quantum::logging::LogLevel::Info,
            "EDIT",
            "Authored edit accepted; track now has 3 sections."
        );

        require(!capture.overflowed, "The logging capture overflowed.");
        require(
            capture.recordCount == 1,
            "The default level did not suppress Trace and Debug records."
        );
        require(
            capture.records[0].level == quantum::logging::LogLevel::Info,
            "The meaningful editor record did not retain Info severity."
        );
        require(
            capture.records[0].category == "EDIT",
            "The meaningful editor category was not preserved."
        );
        require(
            capture.records[0].message.find("Authored edit accepted")
                != std::string::npos,
            "The meaningful editor message was not emitted."
        );
    }

    void testThresholdIncludesConfiguredSeverityAndHigher()
    {
        Capture capture;
        quantum::logging::setLogSinkForTesting(captureLog, &capture);
        quantum::logging::setLogLevel(quantum::logging::LogLevel::Warning);

        quantum::logging::logMessage(
            quantum::logging::LogLevel::Info,
            "APP",
            "suppressed"
        );
        quantum::logging::logMessage(
            quantum::logging::LogLevel::Warning,
            "VK:validation",
            "recoverable warning"
        );
        quantum::logging::logMessage(
            quantum::logging::LogLevel::Error,
            "VK",
            "fatal renderer error"
        );

        require(
            capture.recordCount == 2,
            "Messages at or above the configured severity were not emitted."
        );
        require(
            capture.records[0].category == "VK:validation",
            "The Vulkan source category was not preserved."
        );
        require(
            capture.records[1].level == quantum::logging::LogLevel::Error,
            "Error was not emitted above the Warning threshold."
        );
    }

    void testTraceOptInEmitsFormattedInputRecords()
    {
        Capture capture;
        quantum::logging::setLogSinkForTesting(captureLog, &capture);
        quantum::logging::setLogLevel(quantum::logging::LogLevel::Trace);

        quantum::logging::logMessagef(
            quantum::logging::LogLevel::Trace,
            "INP",
            "MOUSE_UP btn=%d pos=(%d,%d)",
            3,
            120,
            240
        );

        require(
            capture.recordCount == 1,
            "Trace input was not emitted after Trace was enabled."
        );
        require(
            capture.records[0].category == "INP",
            "The input category was not preserved."
        );
        require(
            capture.records[0].message
                == "MOUSE_UP btn=3 pos=(120,240)",
            "The formatted Trace input message was not preserved."
        );
    }

    void testLevelParsing()
    {
        require(
            quantum::logging::parseLogLevel("TRACE")
                == quantum::logging::LogLevel::Trace,
            "Trace parsing should be case-insensitive."
        );
        require(
            quantum::logging::parseLogLevel("warn")
                == quantum::logging::LogLevel::Warning,
            "The warn alias was not parsed."
        );
        require(
            !quantum::logging::parseLogLevel("verbose").has_value(),
            "An unsupported logging level was accepted."
        );
    }
}

int main()
{
    try
    {
        testDefaultFilteringPreservesMeaningfulEditorMessages();
        testThresholdIncludesConfiguredSeverityAndHigher();
        testTraceOptInEmitsFormattedInputRecords();
        testLevelParsing();
        quantum::logging::resetLogSinkForTesting();
        quantum::logging::setLogLevel(quantum::logging::defaultLogLevel);
    }
    catch (const std::exception& exception)
    {
        quantum::logging::resetLogSinkForTesting();
        std::cerr << "Logging test failure: " << exception.what() << '\n';
        return 1;
    }

    std::cout << "Logging tests passed.\n";
    return 0;
}
