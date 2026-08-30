#pragma once

#include <cstdint>
#include <optional>
#include <string_view>

namespace quantum::logging
{
    enum class LogLevel : std::uint8_t
    {
        Trace,
        Debug,
        Info,
        Warning,
        Error
    };

    inline constexpr LogLevel defaultLogLevel = LogLevel::Info;

    [[nodiscard]] LogLevel logLevel() noexcept;
    void setLogLevel(LogLevel level) noexcept;

    [[nodiscard]] bool isLogEnabled(LogLevel level) noexcept;
    [[nodiscard]] std::optional<LogLevel> parseLogLevel(
        std::string_view text) noexcept;
    [[nodiscard]] std::string_view logLevelName(LogLevel level) noexcept;

    void logMessage(
        LogLevel level,
        std::string_view category,
        std::string_view message) noexcept;

    // Formatting is skipped entirely when the supplied level is disabled.
    void logMessagef(
        LogLevel level,
        std::string_view category,
        const char* format,
        ...) noexcept;

    using LogSink = void (*)(
        LogLevel level,
        std::string_view category,
        std::string_view message,
        void* userData) noexcept;

    // Output redirection is intended for focused logger tests. Configure it
    // only while no other threads are producing log messages.
    void setLogSinkForTesting(LogSink sink, void* userData) noexcept;
    void resetLogSinkForTesting() noexcept;
}
