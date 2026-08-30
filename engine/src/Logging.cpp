#include <quantum/engine/Logging.hpp>

#include <SDL3/SDL_log.h>

#include <array>
#include <atomic>
#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <vector>

namespace
{
    [[nodiscard]] SDL_LogPriority sdlPriority(
        const quantum::logging::LogLevel level) noexcept
    {
        using quantum::logging::LogLevel;

        switch (level)
        {
        case LogLevel::Trace:
            return SDL_LOG_PRIORITY_TRACE;
        case LogLevel::Debug:
            return SDL_LOG_PRIORITY_DEBUG;
        case LogLevel::Info:
            return SDL_LOG_PRIORITY_INFO;
        case LogLevel::Warning:
            return SDL_LOG_PRIORITY_WARN;
        case LogLevel::Error:
            return SDL_LOG_PRIORITY_ERROR;
        }

        return SDL_LOG_PRIORITY_INFO;
    }

    void sdlLogSink(
        const quantum::logging::LogLevel level,
        const std::string_view category,
        const std::string_view message,
        void*) noexcept
    {
        SDL_LogMessage(
            SDL_LOG_CATEGORY_APPLICATION,
            sdlPriority(level),
            "[%.*s] %.*s",
            static_cast<int>(category.size()),
            category.data(),
            static_cast<int>(message.size()),
            message.data()
        );
    }

    std::atomic<quantum::logging::LogLevel> currentLogLevel{
        quantum::logging::defaultLogLevel
    };
    std::atomic<quantum::logging::LogSink> currentLogSink{sdlLogSink};
    std::atomic<void*> currentLogSinkUserData{nullptr};

    [[nodiscard]] bool equalsIgnoreCase(
        const std::string_view left,
        const std::string_view right) noexcept
    {
        if (left.size() != right.size())
        {
            return false;
        }

        for (std::size_t index = 0; index < left.size(); ++index)
        {
            const auto leftCharacter = static_cast<unsigned char>(left[index]);
            const auto rightCharacter =
                static_cast<unsigned char>(right[index]);
            if (std::tolower(leftCharacter) != std::tolower(rightCharacter))
            {
                return false;
            }
        }

        return true;
    }
}

namespace quantum::logging
{
    LogLevel logLevel() noexcept
    {
        return currentLogLevel.load(std::memory_order_relaxed);
    }

    void setLogLevel(const LogLevel level) noexcept
    {
        currentLogLevel.store(level, std::memory_order_relaxed);

        // SDL applies its own category threshold after the QUANTUM filter.
        // Keep the application category aligned so opt-in Trace/Debug records
        // are not discarded by SDL after QUANTUM has accepted them.
        SDL_SetLogPriority(SDL_LOG_CATEGORY_APPLICATION, sdlPriority(level));
    }

    bool isLogEnabled(const LogLevel level) noexcept
    {
        return level >= logLevel();
    }

    std::optional<LogLevel> parseLogLevel(
        const std::string_view text) noexcept
    {
        if (equalsIgnoreCase(text, "trace"))
        {
            return LogLevel::Trace;
        }
        if (equalsIgnoreCase(text, "debug"))
        {
            return LogLevel::Debug;
        }
        if (equalsIgnoreCase(text, "info"))
        {
            return LogLevel::Info;
        }
        if (equalsIgnoreCase(text, "warning")
            || equalsIgnoreCase(text, "warn"))
        {
            return LogLevel::Warning;
        }
        if (equalsIgnoreCase(text, "error"))
        {
            return LogLevel::Error;
        }

        return std::nullopt;
    }

    std::string_view logLevelName(const LogLevel level) noexcept
    {
        switch (level)
        {
        case LogLevel::Trace:
            return "trace";
        case LogLevel::Debug:
            return "debug";
        case LogLevel::Info:
            return "info";
        case LogLevel::Warning:
            return "warning";
        case LogLevel::Error:
            return "error";
        }

        return "unknown";
    }

    void logMessage(
        const LogLevel level,
        const std::string_view category,
        const std::string_view message) noexcept
    {
        if (!isLogEnabled(level))
        {
            return;
        }

        const LogSink sink = currentLogSink.load(std::memory_order_relaxed);
        if (sink != nullptr)
        {
            sink(
                level,
                category,
                message,
                currentLogSinkUserData.load(std::memory_order_relaxed)
            );
        }
    }

    void logMessagef(
        const LogLevel level,
        const std::string_view category,
        const char* const format,
        ...) noexcept
    {
        if (!isLogEnabled(level))
        {
            return;
        }

        if (format == nullptr)
        {
            logMessage(level, category, "Invalid null log format string.");
            return;
        }

        std::array<char, 1024> stackBuffer{};
        va_list arguments;
        va_start(arguments, format);
        va_list argumentsCopy;
        va_copy(argumentsCopy, arguments);
        const int requiredLength = std::vsnprintf(
            stackBuffer.data(),
            stackBuffer.size(),
            format,
            arguments
        );
        va_end(arguments);

        if (requiredLength < 0)
        {
            va_end(argumentsCopy);
            logMessage(level, category, "Unable to format log message.");
            return;
        }

        if (static_cast<std::size_t>(requiredLength) < stackBuffer.size())
        {
            va_end(argumentsCopy);
            logMessage(
                level,
                category,
                std::string_view{
                    stackBuffer.data(),
                    static_cast<std::size_t>(requiredLength)}
            );
            return;
        }

        try
        {
            std::vector<char> dynamicBuffer(
                static_cast<std::size_t>(requiredLength) + 1);
            std::vsnprintf(
                dynamicBuffer.data(),
                dynamicBuffer.size(),
                format,
                argumentsCopy
            );
            va_end(argumentsCopy);
            logMessage(
                level,
                category,
                std::string_view{
                    dynamicBuffer.data(),
                    static_cast<std::size_t>(requiredLength)}
            );
        }
        catch (...)
        {
            va_end(argumentsCopy);
            logMessage(
                level,
                category,
                std::string_view{stackBuffer.data(), stackBuffer.size() - 1}
            );
        }
    }

    void setLogSinkForTesting(
        const LogSink sink,
        void* const userData) noexcept
    {
        currentLogSinkUserData.store(userData, std::memory_order_relaxed);
        currentLogSink.store(
            sink != nullptr ? sink : sdlLogSink,
            std::memory_order_relaxed
        );
    }

    void resetLogSinkForTesting() noexcept
    {
        currentLogSink.store(sdlLogSink, std::memory_order_relaxed);
        currentLogSinkUserData.store(nullptr, std::memory_order_relaxed);
    }
}
