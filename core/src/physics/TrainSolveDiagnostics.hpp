#pragma once

#include <quantum/physics/CarPose.hpp>

#include <chrono>
#include <cstdint>

namespace quantum::physics::detail
{
    // The timer is instantiated only on paths that already received a
    // TrainSolveCounters pointer. A null pointer leaves the hot path untimed.
    class ScopedCounterTimer
    {
    public:
        ScopedCounterTimer(
            const TrainSolveCounters* const counters,
            std::uint64_t* const elapsedNanoseconds) noexcept
            : elapsedNanoseconds_(counters ? elapsedNanoseconds : nullptr),
              begin_(elapsedNanoseconds_
                    ? std::chrono::steady_clock::now()
                    : std::chrono::steady_clock::time_point{})
        {
        }

        ~ScopedCounterTimer()
        {
            if (elapsedNanoseconds_)
            {
                *elapsedNanoseconds_ += static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now() - begin_).count());
            }
        }

        ScopedCounterTimer(const ScopedCounterTimer&) = delete;
        ScopedCounterTimer& operator=(const ScopedCounterTimer&) = delete;

    private:
        std::uint64_t* elapsedNanoseconds_ = nullptr;
        std::chrono::steady_clock::time_point begin_;
    };
}
