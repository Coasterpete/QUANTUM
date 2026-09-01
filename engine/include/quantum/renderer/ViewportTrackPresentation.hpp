#pragma once

#include <cstdint>
#include <stdexcept>

namespace quantum::renderer
{
    // Viewport presentation is renderer state. It is deliberately independent
    // of authored geometry, geometry family, and track-style preset data.
    enum class TrackPresentationMode : std::uint8_t
    {
        Shaded,
        Wireframe,
        ShadedWireframe,
        CenterlineDebug
    };

    [[nodiscard]] constexpr bool isValidTrackPresentationMode(
        const TrackPresentationMode mode) noexcept
    {
        switch (mode)
        {
        case TrackPresentationMode::Shaded:
        case TrackPresentationMode::Wireframe:
        case TrackPresentationMode::ShadedWireframe:
        case TrackPresentationMode::CenterlineDebug:
            return true;
        }
        return false;
    }

    [[nodiscard]] constexpr const char* trackPresentationModeName(
        const TrackPresentationMode mode) noexcept
    {
        switch (mode)
        {
        case TrackPresentationMode::Shaded:
            return "Shaded";
        case TrackPresentationMode::Wireframe:
            return "Wireframe";
        case TrackPresentationMode::ShadedWireframe:
            return "Shaded + Wireframe";
        case TrackPresentationMode::CenterlineDebug:
            return "Centerline / Debug";
        }
        return "Invalid";
    }

    class TrackPresentationState
    {
    public:
        [[nodiscard]] TrackPresentationMode mode() const noexcept
        {
            return mode_;
        }

        void setMode(const TrackPresentationMode mode)
        {
            if (!isValidTrackPresentationMode(mode))
            {
                throw std::invalid_argument(
                    "The viewport track-presentation mode is invalid."
                );
            }
            mode_ = mode;
        }

    private:
        TrackPresentationMode mode_ = TrackPresentationMode::Shaded;
    };
}
