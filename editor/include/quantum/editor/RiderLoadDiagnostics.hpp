#pragma once

#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/RiderLoads.hpp>

#include <cstddef>
#include <optional>
#include <vector>

namespace quantum::editor
{
    // Temporary editor-owned development settings. These remain separate
    // from authored document state until QUANTUM has a simulation-settings
    // model. Physics and units are still interpreted by QuantumCore.
    struct RiderLoadDiagnosticSettings
    {
        double integrationSpacing = 0.75;
        coaster::RiderLoadEvaluationSettings evaluation{
            .initialSpeed = 20.0,
            .metersPerCoordinateUnit = 1.0,
            .gravityAcceleration = coaster::standardGravityAcceleration
        };
    };

    inline constexpr RiderLoadDiagnosticSettings
        developmentRiderLoadDiagnosticSettings{};

    struct RiderLoadDiagnosticSample
    {
        double localDistance = 0.0;
        double normalG = 0.0;
        double lateralG = 0.0;
        double longitudinalG = 0.0;
        double vehicleSpeed = 0.0;
    };

    enum class RiderLoadUnreachableLocation
    {
        None,
        BeforeSelectedSection,
        WithinSelectedSection,
        AfterSelectedSection
    };

    // Read-only load plot data for one authored section. Samples retain the
    // exact values returned by Core; only cumulative distance is translated
    // into the selected section's local [0, length] domain.
    struct SectionRiderLoadDiagnostics
    {
        std::size_t sectionIndex = 0;
        double trackDistanceBegin = 0.0;
        double sectionLength = 0.0;
        std::vector<RiderLoadDiagnosticSample> samples;
        std::optional<coaster::RiderLoadUnreachableState> unreachable;
        RiderLoadUnreachableLocation unreachableLocation =
            RiderLoadUnreachableLocation::None;
        std::optional<double> unreachableLocalDistance;
    };

    // Pure editor-side mapping model. It stores one authoritative whole-track
    // RiderLoadHistory and rebuilds a selected-section view without knowing
    // about ImGui, renderer data, or authored construction variants.
    class RiderLoadDiagnosticsModel
    {
    public:
        void update(
            const coaster::AuthoredTrack& track,
            coaster::RiderLoadHistory history
        );
        void selectSection(std::size_t sectionIndex);
        void clear() noexcept;

        [[nodiscard]] std::size_t sectionCount() const noexcept;
        [[nodiscard]] const SectionRiderLoadDiagnostics& selectedSection()
            const noexcept;

    private:
        struct SectionRange
        {
            double begin = 0.0;
            double length = 0.0;
        };

        void rebuildSelectedSection();

        std::vector<SectionRange> sectionRanges_;
        coaster::RiderLoadHistory history_;
        std::size_t selectedSectionIndex_ = 0;
        SectionRiderLoadDiagnostics selectedSection_;
    };

    // Orchestrates the existing Core pipeline for the editor. All kinematic,
    // speed, and rider-load calculations remain in QuantumCore.
    [[nodiscard]] coaster::RiderLoadHistory evaluateRiderLoadDiagnostics(
        const coaster::AuthoredTrack& track,
        const RiderLoadDiagnosticSettings& settings =
            developmentRiderLoadDiagnosticSettings
    );
}
