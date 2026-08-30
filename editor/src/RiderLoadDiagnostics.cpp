#include <quantum/editor/RiderLoadDiagnostics.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace quantum::editor
{
    namespace
    {
        inline constexpr double boundaryToleranceRelative = 1.0e-9;

        [[nodiscard]] double boundaryTolerance(
            const double distance) noexcept
        {
            return boundaryToleranceRelative
                * std::max(1.0, std::abs(distance));
        }
    }

    void RiderLoadDiagnosticsModel::update(
        const coaster::AuthoredTrack& track,
        coaster::RiderLoadHistory history)
    {
        sectionRanges_.clear();
        sectionRanges_.reserve(track.sectionCount());

        double sectionBegin = 0.0;
        for (std::size_t index = 0; index < track.sectionCount(); ++index)
        {
            const double length = coaster::sectionLength(
                track.section(index));
            sectionRanges_.push_back({sectionBegin, length});
            sectionBegin += length;
        }

        history_ = std::move(history);
        if (sectionRanges_.empty())
        {
            selectedSectionIndex_ = 0;
        }
        else
        {
            selectedSectionIndex_ = std::min(
                selectedSectionIndex_,
                sectionRanges_.size() - 1);
        }

        rebuildSelectedSection();
    }

    void RiderLoadDiagnosticsModel::selectSection(
        const std::size_t sectionIndex)
    {
        if (sectionIndex >= sectionRanges_.size())
        {
            throw std::out_of_range(
                "Rider-load diagnostic section index is out of range."
            );
        }

        selectedSectionIndex_ = sectionIndex;
        rebuildSelectedSection();
    }

    void RiderLoadDiagnosticsModel::clear() noexcept
    {
        sectionRanges_.clear();
        history_ = {};
        selectedSectionIndex_ = 0;
        selectedSection_ = {};
    }

    std::size_t RiderLoadDiagnosticsModel::sectionCount() const noexcept
    {
        return sectionRanges_.size();
    }

    const SectionRiderLoadDiagnostics&
    RiderLoadDiagnosticsModel::selectedSection() const noexcept
    {
        return selectedSection_;
    }

    void RiderLoadDiagnosticsModel::rebuildSelectedSection()
    {
        selectedSection_ = {};
        if (sectionRanges_.empty())
        {
            return;
        }

        const SectionRange range = sectionRanges_[selectedSectionIndex_];
        const double sectionEnd = range.begin + range.length;
        const double tolerance = std::max(
            boundaryTolerance(range.begin),
            boundaryTolerance(sectionEnd));

        selectedSection_.sectionIndex = selectedSectionIndex_;
        selectedSection_.trackDistanceBegin = range.begin;
        selectedSection_.sectionLength = range.length;
        selectedSection_.samples.reserve(history_.states.size());

        for (const coaster::RiderLoadState& state : history_.states)
        {
            if (state.distance < range.begin - tolerance)
            {
                continue;
            }
            if (state.distance > sectionEnd + tolerance)
            {
                break;
            }

            double localDistance = state.distance - range.begin;
            if (std::abs(localDistance) <= tolerance)
            {
                localDistance = 0.0;
            }
            else if (std::abs(localDistance - range.length) <= tolerance)
            {
                localDistance = range.length;
            }

            selectedSection_.samples.push_back({
                .localDistance = localDistance,
                .normalG = state.normalG,
                .lateralG = state.lateralG,
                .longitudinalG = state.longitudinalG,
                .vehicleSpeed = state.vehicleSpeed
            });
        }

        if (!history_.unreachable.has_value())
        {
            return;
        }

        selectedSection_.unreachable = history_.unreachable;
        const double stopDistance = history_.unreachable->distance;
        if (stopDistance < range.begin - tolerance)
        {
            selectedSection_.unreachableLocation =
                RiderLoadUnreachableLocation::BeforeSelectedSection;
        }
        else if (stopDistance > sectionEnd + tolerance
            || (selectedSectionIndex_ + 1 < sectionRanges_.size()
                && std::abs(stopDistance - sectionEnd) <= tolerance))
        {
            selectedSection_.unreachableLocation =
                RiderLoadUnreachableLocation::AfterSelectedSection;
        }
        else
        {
            selectedSection_.unreachableLocation =
                RiderLoadUnreachableLocation::WithinSelectedSection;
            selectedSection_.unreachableLocalDistance = std::clamp(
                stopDistance - range.begin,
                0.0,
                range.length);
        }
    }

    coaster::RiderLoadHistory evaluateRiderLoadDiagnostics(
        const coaster::AuthoredTrack& track,
        const RiderLoadDiagnosticSettings& settings)
    {
        if (track.sectionCount() == 0)
        {
            return {};
        }

        const std::vector<coaster::TrackKinematicState> kinematics =
            coaster::integrateAuthoredTrackKinematics(
                track,
                settings.integrationSpacing);
        return coaster::evaluateRiderLoads(
            kinematics,
            coaster::riderLoadEvaluationSettings(track.physicalSettings()));
    }
}
