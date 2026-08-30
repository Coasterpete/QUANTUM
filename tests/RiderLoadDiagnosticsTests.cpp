#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/RiderLoads.hpp>
#include <quantum/editor/RiderLoadDiagnostics.hpp>

#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
    using quantum::coaster::AuthoredTrack;
    using quantum::coaster::RiderLoadHistory;
    using quantum::coaster::RiderLoadState;
    using quantum::coaster::RiderLoadUnreachableState;
    using quantum::coaster::convertSectionToPlanarArc;
    using quantum::coaster::setPlanarArcRadius;
    using quantum::coaster::setPlanarArcSweptAngle;
    using quantum::coaster::setSectionLength;
    using quantum::editor::RiderLoadDiagnosticsModel;
    using quantum::editor::RiderLoadUnreachableLocation;
    using quantum::editor::SectionRiderLoadDiagnostics;

    class TestFailure final : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    void require(const bool condition, const std::string_view message)
    {
        if (!condition)
        {
            throw TestFailure(std::string(message));
        }
    }

    void requireNear(
        const double actual,
        const double expected,
        const double tolerance,
        const std::string_view message)
    {
        if (std::abs(actual - expected) > tolerance)
        {
            throw TestFailure(
                std::string(message) + ": expected "
                + std::to_string(expected) + ", got "
                + std::to_string(actual));
        }
    }

    [[nodiscard]] AuthoredTrack createMixedTrack()
    {
        AuthoredTrack track;
        track.appendSection();
        setSectionLength(track.section(0), 10.0);

        track.appendSection();
        convertSectionToPlanarArc(track.section(1));
        setPlanarArcRadius(track.section(1), 20.0);
        setPlanarArcSweptAngle(track.section(1), 0.5);

        track.appendSection();
        setSectionLength(track.section(2), 5.0);
        return track;
    }

    [[nodiscard]] RiderLoadState loadState(const double distance)
    {
        return RiderLoadState{
            .distance = distance,
            .vehicleSpeed = 20.0 + distance,
            .normalG = 1.0 + distance,
            .lateralG = -2.0 * distance,
            .longitudinalG = 0.25 * distance
        };
    }

    [[nodiscard]] RiderLoadHistory completeSyntheticHistory()
    {
        RiderLoadHistory history;
        for (const double distance : {0.0, 5.0, 10.0, 15.0, 20.0, 25.0})
        {
            history.states.push_back(loadState(distance));
        }
        return history;
    }

    void wholeTrackHistoryMapsToSectionLocalDistance()
    {
        const AuthoredTrack track = createMixedTrack();
        RiderLoadDiagnosticsModel model;
        model.update(track, completeSyntheticHistory());
        model.selectSection(1);

        const SectionRiderLoadDiagnostics& selected =
            model.selectedSection();
        require(selected.samples.size() == 3,
            "middle section keeps its three cumulative samples");
        requireNear(selected.trackDistanceBegin, 10.0, 0.0,
            "middle section prefix distance");
        requireNear(selected.samples[0].localDistance, 0.0, 0.0,
            "middle section starts at local zero");
        requireNear(selected.samples[1].localDistance, 5.0, 0.0,
            "middle sample subtracts the section prefix");
        requireNear(selected.samples[2].localDistance, 10.0, 0.0,
            "middle section ends at its authored length");
        requireNear(selected.samples[1].normalG, 16.0, 0.0,
            "normal G is copied from Core history");
        requireNear(selected.samples[1].lateralG, -30.0, 0.0,
            "lateral G is copied from Core history");
        requireNear(selected.samples[1].longitudinalG, 3.75, 0.0,
            "longitudinal G is copied from Core history");
        requireNear(selected.samples[1].vehicleSpeed, 35.0, 0.0,
            "vehicle speed is copied from Core history");
    }

    void firstSectionMapsWithoutOffset()
    {
        const AuthoredTrack track = createMixedTrack();
        RiderLoadDiagnosticsModel model;
        model.update(track, completeSyntheticHistory());

        const SectionRiderLoadDiagnostics& selected =
            model.selectedSection();
        require(selected.sectionIndex == 0, "first section starts selected");
        requireNear(selected.trackDistanceBegin, 0.0, 0.0,
            "first section has zero prefix");
        requireNear(selected.samples.front().localDistance, 0.0, 0.0,
            "first section begins at zero");
        requireNear(selected.samples.back().localDistance, 10.0, 0.0,
            "first section reaches authored length");
    }

    void nonFirstSectionSubtractsFullPrefix()
    {
        const AuthoredTrack track = createMixedTrack();
        RiderLoadDiagnosticsModel model;
        model.update(track, completeSyntheticHistory());
        model.selectSection(2);

        const SectionRiderLoadDiagnostics& selected =
            model.selectedSection();
        requireNear(selected.trackDistanceBegin, 20.0, 0.0,
            "third section prefix includes both earlier section lengths");
        requireNear(selected.samples.front().localDistance, 0.0, 0.0,
            "third section boundary maps to local zero");
        requireNear(selected.samples.back().localDistance, 5.0, 0.0,
            "third section endpoint maps to local authored length");
    }

    void mixedConstructionTypesUseOneCorePipeline()
    {
        const AuthoredTrack track = createMixedTrack();
        const RiderLoadHistory history =
            quantum::editor::evaluateRiderLoadDiagnostics(track);
        require(!history.states.empty(),
            "mixed Rate/Profile and Planar Arc track evaluates loads");

        RiderLoadDiagnosticsModel model;
        model.update(track, history);
        for (std::size_t index = 0; index < track.sectionCount(); ++index)
        {
            model.selectSection(index);
            const SectionRiderLoadDiagnostics& selected =
                model.selectedSection();
            require(!selected.samples.empty(),
                "each mixed construction section uses the same mapped data");
            requireNear(selected.samples.front().localDistance, 0.0, 1.0e-9,
                "each mixed section begins at local zero");
            requireNear(
                selected.samples.back().localDistance,
                selected.sectionLength,
                1.0e-9,
                "each mixed section ends at its local authored length");
        }
    }

    void sharedBoundaryRetainsRightContinuousCoreValue()
    {
        const AuthoredTrack track = createMixedTrack();
        RiderLoadHistory history = completeSyntheticHistory();
        history.states[2] = RiderLoadState{
            .distance = 10.0,
            .vehicleSpeed = 77.0,
            .normalG = 9.0,
            .lateralG = -8.0,
            .longitudinalG = 7.0
        };

        RiderLoadDiagnosticsModel model;
        model.update(track, std::move(history));
        const auto firstBoundary = model.selectedSection().samples.back();
        model.selectSection(1);
        const auto secondBoundary = model.selectedSection().samples.front();

        requireNear(firstBoundary.localDistance, 10.0, 0.0,
            "shared boundary is the first section endpoint");
        requireNear(secondBoundary.localDistance, 0.0, 0.0,
            "shared boundary is the following section entry");
        requireNear(firstBoundary.normalG, 9.0, 0.0,
            "first view preserves the exact Core boundary load");
        requireNear(secondBoundary.normalG, 9.0, 0.0,
            "following view preserves the same right-continuous load");
        requireNear(secondBoundary.vehicleSpeed, 77.0, 0.0,
            "boundary speed is not averaged or recalculated");

        RiderLoadHistory unreachableAtBoundary;
        unreachableAtBoundary.states = {loadState(0.0), loadState(5.0)};
        unreachableAtBoundary.unreachable = RiderLoadUnreachableState{
            .distance = 10.0,
            .speedSquared = -2.0
        };
        model.update(track, std::move(unreachableAtBoundary));
        model.selectSection(0);
        require(model.selectedSection().unreachableLocation
                == RiderLoadUnreachableLocation::AfterSelectedSection,
            "an unreachable internal boundary belongs after the left view");
        model.selectSection(1);
        require(model.selectedSection().unreachableLocation
                == RiderLoadUnreachableLocation::WithinSelectedSection,
            "an unreachable internal boundary belongs to the right section");
        requireNear(
            *model.selectedSection().unreachableLocalDistance,
            0.0,
            0.0,
            "right-continuous boundary failure maps to the right local zero");
    }

    void selectionChangeRebuildsTheCorrectSectionView()
    {
        const AuthoredTrack track = createMixedTrack();
        RiderLoadDiagnosticsModel model;
        model.update(track, completeSyntheticHistory());

        model.selectSection(1);
        requireNear(model.selectedSection().samples[1].vehicleSpeed,
            35.0, 0.0, "middle selection uses the middle sample");
        model.selectSection(2);
        require(model.selectedSection().sectionIndex == 2,
            "selection change records the new section");
        requireNear(model.selectedSection().samples[1].vehicleSpeed,
            45.0, 0.0, "new selection uses the final section sample");
    }

    void unreachableHistoryStopsWithoutFabricatedSamples()
    {
        const AuthoredTrack track = createMixedTrack();
        RiderLoadHistory history;
        for (const double distance : {0.0, 5.0, 10.0, 12.0})
        {
            history.states.push_back(loadState(distance));
        }
        history.unreachable = RiderLoadUnreachableState{
            .distance = 15.0,
            .speedSquared = -4.0
        };

        RiderLoadDiagnosticsModel model;
        model.update(track, std::move(history));
        model.selectSection(1);
        const SectionRiderLoadDiagnostics& failedSection =
            model.selectedSection();
        require(failedSection.samples.size() == 2,
            "only valid samples before the unreachable point are plotted");
        requireNear(failedSection.samples.back().localDistance, 2.0, 0.0,
            "last valid sample stays before failure");
        require(failedSection.unreachableLocation
                == RiderLoadUnreachableLocation::WithinSelectedSection,
            "failure is classified inside the selected section");
        require(failedSection.unreachableLocalDistance.has_value(),
            "in-section failure has a local distance");
        requireNear(*failedSection.unreachableLocalDistance, 5.0, 0.0,
            "failure distance subtracts the section prefix");

        model.selectSection(2);
        const SectionRiderLoadDiagnostics& laterSection =
            model.selectedSection();
        require(laterSection.samples.empty(),
            "no samples are fabricated after the unreachable point");
        require(laterSection.unreachableLocation
                == RiderLoadUnreachableLocation::BeforeSelectedSection,
            "later section reports that evaluation stopped before it");
    }

    void emptyAndNoValidHistoryRemainSafe()
    {
        const AuthoredTrack track = createMixedTrack();
        RiderLoadDiagnosticsModel model;
        model.update(track, {});
        require(model.selectedSection().samples.empty(),
            "empty history produces an empty section plot");
        requireNear(model.selectedSection().sectionLength, 10.0, 0.0,
            "empty history still retains the selected authored domain");

        RiderLoadHistory noValidStates;
        noValidStates.unreachable = RiderLoadUnreachableState{
            .distance = 0.0,
            .speedSquared = -1.0
        };
        model.update(track, std::move(noValidStates));
        require(model.selectedSection().samples.empty(),
            "unreachable-at-entry history has no fabricated sample");
        require(model.selectedSection().unreachableLocation
                == RiderLoadUnreachableLocation::WithinSelectedSection,
            "unreachable-at-entry is represented in the first section");
        requireNear(
            *model.selectedSection().unreachableLocalDistance,
            0.0,
            0.0,
            "unreachable-at-entry maps to local zero");

        model.clear();
        require(model.sectionCount() == 0,
            "cleared model has no section ranges");
        require(model.selectedSection().samples.empty(),
            "cleared model has no stale samples");
    }

    void newHistoryReplacesOldSamples()
    {
        const AuthoredTrack track = createMixedTrack();
        RiderLoadDiagnosticsModel model;
        model.update(track, completeSyntheticHistory());
        require(model.selectedSection().samples.size() == 3,
            "initial history supplies three first-section samples");

        RiderLoadHistory replacement;
        RiderLoadState entry = loadState(0.0);
        entry.normalG = 101.0;
        RiderLoadState exit = loadState(10.0);
        exit.normalG = 202.0;
        replacement.states = {entry, exit};
        model.update(track, std::move(replacement));

        const auto& samples = model.selectedSection().samples;
        require(samples.size() == 2,
            "replacement history discards old intermediate samples");
        requireNear(samples.front().normalG, 101.0, 0.0,
            "replacement entry value is current");
        requireNear(samples.back().normalG, 202.0, 0.0,
            "replacement exit value is current");
    }
}

int main()
{
    using Test = std::pair<std::string_view, void (*)()>;
    const std::vector<Test> tests{
        {"whole-track history maps to section-local distance",
            wholeTrackHistoryMapsToSectionLocalDistance},
        {"first section mapping", firstSectionMapsWithoutOffset},
        {"non-first section prefix subtraction",
            nonFirstSectionSubtractsFullPrefix},
        {"mixed construction types share one Core pipeline",
            mixedConstructionTypesUseOneCorePipeline},
        {"right-continuous shared boundary",
            sharedBoundaryRetainsRightContinuousCoreValue},
        {"selection change rebuilds section view",
            selectionChangeRebuildsTheCorrectSectionView},
        {"unreachable history truncates diagnostics",
            unreachableHistoryStopsWithoutFabricatedSamples},
        {"empty and no-valid history", emptyAndNoValidHistoryRemainSafe},
        {"new history replaces stale samples", newHistoryReplacesOldSamples}
    };

    try
    {
        for (const auto& [name, test] : tests)
        {
            test();
            std::cout << "[PASS] " << name << '\n';
        }
    }
    catch (const std::exception& exception)
    {
        std::cerr << "[FAIL] " << exception.what() << '\n';
        return 1;
    }

    std::cout << "All rider-load diagnostic model tests passed.\n";
    return 0;
}
