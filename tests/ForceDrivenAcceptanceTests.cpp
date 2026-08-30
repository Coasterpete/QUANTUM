#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/TrackTopology.hpp>
#include <quantum/editor/AuthoredTrackEditTransaction.hpp>
#include <quantum/editor/CenterlineVisualization.hpp>
#include <quantum/editor/RiderLoadDiagnostics.hpp>

#include <glm/geometric.hpp>

#include <array>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
    using namespace quantum::coaster;
    using namespace quantum::editor;

    void require(const bool condition, const char* message)
    {
        if (!condition) throw std::runtime_error(message);
    }

    AuthoredTrack mixedTrack()
    {
        auto track = createNewDocument();
        setSectionLength(track.section(0), 8);
        track.insertSectionAfter(0, createForceDrivenSection(15));
        auto& force = std::get<ForceDrivenRegion>(std::get<GeometryRegion>(track.section(1).region).construction);
        force.targetNormalG.segments[0].transition.valueBegin = 1.3;
        force.targetNormalG.segments[0].transition.valueEnd = 0.8;
        force.targetNormalG.segments[0].transition.transitionType = quantum::math::TransitionType::Smootherstep;
        force.rollRate.segments[0].transition.valueBegin = 0.02;
        force.rollRate.segments[0].transition.valueEnd = 0.02;
        track.appendSection();
        convertSectionToPlanarArc(track.section(2));
        setSectionLength(track.section(2), 8);
        return track;
    }

    void centerlineAnchorsTopologyAndDiagnostics()
    {
        auto track = mixedTrack();
        auto physical = track.physicalSettings();
        physical.initialSpeed = 27;
        physical.metersPerCoordinateUnit = 0.5;
        track.setPhysicalSettings(physical);
        const auto centerline = createCenterlineVisualization(track);
        const auto canonical = integrateAuthoredTrackKinematics(track, centerlineVisualizationSampleSpacing);
        require(centerline.samples.size() == canonical.size(), "force visualization uses canonical generation");
        require(centerline.anchors.size() == 4 && centerline.sectionSlices.size() == 3, "mixed construction anchors and slices");
        for (std::size_t i = 0; i < canonical.size(); ++i)
            require(centerline.samples[i].position == canonical[i].position, "visualization does not rigidly transform force samples");
        for (std::size_t i = 0; i < centerline.sectionSlices.size(); ++i)
        {
            require(glm::length(centerline.sectionSlices[i].startPosition - centerline.anchors[i].position) < 1.0e-8,
                "sparse semantic anchors agree with visual section entry");
            require(glm::length(centerline.sectionSlices[i].endPosition - centerline.anchors[i + 1].position) < 1.0e-8,
                "sparse semantic anchors agree with visual section exit");
        }
        const auto topology = computeTrackTopology(track);
        require(glm::length(topology.endEndpoint.position - centerline.samples.back().position) < 1.0e-8,
            "topology dispatches force constructions");
        const auto history = evaluateRiderLoadDiagnostics(track);
        require(history.completed() && history.states.front().vehicleSpeed == 27,
            "editor diagnostics consume document settings");
        const auto expected = evaluateRiderLoads(canonical, riderLoadEvaluationSettings(track.physicalSettings()));
        require(history.states.size() == expected.states.size(), "diagnostics sample canonical states");
        for (std::size_t i = 0; i < expected.states.size(); ++i)
            require(history.states[i].normalG == expected.states[i].normalG
                && history.states[i].vehicleSpeed == expected.states[i].vehicleSpeed,
                "diagnostic values are universal evaluated truth");
    }

    void failedCandidatesDoNotPublish()
    {
        auto committed = mixedTrack();
        const auto serialized = serializeCoasterDocument(committed);
        CenterlineVisualizationCache cache;
        require(cache.rebuildIfDirty(committed), "initial cache build");
        const auto generation = cache.generation();
        const auto beforeVertices = cache.visualization().vertices;
        RiderLoadDiagnosticsModel diagnostics;
        diagnostics.update(committed, evaluateRiderLoadDiagnostics(committed));
        diagnostics.selectSection(1);
        const auto beforeDiagnostics = diagnostics.selectedSection();
        std::size_t selection = 1;
        std::size_t gpuUploads = 0;
        std::array<double, 3> buffers{15.0, 1.3, 0.02};

        for (int defect = 0; defect < 3; ++defect)
        {
            AuthoredTrackEditTransaction transaction{committed};
            transaction.stageSelectionAfterCommit(2);
            transaction.requestSectionLengthBufferSync();
            auto& candidate = transaction.candidate();
            if (defect == 0)
                candidate.setPhysicalSettings({0, 1, standardGravityAcceleration});
            else if (defect == 1)
                candidate.section(1).length = -1;
            else
            {
                // The force section itself succeeds. A later vertical arc
                // makes universal evaluation incomplete, so acceptance must
                // still reject before upload/commit.
                candidate.section(0) = createForceDrivenSection(8);
                candidate.section(1) = createRateProfileSection(15);
                setPlanarArcRadius(candidate.section(2), 100);
                setPlanarArcSweptAngle(candidate.section(2), 3.141592653589793);
                setPlanarArcPlaneTilt(candidate.section(2), 3.141592653589793 / 2.0);
            }
            bool rejected = false;
            try
            {
                auto geometry = createCenterlineVisualization(candidate);
                auto loads = evaluateRiderLoadDiagnostics(candidate);
                transaction.requireAcceptableRiderLoads(loads);
                ++gpuUploads; // Upload boundary; no real GPU needed for this gate test.
                cache.replace(std::move(geometry));
                transaction.commit(committed);
                diagnostics.update(committed, std::move(loads));
                selection = *transaction.selectionAfterCommit();
            }
            catch (const std::exception&) { rejected = true; }
            require(rejected && !transaction.committed() && !transaction.selectionAfterCommit(),
                "force generation/evaluation failures cannot publish selection");
            require(serializeCoasterDocument(committed) == serialized, "committed document unchanged");
            require(cache.generation() == generation && !cache.isDirty(), "committed geometry cache unchanged");
            require(gpuUploads == 0 && selection == 1 && buffers == std::array{15.0, 1.3, 0.02},
                "upload, selection, and edit buffers untouched");
            require(cache.visualization().vertices.size() == beforeVertices.size(), "vertex count unchanged");
            for (std::size_t i = 0; i < beforeVertices.size(); ++i)
            {
                const auto& a = cache.visualization().vertices[i];
                const auto& b = beforeVertices[i];
                require(a.x == b.x && a.y == b.y && a.z == b.z && a.color == b.color, "committed vertex data unchanged");
            }
            const auto& after = diagnostics.selectedSection();
            require(after.sectionIndex == beforeDiagnostics.sectionIndex && after.samples.size() == beforeDiagnostics.samples.size(),
                "diagnostic selection and count unchanged");
            for (std::size_t i = 0; i < after.samples.size(); ++i)
                require(after.samples[i].normalG == beforeDiagnostics.samples[i].normalG
                    && after.samples[i].lateralG == beforeDiagnostics.samples[i].lateralG
                    && after.samples[i].vehicleSpeed == beforeDiagnostics.samples[i].vehicleSpeed,
                    "committed diagnostics unchanged");
        }

        AuthoredTrackEditTransaction accepted{committed};
        auto physical = committed.physicalSettings();
        physical.initialSpeed = 30;
        accepted.candidate().setPhysicalSettings(physical);
        auto geometry = createCenterlineVisualization(accepted.candidate());
        auto loads = evaluateRiderLoadDiagnostics(accepted.candidate());
        accepted.requireAcceptableRiderLoads(loads);
        ++gpuUploads;
        cache.replace(std::move(geometry));
        accepted.commit(committed);
        diagnostics.update(committed, std::move(loads));
        require(accepted.committed() && committed.physicalSettings().initialSpeed == 30 && gpuUploads == 1,
            "valid physical document edit commits after evaluation and upload gate");

        // Preserve the pre-existing policy for rate/arc-only tracks.
        auto legacy = createNewDocument();
        AuthoredTrackEditTransaction legacyEdit{legacy};
        RiderLoadHistory incomplete;
        incomplete.unreachable = RiderLoadUnreachableState{5, -1};
        legacyEdit.requireAcceptableRiderLoads(incomplete);
    }
}

int main()
{
    try
    {
        centerlineAnchorsTopologyAndDiagnostics();
        failedCandidatesDoNotPublish();
        std::cout << "All force-driven acceptance tests passed.\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
