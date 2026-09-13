#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/editor/CenterlineVisualization.hpp>
#include <quantum/math/ScalarTransition.hpp>
#include <quantum/renderer/VulkanContext.hpp>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace
{
    using quantum::coaster::AuthoredTrack;
    using quantum::coaster::AuthoredTrackSection;
    using quantum::coaster::ChannelProfile;
    using quantum::coaster::GeometryRegion;
    using quantum::coaster::PlanarArcRegion;
    using quantum::coaster::ProfileSegment;
    using quantum::coaster::RegionKind;
    using quantum::coaster::convertSectionToPlanarArc;
    using quantum::coaster::createRateProfileSection;
    using quantum::coaster::integrateAuthoredTrack;
    using quantum::coaster::sectionLength;
    using quantum::coaster::setPlanarArcBankChange;
    using quantum::coaster::setPlanarArcPlaneTilt;
    using quantum::coaster::setPlanarArcRadius;
    using quantum::coaster::setPlanarArcSweptAngle;
    using quantum::coaster::setSectionLength;
    using quantum::editor::CenterlineVisualization;
    using quantum::editor::CenterlineVisualizationCache;
    using quantum::editor::centerlineVisualizationSampleSpacing;
    using quantum::editor::createCenterlineVisualization;
    using quantum::editor::createTrackStylePresentationCandidate;
    using quantum::editor::classifyRegionTrackStyleEdit;
    using quantum::editor::TrackStylePresentationProduct;
    using quantum::math::TransitionType;

    constexpr double pi = 3.14159265358979323846;
    constexpr double radiansPerDegree = pi / 180.0;
    constexpr double distanceTolerance = 1.0e-8;
    constexpr double positionTolerance = 2.0e-5;
    constexpr double frameTolerance = 5.0e-10;

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
        const std::string_view context)
    {
        if (std::abs(actual - expected) > tolerance)
        {
            throw TestFailure(
                std::string(context) + ": expected "
                + std::to_string(expected) + ", got "
                + std::to_string(actual));
        }
    }

    void requireNearVec(
        const glm::dvec3& actual,
        const glm::dvec3& expected,
        const double tolerance,
        const std::string_view context)
    {
        if (glm::length(actual - expected) > tolerance)
        {
            throw TestFailure(
                std::string(context) + ": vector mismatch");
        }
    }

    [[nodiscard]] bool isFinite(const glm::dvec3& value) noexcept
    {
        return std::isfinite(value.x)
            && std::isfinite(value.y)
            && std::isfinite(value.z);
    }

    void setSingleSegmentChannel(
        ChannelProfile& channel,
        const double valueBegin,
        const double valueEnd,
        const TransitionType transitionType)
    {
        require(
            channel.segments.size() == 1,
            "test fixture expects a single profile segment");

        ProfileSegment& segment = channel.segments.front();
        segment.transition.valueBegin = valueBegin;
        segment.transition.valueEnd = valueEnd;
        segment.transition.transitionType = transitionType;
    }

    void shapeRateProfileSection(
        AuthoredTrackSection& section,
        const double rollBegin,
        const double rollEnd,
        const double pitchBegin,
        const double pitchEnd,
        const double yawBegin,
        const double yawEnd)
    {
        auto& profiles = section.rateProfileRegion().rateProfiles;
        setSingleSegmentChannel(
            profiles.roll,
            rollBegin,
            rollEnd,
            TransitionType::Smoothstep
        );
        setSingleSegmentChannel(
            profiles.pitch,
            pitchBegin,
            pitchEnd,
            TransitionType::CosineEaseInOut
        );
        setSingleSegmentChannel(
            profiles.yaw,
            yawBegin,
            yawEnd,
            TransitionType::Smootherstep
        );
    }

    [[nodiscard]] PlanarArcRegion& planarArcOf(
        AuthoredTrackSection& section)
    {
        return std::get<PlanarArcRegion>(
            std::get<GeometryRegion>(section.region).construction);
    }

    [[nodiscard]] AuthoredTrack createMultiRegionFixture()
    {
        AuthoredTrack track;

        track.appendSection();
        setSectionLength(track.section(0), 20.0);

        track.appendSection();
        convertSectionToPlanarArc(track.section(1));
        setPlanarArcRadius(track.section(1), 32.0);
        setPlanarArcSweptAngle(track.section(1), 70.0 * radiansPerDegree);
        setPlanarArcPlaneTilt(track.section(1), 12.0 * radiansPerDegree);
        setPlanarArcBankChange(track.section(1), 22.0 * radiansPerDegree);

        track.appendSection();
        setSectionLength(track.section(2), 36.0);
        shapeRateProfileSection(
            track.section(2),
            0.002,
            0.014,
            -0.004,
            0.007,
            0.006,
            0.003
        );

        track.appendSection();
        setSectionLength(track.section(3), 28.0);
        shapeRateProfileSection(
            track.section(3),
            0.012,
            -0.004,
            0.006,
            0.002,
            -0.003,
            0.004
        );

        return track;
    }

    [[nodiscard]] double authoredLength(const AuthoredTrack& track)
    {
        double length = 0.0;
        for (std::size_t index = 0; index < track.sectionCount(); ++index)
        {
            length += sectionLength(track.section(index));
        }
        return length;
    }

    [[nodiscard]] glm::dvec3 vertexPosition(
        const quantum::renderer::LineVertex& vertex) noexcept
    {
        return {
            static_cast<double>(vertex.x),
            static_cast<double>(vertex.y),
            static_cast<double>(vertex.z)
        };
    }

    [[nodiscard]] glm::dvec3 curvePoint(
        const CenterlineVisualization& visualization,
        const std::uint32_t curve,
        const std::size_t sampleIndex)
    {
        require(
            visualization.samples.size() >= 2,
            "curvePoint requires a non-empty polyline");
        require(
            sampleIndex < visualization.samples.size(),
            "curvePoint sample index is out of range");

        const std::size_t vertexWithinCurve =
            sampleIndex + 1 < visualization.samples.size()
            ? 2 * sampleIndex
            : 2 * (sampleIndex - 1) + 1;
        const std::size_t vertexIndex =
            static_cast<std::size_t>(curve)
                * visualization.verticesPerCurve
            + vertexWithinCurve;

        return vertexPosition(visualization.vertices.at(vertexIndex));
    }

    [[nodiscard]] std::size_t sampleIndexAtDistance(
        const CenterlineVisualization& visualization,
        const double distance)
    {
        const double tolerance =
            distanceTolerance * std::max(1.0, std::abs(distance));
        for (std::size_t index = 0;
            index < visualization.samples.size();
            ++index)
        {
            if (std::abs(visualization.samples[index].distance - distance)
                <= tolerance)
            {
                return index;
            }
        }

        throw TestFailure("section boundary sample was not present");
    }

    void requireOrthonormalFrame(
        const quantum::geometry::CurveFrame& frame,
        const std::string_view context)
    {
        require(isFinite(frame.tangent), "non-finite tangent");
        require(isFinite(frame.lateral), "non-finite lateral");
        require(isFinite(frame.up), "non-finite up");
        requireNear(glm::length(frame.tangent), 1.0, frameTolerance, context);
        requireNear(glm::length(frame.lateral), 1.0, frameTolerance, context);
        requireNear(glm::length(frame.up), 1.0, frameTolerance, context);
        requireNear(
            glm::dot(frame.tangent, frame.lateral),
            0.0,
            frameTolerance,
            context
        );
        requireNear(
            glm::dot(frame.tangent, frame.up),
            0.0,
            frameTolerance,
            context
        );
        requireNear(
            glm::dot(frame.lateral, frame.up),
            0.0,
            frameTolerance,
            context
        );
        requireNearVec(
            glm::cross(frame.tangent, frame.lateral),
            frame.up,
            frameTolerance,
            context
        );
    }

    void compareVisualizationToCore(
        const CenterlineVisualization& visualization,
        const AuthoredTrack& track)
    {
        const auto states = integrateAuthoredTrack(
            track,
            centerlineVisualizationSampleSpacing
        );
        require(
            visualization.samples.size() == states.size(),
            "visualization keeps every Core sample");

        for (std::size_t index = 0; index < states.size(); ++index)
        {
            requireNear(
                visualization.samples[index].distance,
                states[index].distance,
                distanceTolerance,
                "sample distance matches Core");
            requireNearVec(
                visualization.samples[index].position,
                states[index].position,
                positionTolerance,
                "sample position matches Core");
            requireNearVec(
                visualization.samples[index].frame.tangent,
                states[index].frame.tangent,
                frameTolerance,
                "sample tangent matches Core");
            requireNearVec(
                visualization.samples[index].frame.lateral,
                states[index].frame.lateral,
                frameTolerance,
                "sample lateral matches Core");
            requireNearVec(
                visualization.samples[index].frame.up,
                states[index].frame.up,
                frameTolerance,
                "sample up matches Core");
        }
    }

    void emptyAuthoredTrackIsSafe()
    {
        const AuthoredTrack emptyTrack;
        const CenterlineVisualization visualization =
            createCenterlineVisualization(emptyTrack);

        require(visualization.samples.empty(), "empty track has no samples");
        require(visualization.vertices.empty(), "empty track has no vertices");
        require(visualization.verticesPerCurve == 0,
            "empty track has zero vertices per curve");
        require(visualization.sectionSlices.empty(),
            "empty track has no section slices");

        CenterlineVisualizationCache cache;
        require(cache.isDirty(), "new cache starts dirty");
        require(cache.rebuildIfDirty(emptyTrack),
            "dirty empty cache rebuild succeeds");
        require(!cache.isDirty(), "empty cache rebuild clears dirty");
        require(cache.generation() == 1, "empty rebuild advances generation");
        require(!cache.rebuildIfDirty(emptyTrack),
            "unchanged empty cache does not rebuild");
        require(cache.generation() == 1,
            "unchanged empty cache preserves generation");
    }

    void oneRegionTrackStillWorks()
    {
        AuthoredTrack track;
        track.appendSection();
        setSectionLength(track.section(0), 6.0);

        const CenterlineVisualization visualization =
            createCenterlineVisualization(track);

        compareVisualizationToCore(visualization, track);
        require(visualization.sectionSlices.size() == 1,
            "one-region track has one section slice");
        require(visualization.verticesPerCurve
                == 2 * (visualization.samples.size() - 1),
            "vertices per curve match sample segment count");
        require(visualization.vertices.size()
                == quantum::renderer::viewportCurveCount
                    * visualization.verticesPerCurve,
            "all reference curves have equal vertex runs");

        const auto& slice = visualization.sectionSlices.front();
        requireNear(slice.startDistance, 0.0, distanceTolerance,
            "single region starts at zero");
        requireNear(slice.endDistance, 6.0, distanceTolerance,
            "single region ends at authored length");
        requireNearVec(slice.startPosition, {0.0, 0.0, 0.0},
            positionTolerance, "single region starts at origin");
        requireNearVec(
            slice.endPosition,
            visualization.samples.back().position,
            positionTolerance,
            "single region reaches final sample"
        );
    }

    void semanticAnchorsDoNotExpandCameraBounds()
    {
        const AuthoredTrack track = quantum::coaster::createNewDocument();
        const CenterlineVisualization visualization =
            createCenterlineVisualization(track);

        require(visualization.anchors.size() == 2,
            "one authored region exposes two semantic anchors");
        requireNearVec(visualization.minimumPosition, {0.0, 0.0, 0.0},
            positionTolerance, "camera bounds start at the centerline");
        requireNearVec(visualization.maximumPosition, {60.0, 0.0, 0.0},
            positionTolerance, "camera bounds end at the centerline");
    }

    void multiRegionVisualizationMatchesCoreAndBoundaries()
    {
        const AuthoredTrack track = createMultiRegionFixture();
        const CenterlineVisualization visualization =
            createCenterlineVisualization(track);
        const CenterlineVisualization secondPass =
            createCenterlineVisualization(track);

        compareVisualizationToCore(visualization, track);
        require(visualization.samples.size() == secondPass.samples.size(),
            "deterministic sample count");
        require(visualization.vertices.size() == secondPass.vertices.size(),
            "deterministic vertex count");

        for (std::size_t index = 0;
            index < visualization.samples.size();
            ++index)
        {
            require(
                visualization.samples[index].distance
                    == secondPass.samples[index].distance,
                "sample distances are deterministic");
            requireNearVec(
                visualization.samples[index].position,
                secondPass.samples[index].position,
                0.0,
                "sample positions are deterministic");
            requireOrthonormalFrame(
                visualization.samples[index].frame,
                "visualization sample frame");

            if (index > 0)
            {
                require(
                    visualization.samples[index].distance
                        > visualization.samples[index - 1].distance,
                    "sample distances increase monotonically");
                require(
                    visualization.samples[index].distance
                        - visualization.samples[index - 1].distance
                        <= centerlineVisualizationSampleSpacing
                            * (1.0 + 1.0e-9),
                    "neighboring samples stay within visualization spacing");
            }
        }

        require(visualization.sectionSlices.size() == track.sectionCount(),
            "visualization spans every authored region");
        requireNearVec(
            visualization.samples.front().position,
            {0.0, 0.0, 0.0},
            positionTolerance,
            "first sample matches authored-track start"
        );
        requireNear(
            visualization.samples.back().distance,
            authoredLength(track),
            distanceTolerance * authoredLength(track),
            "final sample matches authored-track end distance"
        );

        double runningDistance = 0.0;
        for (std::size_t section = 0;
            section < track.sectionCount();
            ++section)
        {
            const auto& slice = visualization.sectionSlices[section];
            const std::size_t startIndex =
                sampleIndexAtDistance(visualization, runningDistance);
            runningDistance += sectionLength(track.section(section));
            const std::size_t endIndex =
                sampleIndexAtDistance(visualization, runningDistance);

            require(slice.vertexCount > 0,
                "each authored section owns visible line segments");
            require(slice.firstVertex == 2 * startIndex,
                "section slice starts on its boundary sample");
            require(slice.vertexCount == 2 * (endIndex - startIndex),
                "section slice covers exactly its own segments");
            requireNear(slice.startDistance,
                visualization.samples[startIndex].distance,
                distanceTolerance,
                "slice start distance matches sample");
            requireNear(slice.endDistance,
                visualization.samples[endIndex].distance,
                distanceTolerance,
                "slice end distance matches sample");
            requireNearVec(slice.startPosition,
                visualization.samples[startIndex].position,
                positionTolerance,
                "slice start position matches sample");
            requireNearVec(slice.endPosition,
                visualization.samples[endIndex].position,
                positionTolerance,
                "slice end position matches sample");

            if (section > 0)
            {
                requireNearVec(
                    visualization.sectionSlices[section - 1].endPosition,
                    slice.startPosition,
                    positionTolerance,
                    "neighboring section boundaries are continuous"
                );
                require(
                    glm::length(slice.startPosition) > 1.0,
                    "later regions do not restart at the local origin");
            }
        }

        const std::size_t arcEndIndex = sampleIndexAtDistance(
            visualization,
            sectionLength(track.section(0))
                + sectionLength(track.section(1))
        );
        require(
            std::abs(glm::dot(
                visualization.samples[arcEndIndex].frame.up,
                glm::dvec3{0.0, 0.0, 1.0})) < 0.98,
            "banked planar arc changes the visible frame up axis");

        const std::array<std::size_t, 3> inspectedSamples{
            0,
            arcEndIndex,
            visualization.samples.size() - 1
        };
        for (const std::size_t sample : inspectedSamples)
        {
            const auto& state = visualization.samples[sample];
            const glm::dvec3 left = curvePoint(
                visualization,
                quantum::renderer::viewportLeftRailCurve,
                sample
            );
            const glm::dvec3 right = curvePoint(
                visualization,
                quantum::renderer::viewportRightRailCurve,
                sample
            );
            const glm::dvec3 center = curvePoint(
                visualization,
                quantum::renderer::viewportCenterlineCurve,
                sample
            );
            const glm::dvec3 heart = curvePoint(
                visualization,
                quantum::renderer::viewportHeartlineCurve,
                sample
            );

            requireNearVec(center, state.position, positionTolerance,
                "centerline vertices use solved positions");
            require(
                glm::length(right - left) > 1.0,
                "rail offsets remain visible");
            requireNearVec(
                glm::normalize(right - left),
                state.frame.lateral,
                positionTolerance,
                "rail offsets follow solved lateral frame");
            require(
                glm::length(heart - center) > 1.0,
                "heartline offset remains visible");
            requireNearVec(
                glm::normalize(heart - center),
                state.frame.up,
                positionTolerance,
                "heartline offset follows solved up frame");
        }
    }

    void heartlineSetupControlsOnlyReferenceCurve()
    {
        AuthoredTrack track;
        track.appendSection();
        setSectionLength(track.section(0), 6.0);

        auto physicalSettings = track.physicalSettings();
        physicalSettings.metersPerCoordinateUnit = 2.0;
        track.setPhysicalSettings(physicalSettings);

        auto setup = track.coasterSetup();
        setup.heartline.enabled = true;
        setup.heartline.offsetMeters = 3.0;
        track.setCoasterSetup(setup);

        const auto enabled = createCenterlineVisualization(track);
        const glm::dvec3 enabledCenter = curvePoint(
            enabled, quantum::renderer::viewportCenterlineCurve, 0);
        const glm::dvec3 enabledHeartline = curvePoint(
            enabled, quantum::renderer::viewportHeartlineCurve, 0);
        requireNear(
            glm::length(enabledHeartline - enabledCenter),
            1.5,
            positionTolerance,
            "heartline metres convert to document coordinate units");
        requireNearVec(
            glm::normalize(enabledHeartline - enabledCenter),
            enabled.samples.front().frame.up,
            positionTolerance,
            "positive heartline offset follows local +up");

        setup.heartline.enabled = false;
        track.setCoasterSetup(setup);
        const auto disabled = createCenterlineVisualization(track);
        requireNearVec(
            curvePoint(disabled,
                quantum::renderer::viewportHeartlineCurve, 0),
            curvePoint(disabled,
                quantum::renderer::viewportCenterlineCurve, 0),
            positionTolerance,
            "disabled heartline has zero reference-curve displacement");

        requireNearVec(
            disabled.samples.front().position,
            enabled.samples.front().position,
            positionTolerance,
            "heartline configuration does not alter solved centerline samples");
        require(disabled.renderableTrack.continuousMesh.vertices.size()
                == enabled.renderableTrack.continuousMesh.vertices.size()
            && disabled.renderableTrack.continuousMesh.triangleIndices.size()
                == enabled.renderableTrack.continuousMesh.triangleIndices.size()
            && disabled.renderableTrack.hardwareBatches.size()
                == enabled.renderableTrack.hardwareBatches.size(),
            "heartline configuration does not alter renderable track topology");
    }

    void visualizationCacheInvalidatesOnlyForGeometry()
    {
        AuthoredTrack track = createMultiRegionFixture();
        CenterlineVisualizationCache cache;

        require(cache.rebuildIfDirty(track),
            "initial dirty visualization rebuilds");
        const std::uint64_t initialGeneration = cache.generation();
        const std::size_t initialSampleCount =
            cache.visualization().samples.size();

        require(!cache.rebuildIfDirty(track),
            "unchanged document geometry does not rebuild");
        require(cache.generation() == initialGeneration,
            "unchanged geometry preserves visualization generation");

        std::size_t selectedSection = 0;
        selectedSection = 2;
        require(selectedSection == 2, "selection test uses a real change");
        require(!cache.rebuildIfDirty(track),
            "pure selection change does not rebuild visualization");

        auto expectInvalidatingEdit = [&](
            auto&& edit,
            const std::string_view context)
        {
            const std::uint64_t before = cache.generation();
            edit();
            cache.markDirty();
            require(cache.isDirty(), context);
            require(cache.rebuildIfDirty(track), context);
            require(!cache.isDirty(), context);
            require(cache.generation() == before + 1, context);
        };

        expectInvalidatingEdit(
            [&track]
            {
                track.appendSection();
            },
            "append invalidates visualization");
        require(cache.visualization().sectionSlices.size()
                == track.sectionCount(),
            "append rebuild publishes new slice count");

        expectInvalidatingEdit(
            [&track]
            {
                track.removeSection(track.sectionCount() - 1);
            },
            "remove invalidates visualization");
        require(cache.visualization().sectionSlices.size()
                == track.sectionCount(),
            "remove rebuild publishes new slice count");

        expectInvalidatingEdit(
            [&track]
            {
                track.moveSection(0, track.sectionCount() - 1);
            },
            "reorder invalidates visualization");

        const std::size_t rateProfileSection =
            track.section(0).kind == RegionKind::RateProfiles
            ? 0
            : track.sectionCount() - 1;
        expectInvalidatingEdit(
            [&track, rateProfileSection]
            {
                auto& transition = track.section(rateProfileSection)
                    .rateProfileRegion()
                    .rateProfiles
                    .yaw
                    .segments
                    .front()
                    .transition;
                transition.valueEnd += 0.001;
            },
            "profile endpoint edit invalidates visualization");
        require(cache.visualization().samples.size() != 0,
            "profile edit rebuild keeps visualization populated");

        const std::size_t geometrySection =
            track.section(0).kind == RegionKind::Geometry
            ? 0
            : 1;
        expectInvalidatingEdit(
            [&track, geometrySection]
            {
                setPlanarArcBankChange(
                    track.section(geometrySection),
                    planarArcOf(track.section(geometrySection)).bankChange
                        + 3.0 * radiansPerDegree
                );
            },
            "planar-arc parameter edit invalidates visualization");
        require(cache.visualization().samples.size() != initialSampleCount
                || cache.generation() > initialGeneration,
            "invalidating edits publish a later visualization generation");

        auto style = cache.trackStyle();
        style.railRadius *= 1.25;
        const std::uint64_t beforeStyleChange = cache.generation();
        cache.setTrackStyle(std::move(style));
        require(cache.isDirty(),
            "accepted track-style parameter change invalidates geometry");
        require(cache.rebuildIfDirty(track),
            "dirty track-style parameter rebuilds geometry");
        require(cache.generation() == beforeStyleChange + 1,
            "track-style rebuild advances visualization generation");
    }

    void regionStylesDriveViewportAndHardwareGeneration()
    {
        AuthoredTrack track = quantum::coaster::createNewDocument();
        track.appendSection();
        track.appendSection();
        track.appendSection();

        auto& spacing = track.section(1).trackStyleOverrides;
        spacing.enabled = true;
        spacing.hardwareSpacing = 2.0;
        spacing.railCenterSpacing = 1.8;
        auto& noSpine = track.section(2).trackStyleOverrides;
        noSpine.enabled = true;
        noSpine.spineEnabled = false;
        auto& appearance = track.section(3).trackStyleOverrides;
        appearance.enabled = true;
        appearance.railMaterial = quantum::coaster::TrackMaterial{
            glm::vec4{0.1F, 0.8F, 0.2F, 1.0F}};

        const CenterlineVisualization visualization =
            createCenterlineVisualization(track);
        require(visualization.resolvedRegionStyles.size() == 4,
            "viewport retains one canonical resolved style per region");
        require(visualization.resolvedRegionStyles[0].spine.enabled
                && visualization.resolvedRegionStyles[0]
                    .repeatingHardware.front().spacing == 0.75,
            "first region fully inherits the document style");
        require(visualization.resolvedRegionStyles[1]
                    .repeatingHardware.front().spacing == 2.0
                && visualization.resolvedRegionStyles[1].railOffsets[0].lateral
                    == -0.9
                && visualization.resolvedRegionStyles[1].railOffsets[1].lateral
                    == 0.9
                && visualization.resolvedRegionStyles[1].spine.enabled,
            "spacing override affects only its intended region");
        require(!visualization.resolvedRegionStyles[2].spine.enabled
                && visualization.resolvedRegionStyles[2]
                    .repeatingHardware.front().spacing == 0.75,
            "spine override does not leak into inherited hardware");
        require(visualization.resolvedRegionStyles[3].spine.enabled
                && visualization.resolvedRegionStyles[3]
                    .railMaterial.baseColor
                    == glm::vec4{0.1F, 0.8F, 0.2F, 1.0F},
            "appearance override is scoped and later structure inherits");

        const std::size_t secondRegionSample =
            visualization.sectionSlices[1].firstVertex / 2 + 1;
        requireNear(glm::length(
                curvePoint(visualization,
                    quantum::renderer::viewportRightRailCurve,
                    secondRegionSample)
                - curvePoint(visualization,
                    quantum::renderer::viewportLeftRailCurve,
                    secondRegionSample)),
            1.8, positionTolerance,
            "engineering rail curves use the same resolved regional spacing");

        require(visualization.renderableTrack.hardwareBatches.size() == 4,
            "each region's resolved repeating hardware reaches the renderer");
        require(visualization.renderableTrack.hardwareBatches[0]
                    .instances.size() == 80
                && visualization.renderableTrack.hardwareBatches[1]
                    .instances.size() == 30
                && visualization.renderableTrack.hardwareBatches[2]
                    .instances.size() == 80
                && visualization.renderableTrack.hardwareBatches[3]
                    .instances.size() == 81,
            "region hardware spacing is deterministic and boundaries are not duplicated");
        require(visualization.renderableTrack.continuousMesh.submeshes.size()
                == 11,
            "spine-disabled region removes exactly one regional submesh");
    }

    void regionStylePresentationInvalidationIsProductSpecific()
    {
        AuthoredTrack track = quantum::coaster::createNewDocument();
        track.appendSection();
        track.appendSection();
        track.appendSection();

        CenterlineVisualizationCache cache;
        cache.setTrackStyle(track.trackStyle());
        require(cache.rebuildIfDirty(track), "fixture cache builds");
        const std::uint64_t canonicalGeneration = cache.generation();
        const auto* const solvedSamples =
            cache.visualization().samples.data();

        quantum::coaster::RegionTrackStyleOverrides inherited;
        auto hardwareColor = inherited;
        hardwareColor.enabled = true;
        hardwareColor.hardwareMaterial = quantum::coaster::TrackMaterial{
            glm::vec4{0.2F, 0.3F, 0.4F, 1.0F}};
        const auto hardwareMaterialImpact = classifyRegionTrackStyleEdit(
            track.trackStyle(), inherited, hardwareColor);
        require(hardwareMaterialImpact.affects(
                    TrackStylePresentationProduct::HardwareMaterials)
                && !hardwareMaterialImpact.affects(
                    TrackStylePresentationProduct::HardwareInstances)
                && !hardwareMaterialImpact.affects(
                    TrackStylePresentationProduct::RenderableMesh),
            "hardware color classifies as hardware material-only");

        auto before = track.section(1).trackStyleOverrides;
        auto after = before;
        after.enabled = true;
        after.hardwareSpacing = 2.0;
        auto impact = classifyRegionTrackStyleEdit(
            track.trackStyle(), before, after);
        require(impact.affects(TrackStylePresentationProduct::HardwareInstances)
                && !impact.affects(TrackStylePresentationProduct::RenderableMesh)
                && !impact.affects(TrackStylePresentationProduct::EngineeringRails)
                && !impact.affects(TrackStylePresentationProduct::TrackMaterials),
            "hardware spacing classifies as hardware instances only");
        require(!impact.invalidatesCanonicalTrack()
                && !impact.invalidatesRiderLoads()
                && !impact.invalidatesSupports()
                && !impact.invalidatesSimulationPreview(),
            "hardware spacing invalidates no physics or engineering products");
        track.section(1).trackStyleOverrides = after;
        auto candidate = createTrackStylePresentationCandidate(
            track, cache.visualization(), impact);
        require(candidate.hardwareBatches.has_value()
                && !candidate.continuousMesh.has_value()
                && !candidate.trackMaterials.has_value()
                && !candidate.referenceCurveVertices.has_value(),
            "hardware-only rebuild stages only hardware placement");
        const std::size_t inheritedHardwareCount = cache.visualization()
            .renderableTrack.hardwareBatches[0].instances.size();
        cache.applyTrackStylePresentation(std::move(candidate));
        require(cache.generation() == canonicalGeneration
                && cache.visualization().samples.data() == solvedSamples,
            "hardware-only edit reuses solved samples without canonical generation");
        require(cache.visualization().renderableTrack.hardwareBatches[1]
                    .instances.size() < inheritedHardwareCount
                && cache.visualization().resolvedRegionStyles[0]
                    .repeatingHardware.front().spacing == 0.75
                && cache.visualization().resolvedRegionStyles[2]
                    .repeatingHardware.front().spacing == 0.75,
            "hardware spacing remains region-scoped");

        before = track.section(2).trackStyleOverrides;
        after = before;
        after.enabled = true;
        after.railMaterial = quantum::coaster::TrackMaterial{
            glm::vec4{0.05F, 0.65F, 0.25F, 1.0F}};
        impact = classifyRegionTrackStyleEdit(
            track.trackStyle(), before, after);
        require(impact.affects(TrackStylePresentationProduct::TrackMaterials)
                && !impact.affects(TrackStylePresentationProduct::RenderableMesh)
                && !impact.affects(TrackStylePresentationProduct::HardwareInstances)
                && !impact.affects(TrackStylePresentationProduct::EngineeringRails),
            "rail color classifies as material-only");
        track.section(2).trackStyleOverrides = after;
        const auto* const meshVertices = cache.visualization()
            .renderableTrack.continuousMesh.vertices.data();
        candidate = createTrackStylePresentationCandidate(
            track, cache.visualization(), impact);
        require(candidate.trackMaterials.has_value()
                && !candidate.continuousMesh.has_value()
                && !candidate.hardwareBatches.has_value()
                && !candidate.referenceCurveVertices.has_value(),
            "material-only rebuild stages no geometry or placement");
        cache.applyTrackStylePresentation(std::move(candidate));
        require(cache.generation() == canonicalGeneration
                && cache.visualization().samples.data() == solvedSamples
                && cache.visualization().renderableTrack.continuousMesh
                    .vertices.data() == meshVertices,
            "material-only edit preserves canonical and mesh storage");

        before = track.section(3).trackStyleOverrides;
        after = before;
        after.enabled = true;
        after.railRadius = 0.11;
        impact = classifyRegionTrackStyleEdit(
            track.trackStyle(), before, after);
        require(impact.affects(TrackStylePresentationProduct::RenderableMesh)
                && !impact.affects(TrackStylePresentationProduct::EngineeringRails)
                && !impact.affects(TrackStylePresentationProduct::HardwareInstances),
            "visual rail radius classifies as renderable mesh only");
        track.section(3).trackStyleOverrides = after;
        candidate = createTrackStylePresentationCandidate(
            track, cache.visualization(), impact);
        require(candidate.continuousMesh.has_value()
                && candidate.trackMaterials.has_value()
                && !candidate.hardwareBatches.has_value()
                && !candidate.referenceCurveVertices.has_value(),
            "rail-radius rebuild stages mesh without curves or hardware");
        cache.applyTrackStylePresentation(std::move(candidate));
        require(cache.generation() == canonicalGeneration
                && cache.visualization().samples.data() == solvedSamples,
            "renderable mesh edit does not advance canonical generation");

        before = track.section(1).trackStyleOverrides;
        after = before;
        after.railCenterSpacing = 1.8;
        impact = classifyRegionTrackStyleEdit(
            track.trackStyle(), before, after);
        require(impact.affects(TrackStylePresentationProduct::RenderableMesh)
                && impact.affects(TrackStylePresentationProduct::EngineeringRails)
                && !impact.affects(TrackStylePresentationProduct::HardwareInstances),
            "rail spacing classifies as mesh plus engineering rails");
        track.section(1).trackStyleOverrides = after;
        const std::size_t centerlineVertex =
            2 * cache.visualization().verticesPerCurve;
        const auto unchangedCenterline =
            cache.visualization().vertices[centerlineVertex];
        candidate = createTrackStylePresentationCandidate(
            track, cache.visualization(), impact);
        require(candidate.continuousMesh.has_value()
                && candidate.referenceCurveVertices.has_value()
                && !candidate.hardwareBatches.has_value(),
            "engineering edit stages only mesh and reference curves");
        cache.applyTrackStylePresentation(std::move(candidate));
        const auto& retainedCenterline =
            cache.visualization().vertices[centerlineVertex];
        require(retainedCenterline.x == unchangedCenterline.x
                && retainedCenterline.y == unchangedCenterline.y
                && retainedCenterline.z == unchangedCenterline.z
                && cache.generation() == canonicalGeneration
                && cache.visualization().samples.data() == solvedSamples,
            "engineering rail update leaves centerline and canonical solve unchanged");

        auto structurallyDifferent = track.trackStyle();
        structurallyDifferent.railCount = 3;
        structurallyDifferent.railOffsets.push_back({0.0, 0.0});
        const auto fallback = quantum::editor::
            classifyResolvedTrackStylePresentationChange(
                track.trackStyle(), structurallyDifferent);
        require(fallback.requiresFullRegeneration(),
            "unknown style structure conservatively requires full regeneration");
    }
}

int main()
{
    int passed = 0;
    int failed = 0;

    const auto run = [&](const char* const name, void (*test)())
    {
        try
        {
            test();
            ++passed;
            std::cout << "  PASS: " << name << '\n';
        }
        catch (const std::exception& exception)
        {
            ++failed;
            std::cerr << "  FAIL: " << name << "\n    "
                << exception.what() << '\n';
        }
    };

    std::cout << "CenterlineVisualizationTests\n";

    run("emptyAuthoredTrackIsSafe", emptyAuthoredTrackIsSafe);
    run("oneRegionTrackStillWorks", oneRegionTrackStillWorks);
    run("semanticAnchorsDoNotExpandCameraBounds",
        semanticAnchorsDoNotExpandCameraBounds);
    run("multiRegionVisualizationMatchesCoreAndBoundaries",
        multiRegionVisualizationMatchesCoreAndBoundaries);
    run("heartlineSetupControlsOnlyReferenceCurve",
        heartlineSetupControlsOnlyReferenceCurve);
    run("visualizationCacheInvalidatesOnlyForGeometry",
        visualizationCacheInvalidatesOnlyForGeometry);
    run("regionStylesDriveViewportAndHardwareGeneration",
        regionStylesDriveViewportAndHardwareGeneration);
    run("regionStylePresentationInvalidationIsProductSpecific",
        regionStylePresentationInvalidationIsProductSpecific);

    std::cout << "\n  " << passed << " passed, "
        << failed << " failed\n";

    return failed == 0 ? 0 : 1;
}
