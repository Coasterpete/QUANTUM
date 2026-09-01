#include <quantum/editor/TransitionEditorModel.hpp>
#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <quantum/coaster/CoasterDocument.hpp>

#include <array>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace
{
    using quantum::editor::CurveHitCandidate;
    using quantum::editor::GraphMarkerId;
    using quantum::editor::GraphValueRange;
    using quantum::editor::MarkerHitCandidate;
    using quantum::editor::RateChannel;
    using quantum::editor::ScalarProfileEndpoint;

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
                + std::to_string(actual)
            );
        }
    }

    template<typename Function>
    void requireInvalidArgument(
        Function&& function,
        const std::string_view message)
    {
        try
        {
            function();
        }
        catch (const std::invalid_argument&)
        {
            return;
        }
        throw TestFailure(std::string(message));
    }

    void conversionRelationships()
    {
        using namespace quantum::editor;

        const double curvature = angularRateDegreesToCurvature(1.0);
        requireNear(curvature, radiansPerDegree, 1e-15,
            "one degree per meter converts to radians per meter");
        requireNear(curvatureToAngularRateDegrees(curvature), 1.0, 1e-14,
            "curvature converts back to angular rate");
        requireNear(
            angularRateDegreesToRadians(48.0),
            48.0 * radiansPerDegree,
            1e-15,
            "large Roll Rate converts without a curvature assumption"
        );
        requireNear(
            angularRateRadiansToDegrees(48.0 * radiansPerDegree),
            48.0,
            1e-13,
            "large Roll Rate display conversion round trips"
        );

        const auto oneDegreeRadius = angularRateRadiusMeters(1.0);
        require(oneDegreeRadius.has_value(),
            "nonzero rate has a finite radius");
        requireNear(*oneDegreeRadius, degreesPerRadian, 1e-12,
            "one degree per meter radius");

        requireNear(radiusToAngularRateDegrees(20.0, 1.0),
            2.864788975654116, 1e-12, "20 meter positive radius");
        requireNear(radiusToAngularRateDegrees(20.0, -1.0),
            -2.864788975654116, 1e-12, "20 meter negative direction");
        requireNear(radiusToAngularRateDegrees(100.0, 1.0),
            0.5729577951308232, 1e-13, "100 meter radius");

        const double signedRates[] = {-6.0, -1.0, 0.25, 3.0, 6.0};
        for (const double rate : signedRates)
        {
            const auto radius = angularRateRadiusMeters(rate);
            require(radius.has_value(), "signed nonzero rate has radius");
            requireNear(
                radiusToAngularRateDegrees(*radius, rate),
                rate,
                1e-12,
                "rate/radius round trip preserves sign"
            );
        }

        require(!angularRateRadiusMeters(0.0).has_value(),
            "zero rate presents as straight");
        require(!curvatureRadiusMeters(1e-13).has_value(),
            "effectively zero curvature presents as straight");
        requireInvalidArgument(
            [] { static_cast<void>(radiusToAngularRateDegrees(0.0, 1.0)); },
            "zero editable radius must be rejected"
        );
        requireInvalidArgument(
            []
            {
                static_cast<void>(angularRateDegreesToCurvature(
                    std::numeric_limits<double>::infinity()
                ));
            },
            "non-finite rate must be rejected"
        );
    }

    void resultantCurvatureUsesOrthogonalPitchYawComponents()
    {
        using namespace quantum::editor;

        const auto diagnostic = resultantCurvatureDiagnostic(0.03, -0.04);
        requireNear(diagnostic.curvaturePerMeter, 0.05, 1e-15,
            "resultant curvature is pitch/yaw hypotenuse");
        require(diagnostic.radiusMeters.has_value(),
            "nonzero resultant has radius");
        requireNear(*diagnostic.radiusMeters, 20.0, 1e-12,
            "resultant radius is inverse curvature magnitude");
    }

    void independentGraphTransformsShareOneCanvas()
    {
        using namespace quantum::editor;

        const GraphValueRange pitch{
            -3.0 * radiansPerDegree,
            3.0 * radiansPerDegree};
        const GraphValueRange yaw{
            -0.5 * radiansPerDegree,
            0.5 * radiansPerDegree};
        const GraphValueRange roll{
            -30.0 * radiansPerDegree,
            30.0 * radiansPerDegree};

        const double normalizedPitch = graphValueToNormalized(
            1.5 * radiansPerDegree, pitch);
        const double normalizedYaw = graphValueToNormalized(
            0.25 * radiansPerDegree, yaw);
        const double normalizedRoll = graphValueToNormalized(
            15.0 * radiansPerDegree, roll);
        requireNear(normalizedPitch, 0.75, 1e-15,
            "pitch maps into shared canvas");
        requireNear(normalizedYaw, normalizedPitch, 1e-15,
            "independent yaw range may share the same screen position");
        requireNear(normalizedRoll, normalizedPitch, 1e-15,
            "independent roll range may share the same screen position");

        requireNear(normalizedToGraphValue(normalizedPitch, pitch),
            1.5 * radiansPerDegree, 1e-15,
            "pitch graph transform round trip");
        requireNear(normalizedToGraphValue(normalizedYaw, yaw),
            0.25 * radiansPerDegree, 1e-15,
            "yaw graph transform round trip");
        requireNear(normalizedToGraphValue(normalizedRoll, roll),
            15.0 * radiansPerDegree, 1e-15,
            "roll graph transform round trip");

        const std::array<double, 3> values{-0.01, 0.0, 0.02};
        const GraphValueRange fitted = fitSymmetricGraphRange(
            values,
            0.001
        );
        require(fitted.valid() && fitted.minimum == -fitted.maximum,
            "auto-fit produces a valid symmetric range");
        require(fitted.maximum > 0.02,
            "auto-fit pads the visible profile extent");

        const std::array<double, 2> flat{0.0, 0.0};
        const GraphValueRange flatRange = fitSymmetricGraphRange(
            flat,
            radiansPerDegree
        );
        requireNear(flatRange.maximum, radiansPerDegree, 0.0,
            "flat profiles retain a useful presentation span");

        const GraphValueRange zoomed = scaleGraphRange(fitted, 2.0);
        requireNear(zoomed.magnitude(), fitted.magnitude() * 2.0, 1e-15,
            "range control changes presentation only");
        requireNear(graphValueUnitsPerPixel(pitch, 240.0),
            (pitch.maximum - pitch.minimum) / 240.0, 1e-18,
            "drag sensitivity follows the active channel range");

        requireNear(defaultGraphMagnitude(RateChannel::Roll),
            30.0 * radiansPerDegree, 0.0,
            "flat Roll view supports practical large-rate dragging");
        requireNear(defaultGraphMagnitude(RateChannel::Pitch),
            2.0 * radiansPerDegree, 0.0,
            "flat Pitch view keeps its independent engineering context");
        requireNear(defaultGraphMagnitude(RateChannel::Yaw),
            0.5 * radiansPerDegree, 0.0,
            "flat Yaw view keeps its independent engineering context");
    }

    void largeRollRatesRemainRepresentableInTheGraph()
    {
        using namespace quantum::editor;

        const std::array<double, 3> rollValues{
            0.0,
            24.0 * radiansPerDegree,
            48.0 * radiansPerDegree
        };
        const GraphValueRange fitted = fitSymmetricGraphRange(
            rollValues,
            radiansPerDegree
        );

        require(
            fitted.maximum > 48.0 * radiansPerDegree,
            "roll auto-fit pads rather than truncates a 48 deg/m endpoint"
        );
        for (const double value : rollValues)
        {
            requireNear(
                normalizedToGraphValue(
                    graphValueToNormalized(value, fitted),
                    fitted
                ),
                value,
                1e-15,
                "large roll screen/engineering transform round trip"
            );
        }

        const GraphValueRange zoomedIn = scaleGraphRange(fitted, 0.8);
        const GraphValueRange zoomedOut = scaleGraphRange(fitted, 1.25);
        require(
            zoomedIn.magnitude() < fitted.magnitude()
                && zoomedOut.magnitude() > fitted.magnitude(),
            "roll Y controls remain presentation-only scaling operations"
        );

        const GraphValueRange unchanged = expandGraphRangeToInclude(
            zoomedIn,
            12.0 * radiansPerDegree
        );
        requireNear(unchanged.minimum, zoomedIn.minimum, 0.0,
            "in-range authoring does not undo Y In");
        requireNear(unchanged.maximum, zoomedIn.maximum, 0.0,
            "in-range authoring preserves the selected range");

        const GraphValueRange expanded = expandGraphRangeToInclude(
            zoomedIn,
            60.0 * radiansPerDegree
        );
        require(expanded.maximum > 60.0 * radiansPerDegree,
            "an out-of-view Roll edit expands instead of clamps");
    }

    [[nodiscard]] quantum::coaster::ChannelProfile makeSemanticProfile()
    {
        using quantum::coaster::ChannelProfile;
        using quantum::coaster::ProfileSegment;
        using quantum::math::TransitionType;

        ChannelProfile profile;
        profile.segments = {
            ProfileSegment{1, {0.0, 10.0, 0.0, 1.0,
                TransitionType::Linear}},
            ProfileSegment{2, {10.0, 25.0, 1.0, -0.5,
                TransitionType::CosineEaseInOut}},
            ProfileSegment{3, {25.0, 40.0, -0.5, 0.0,
                TransitionType::Smoothstep}}
        };
        profile.nextSegmentId = 4;
        return profile;
    }

    void semanticMarkersReflectOnlyAuthoredProfileBoundaries()
    {
        using namespace quantum::editor;

        const quantum::coaster::ChannelProfile profile =
            makeSemanticProfile();
        const std::vector<SemanticProfileMarker> markers =
            extractSemanticProfileMarkers(profile);

        require(markers.size() == profile.segments.size() + 1,
            "an N-piece analytic profile exposes exactly N+1 markers");
        require(markers.front().segmentId == 1
                && markers.front().endpoint == ScalarProfileEndpoint::Begin
                && markers.front().regionBoundary,
            "the first marker is the first authored Begin endpoint");
        require(markers[1].segmentId == 1
                && markers[1].endpoint == ScalarProfileEndpoint::End
                && !markers[1].regionBoundary,
            "an interior shared boundary is owned by the left segment End");
        require(markers.back().segmentId == 3
                && markers.back().endpoint == ScalarProfileEndpoint::End
                && markers.back().regionBoundary,
            "the final marker is the last authored End endpoint");

        for (const SemanticProfileMarker& marker : markers)
        {
            requireNear(
                quantum::coaster::evaluateChannelProfile(
                    profile,
                    marker.distance
                ),
                marker.value,
                1e-15,
                "every semantic marker lies exactly on its analytic curve"
            );
        }

        requireNear(
            quantum::math::evaluateScalarTransition(
                profile.segments[0].transition,
                5.0
            ),
            0.5,
            1e-15,
            "the Linear piece remains analytically linear"
        );
        requireNear(
            quantum::math::evaluateScalarTransition(
                profile.segments[1].transition,
                17.5
            ),
            0.25,
            1e-15,
            "the Cosine Ease piece remains analytically deterministic"
        );
        require(markers.size() == 4,
            "analytic samples never become editable intermediate markers");
    }

    void markerTransformsPreserveFlatAndDistanceDomainSemantics()
    {
        using namespace quantum::editor;

        const auto section = quantum::coaster::createRateProfileSection(40.0);
        const auto& flatProfile = section.rateProfileRegion().rateProfiles.pitch;
        const std::vector<SemanticProfileMarker> flatMarkers =
            extractSemanticProfileMarkers(flatProfile);
        require(flatMarkers.size() == 2,
            "a flat one-piece profile still exposes both authored endpoints");
        requireNear(flatMarkers.front().distance, 0.0, 0.0,
            "flat profile start marker remains at region start");
        requireNear(flatMarkers.back().distance, 40.0, 0.0,
            "flat profile end marker remains at region end");

        const GraphValueRange pitchRange{
            -2.0 * radiansPerDegree,
            2.0 * radiansPerDegree
        };
        requireNear(
            graphValueToNormalized(flatMarkers.front().value, pitchRange),
            graphValueToNormalized(flatMarkers.back().value, pitchRange),
            0.0,
            "coincident flat marker values keep exact screen Y coincidence"
        );
        requireNear(graphDistanceToNormalized(10.0, 0.0, 40.0),
            0.25, 0.0, "boundary distance maps to the shared X domain");
        requireNear(normalizedToGraphDistance(0.625, 0.0, 40.0),
            25.0, 0.0, "shared X-domain transform round trips");
    }

    void graphSegmentSamplingPreservesExactDomainEndpoints()
    {
        using namespace quantum::editor;

        const quantum::math::ScalarTransition transition{
            0.3,
            0.9,
            1.0,
            2.0,
            quantum::math::TransitionType::Linear
        };
        const double finalSample = normalizedToGraphDistance(
            1.0,
            transition.domainBegin,
            transition.domainEnd
        );

        requireNear(
            quantum::math::evaluateScalarTransition(
                transition,
                finalSample
            ),
            transition.valueEnd,
            0.0,
            "the final graph sample remains a valid inclusive-domain query"
        );
        require(finalSample == transition.domainEnd,
            "a graph segment's final sample keeps the exact authored end");
    }

    void deterministicSemanticMarkerHitTesting()
    {
        using namespace quantum::editor;

        const std::array<MarkerHitCandidate, 3> candidates{{
            {{RateChannel::Roll, 7, ScalarProfileEndpoint::End}, 1.0},
            {{RateChannel::Pitch, 2, ScalarProfileEndpoint::End}, 16.0},
            {{RateChannel::Yaw, 4, ScalarProfileEndpoint::Begin}, 4.0}
        }};
        require(chooseMarkerHit(
                candidates, 5.0, RateChannel::Pitch, std::nullopt)
                == GraphMarkerId{
                    RateChannel::Pitch, 2, ScalarProfileEndpoint::End},
            "an eligible active-channel marker wins before nearer channels");

        const std::array<MarkerHitCandidate, 3> activeOutside{{
            {{RateChannel::Roll, 7, ScalarProfileEndpoint::End}, 1.0},
            {{RateChannel::Pitch, 2, ScalarProfileEndpoint::End}, 100.0},
            {{RateChannel::Yaw, 4, ScalarProfileEndpoint::Begin}, 4.0}
        }};
        const GraphMarkerId previous{
            RateChannel::Yaw, 4, ScalarProfileEndpoint::Begin};
        require(chooseMarkerHit(
                activeOutside, 5.0, RateChannel::Pitch, previous)
                == previous,
            "the exact previous semantic hover wins when active is absent");

        const std::array<MarkerHitCandidate, 2> coincident{{
            {{RateChannel::Pitch, 8, ScalarProfileEndpoint::End}, 0.0},
            {{RateChannel::Yaw, 3, ScalarProfileEndpoint::End}, 0.0}
        }};
        require(chooseMarkerHit(
                coincident, 10.0, RateChannel::Yaw, std::nullopt)
                == GraphMarkerId{
                    RateChannel::Yaw, 3, ScalarProfileEndpoint::End},
            "active Yaw deterministically owns a coincident Pitch/Yaw marker");

        const std::array<MarkerHitCandidate, 2> stableTie{{
            {{RateChannel::Yaw, 9, ScalarProfileEndpoint::End}, 1.0},
            {{RateChannel::Roll, 12, ScalarProfileEndpoint::Begin}, 1.0}
        }};
        require(chooseMarkerHit(
                stableTie, 10.0, RateChannel::Pitch, std::nullopt)
                == GraphMarkerId{
                    RateChannel::Roll, 12, ScalarProfileEndpoint::Begin},
            "semantic identity, not candidate or render order, breaks ties");
    }

    void markerAndBoundaryDragProposalsAreEngineeringSpaceEdits()
    {
        using namespace quantum::editor;

        requireNear(
            proposeMarkerValueDrag(1.0, -10.0, 0.02, 1.0),
            1.2,
            1e-15,
            "upward marker motion increases the engineering value"
        );
        requireNear(
            proposeMarkerValueDrag(1.0, -10.0, 0.02, 0.25),
            1.05,
            1e-15,
            "fine gain reduces the same vertical marker motion"
        );
        requireNear(
            proposeMarkerValueDrag(1.0, -11.0, 0.02, 1.0, 0.1),
            1.2,
            1e-15,
            "vertical marker proposal uses deterministic snapping"
        );

        const quantum::coaster::ChannelProfile profile =
            makeSemanticProfile();
        const auto firstJointBounds = profileBoundaryMoveBounds(
            profile,
            1,
            ScalarProfileEndpoint::End
        );
        require(firstJointBounds.has_value(),
            "the first interior joint has horizontal move bounds");
        requireNear(firstJointBounds->minimum, 0.0, 0.0,
            "boundary lower bound is the left segment start");
        requireNear(firstJointBounds->maximum, 25.0, 0.0,
            "boundary upper bound is the following outer joint");
        require(!profileBoundaryMoveBounds(
                profile, 1, ScalarProfileEndpoint::Begin).has_value(),
            "the region start is pinned horizontally");
        require(!profileBoundaryMoveBounds(
                profile, 3, ScalarProfileEndpoint::End).has_value(),
            "the region end is pinned horizontally");

        requireNear(
            proposeBoundaryDistanceDrag(
                10.0,
                20.0,
                0.5,
                0.25,
                *firstJointBounds,
                5.0
            ),
            15.0,
            1e-15,
            "horizontal boundary proposal preserves fine gain and snapping"
        );
        require(
            proposeBoundaryDistanceDrag(
                10.0,
                1000.0,
                0.5,
                1.0,
                *firstJointBounds
            ) < firstJointBounds->maximum,
            "boundary proposal remains strictly inside Core move bounds"
        );
    }

    void markerIdentityAndNumericEditingShareTheAuthoritativeProfile()
    {
        using namespace quantum::editor;

        quantum::coaster::ChannelProfile profile = makeSemanticProfile();
        const SemanticProfileMarker selectedMarker =
            extractSemanticProfileMarkers(profile)[1];
        quantum::coaster::setChannelSegmentValue(
            profile,
            selectedMarker.segmentId,
            quantum::coaster::ProfileBoundary::End,
            0.75
        );

        const std::vector<SemanticProfileMarker> refreshed =
            extractSemanticProfileMarkers(profile);
        requireNear(refreshed[1].value, 0.75, 0.0,
            "numeric endpoint edit moves the same semantic marker");
        requireNear(profile.segments[1].transition.valueBegin, 0.75, 0.0,
            "the authoritative Core edit preserves shared C0 continuity");
    }

    void deterministicIntersectionHitTesting()
    {
        using namespace quantum::editor;

        const std::array<CurveHitCandidate, 3> crossing{{
            {RateChannel::Roll, 4.0},
            {RateChannel::Pitch, 9.0},
            {RateChannel::Yaw, 1.0}
        }};

        require(chooseCurveHit(
                crossing, 5.0, RateChannel::Pitch, std::nullopt)
                == RateChannel::Pitch,
            "active curve wins within the hit radius");
        require(chooseCurveHit(
                crossing, 5.0, RateChannel::Pitch, RateChannel::Roll)
                == RateChannel::Pitch,
            "active priority precedes previous hover");

        const std::array<CurveHitCandidate, 3> activeOutside{{
            {RateChannel::Roll, 4.0},
            {RateChannel::Pitch, 100.0},
            {RateChannel::Yaw, 1.0}
        }};
        require(chooseCurveHit(
                activeOutside, 5.0, RateChannel::Pitch, RateChannel::Roll)
                == RateChannel::Roll,
            "previous hover wins when the active curve is not eligible");
        require(chooseCurveHit(
                activeOutside, 5.0, RateChannel::Pitch, std::nullopt)
                == RateChannel::Yaw,
            "nearest eligible curve wins without priority state");

        const std::array<CurveHitCandidate, 2> exactTie{{
            {RateChannel::Yaw, 1.0},
            {RateChannel::Roll, 1.0}
        }};
        require(chooseCurveHit(
                exactTie, 5.0, RateChannel::Pitch, std::nullopt)
                == RateChannel::Roll,
            "stable channel order breaks exact ties, not render order");

        const std::array<CurveHitCandidate, 2> coincidentZero{{
            {RateChannel::Pitch, 0.0},
            {RateChannel::Yaw, 0.0}
        }};
        require(chooseCurveHit(
                coincidentZero, 5.0, RateChannel::Pitch, std::nullopt)
                == RateChannel::Pitch,
            "active Pitch owns a coincident zero Pitch/Yaw hit");
        require(chooseCurveHit(
                coincidentZero, 5.0, RateChannel::Yaw, std::nullopt)
                == RateChannel::Yaw,
            "active Yaw owns a coincident zero Pitch/Yaw hit");

        requireNear(squaredDistanceToLineSegment(
            5.0, 2.0, 0.0, 0.0, 10.0, 0.0), 4.0, 1e-15,
            "screen-space distance uses the nearest point on a curve span");
    }

    void viewOnlyGraphOperationsDoNotMutateAuthoredData()
    {
        using namespace quantum::editor;

        quantum::coaster::AuthoredTrack track;
        track.appendSection();
        const std::string before = quantum::coaster::
            serializeCoasterDocument(track);

        const std::array<double, 2> values{0.0, 36.0 * radiansPerDegree};
        GraphValueRange range = fitSymmetricGraphRange(
            values,
            radiansPerDegree
        );
        range = scaleGraphRange(range, 0.8);
        range = scaleGraphRange(range, 1.25);
        RateChannel activeChannel = RateChannel::Pitch;
        activeChannel = RateChannel::Yaw;
        const std::array<CurveHitCandidate, 2> candidates{{
            {RateChannel::Pitch, 0.0},
            {RateChannel::Yaw, 0.0}
        }};
        static_cast<void>(chooseCurveHit(
            candidates,
            5.0,
            activeChannel,
            std::nullopt
        ));
        const std::vector<SemanticProfileMarker> markers =
            extractSemanticProfileMarkers(
                track.section(0).rateProfileRegion().rateProfiles.pitch
            );
        const std::array<MarkerHitCandidate, 1> markerCandidates{{
            {{RateChannel::Pitch,
                markers.front().segmentId,
                markers.front().endpoint},
             0.0}
        }};
        static_cast<void>(chooseMarkerHit(
            markerCandidates,
            10.0,
            RateChannel::Pitch,
            std::nullopt
        ));

        require(range.valid(), "view-only graph range remains valid");
        require(
            quantum::coaster::serializeCoasterDocument(track) == before,
            "active selection, range changes, and hit testing do not mutate "
            "authored data"
        );
    }
}

int main()
{
    const std::pair<std::string_view, void(*)()> tests[] = {
        {"conversion relationships", conversionRelationships},
        {"resultant curvature", resultantCurvatureUsesOrthogonalPitchYawComponents},
        {"independent graph transforms", independentGraphTransformsShareOneCanvas},
        {"large roll graph range", largeRollRatesRemainRepresentableInTheGraph},
        {"semantic marker extraction", semanticMarkersReflectOnlyAuthoredProfileBoundaries},
        {"semantic marker transforms", markerTransformsPreserveFlatAndDistanceDomainSemantics},
        {"exact graph sample endpoints", graphSegmentSamplingPreservesExactDomainEndpoints},
        {"semantic marker hit testing", deterministicSemanticMarkerHitTesting},
        {"marker drag proposals", markerAndBoundaryDragProposalsAreEngineeringSpaceEdits},
        {"marker numeric synchronization", markerIdentityAndNumericEditingShareTheAuthoritativeProfile},
        {"deterministic intersection hit testing", deterministicIntersectionHitTesting},
        {"view-only graph actions", viewOnlyGraphOperationsDoNotMutateAuthoredData}
    };

    for (const auto& [name, test] : tests)
    {
        try
        {
            test();
            std::cout << "[PASS] " << name << '\n';
        }
        catch (const std::exception& exception)
        {
            std::cerr << "[FAIL] " << name << ": "
                << exception.what() << '\n';
            return 1;
        }
    }

    std::cout << "All Transition Editor model tests passed.\n";
    return 0;
}
