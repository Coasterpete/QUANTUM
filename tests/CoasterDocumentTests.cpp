#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/GeometricSection.hpp>
#include <quantum/coaster/PlanarArcRegion.hpp>
#include <quantum/math/TransitionFunctions.hpp>

#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
    using quantum::coaster::AuthoredTrack;
    using quantum::coaster::AuthoredTrackSection;
    using quantum::coaster::ChannelProfile;
    using quantum::coaster::splitChannelSegment;
    using quantum::coaster::currentFormatVersion;
    using quantum::coaster::defaultNewSectionLength;
    using quantum::coaster::deserializeCoasterDocument;
    using quantum::coaster::GeometryRegion;
    using quantum::coaster::PlanarArcRegion;
    using quantum::coaster::ProfileSegment;
    using quantum::coaster::RegionKind;
    using quantum::coaster::serializeCoasterDocument;
    using quantum::coaster::SegmentId;
    using quantum::math::ScalarTransition;
    using quantum::math::TransitionType;

    class TestFailure final : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    void require(
        const bool condition,
        const std::string_view message)
    {
        if (!condition)
        {
            throw TestFailure(std::string(message));
        }
    }

    template<typename T>
    void requireValidDocument(
        const std::expected<T, std::string>& result,
        const std::string_view context)
    {
        if (!result.has_value())
        {
            throw TestFailure(
                std::string(context) + ": "
                + result.error());
        }
    }

    void requireContains(
        const std::string& haystack,
        const std::string& needle,
        const std::string_view context)
    {
        if (haystack.find(needle) == std::string::npos)
        {
            throw TestFailure(
                std::string(context) + ": expected error to contain '"
                + needle + "', got: " + haystack);
        }
    }

    [[nodiscard]] ChannelProfile singleSegmentChannel(
        const double length,
        const double valueBegin,
        const double valueEnd,
        const TransitionType type)
    {
        ChannelProfile profile;
        profile.segments.push_back(ProfileSegment{
            profile.nextSegmentId,
            ScalarTransition{0.0, length, valueBegin, valueEnd, type}
        });
        ++profile.nextSegmentId;
        return profile;
    }

    [[nodiscard]] AuthoredTrackSection rateProfileSection(
        const double length,
        const double rollEnd,
        const double pitchBegin,
        const double pitchEnd,
        const double yawEnd,
        const TransitionType rollType = TransitionType::Smootherstep,
        const TransitionType pitchType = TransitionType::CosineEaseInOut,
        const TransitionType yawType = TransitionType::Smoothstep)
    {
        AuthoredTrackSection section;
        section.length = length;
        section.rateProfileRegion().rateProfiles.roll =
            singleSegmentChannel(length, 0.0, rollEnd, rollType);
        section.rateProfileRegion().rateProfiles.pitch =
            singleSegmentChannel(length, pitchBegin, pitchEnd, pitchType);
        section.rateProfileRegion().rateProfiles.yaw =
            singleSegmentChannel(length, 0.0, yawEnd, yawType);
        return section;
    }

    [[nodiscard]] AuthoredTrackSection planarArcSection(
        const double radius,
        const double sweptAngle,
        const double planeTilt = 0.0,
        const double bankChange = 0.0)
    {
        AuthoredTrackSection section;
        section.kind = RegionKind::Geometry;
        section.length = std::abs(sweptAngle) * radius;

        PlanarArcRegion arc;
        arc.radius = radius;
        arc.sweptAngle = sweptAngle;
        arc.planeTilt = planeTilt;
        arc.bankChange = bankChange;

        section.region = GeometryRegion{arc};
        return section;
    }

    // ================================================================
    // 1. Default authored track round-trip
    // ================================================================

    void defaultTrackRoundTrip()
    {
        const AuthoredTrack original =
            quantum::coaster::createDefaultAuthoredTrack();

        const std::string json = serializeCoasterDocument(original);
        require(!json.empty(), "Serialized JSON must not be empty");

        auto result = deserializeCoasterDocument(json);
        requireValidDocument(result, "default track round-trip");

        const AuthoredTrack& restored = *result;
        require(
            restored.sectionCount() == original.sectionCount(),
            "Section count mismatch");

        const std::string json2 = serializeCoasterDocument(restored);
        require(
            json == json2,
            "Deterministic round-trip: serialize -> deserialize -> "
            "serialize must produce identical JSON");
    }

    // ================================================================
    // 2. Mixed Rate/Profile + Planar Arc ordering
    // ================================================================

    void mixedOrdering()
    {
        AuthoredTrack track;
        track.appendSection();
        track.section(0) = rateProfileSection(
            90.0, 0.01, -0.005, 0.015, 0.008);

        track.insertSectionAfter(0,
            planarArcSection(30.0, 2.0, 0.5, 0.3));

        track.insertSectionAfter(1,
            rateProfileSection(
                45.0, 0.02, 0.0, 0.01, -0.003,
                TransitionType::QuinticEaseInOut,
                TransitionType::CubicEaseIn,
                TransitionType::Linear));

        track.insertSectionAfter(2,
            planarArcSection(50.0, -1.5, 0.0, 0.8));

        require(
            track.sectionCount() == 4,
            "Expected 4 sections");

        const std::string json1 = serializeCoasterDocument(track);
        auto result = deserializeCoasterDocument(json1);
        requireValidDocument(result, "deserialization");

        const std::string json2 = serializeCoasterDocument(*result);
        require(
            json1 == json2,
            "Mixed-order round-trip must be deterministic");

        require(
            result->sectionCount() == 4,
            "Restored section count mismatch");

        require(
            result->section(0).kind == RegionKind::RateProfiles,
            "Section 0 kind");
        require(
            result->section(1).kind == RegionKind::Geometry,
            "Section 1 kind");
        require(
            result->section(2).kind == RegionKind::RateProfiles,
            "Section 2 kind");
        require(
            result->section(3).kind == RegionKind::Geometry,
            "Section 3 kind");
    }

    // ================================================================
    // 3. Multi-segment Roll/Pitch/Yaw profiles
    // ================================================================

    void multiSegmentProfiles()
    {
        AuthoredTrackSection section;
        section.length = 120.0;

        // Roll: 3 segments
        ChannelProfile roll;
        roll.segments.push_back(ProfileSegment{
            roll.nextSegmentId,
            ScalarTransition{0.0, 40.0, 0.0, 0.01, TransitionType::Linear}
        });
        ++roll.nextSegmentId;
        roll.segments.push_back(ProfileSegment{
            roll.nextSegmentId,
            ScalarTransition{40.0, 80.0, 0.01, 0.03, TransitionType::Smoothstep}
        });
        ++roll.nextSegmentId;
        roll.segments.push_back(ProfileSegment{
            roll.nextSegmentId,
            ScalarTransition{80.0, 120.0, 0.03, 0.0, TransitionType::Linear}
        });
        ++roll.nextSegmentId;

        // Pitch: 2 segments
        ChannelProfile pitch;
        pitch.segments.push_back(ProfileSegment{
            pitch.nextSegmentId,
            ScalarTransition{0.0, 60.0, 0.02, -0.01, TransitionType::CosineEaseInOut}
        });
        ++pitch.nextSegmentId;
        pitch.segments.push_back(ProfileSegment{
            pitch.nextSegmentId,
            ScalarTransition{60.0, 120.0, -0.01, 0.005, TransitionType::QuinticEaseOut}
        });
        ++pitch.nextSegmentId;

        // Yaw: 1 segment
        ChannelProfile yaw;
        yaw.segments.push_back(ProfileSegment{
            yaw.nextSegmentId,
            ScalarTransition{0.0, 120.0, 0.003, 0.015, TransitionType::SineEaseIn}
        });
        ++yaw.nextSegmentId;

        section.rateProfileRegion().rateProfiles.roll = std::move(roll);
        section.rateProfileRegion().rateProfiles.pitch = std::move(pitch);
        section.rateProfileRegion().rateProfiles.yaw = std::move(yaw);

        AuthoredTrack track;
        track.appendSection();
        track.section(0) = std::move(section);

        const std::string json1 = serializeCoasterDocument(track);
        auto result = deserializeCoasterDocument(json1);
        requireValidDocument(result, "deserialization");

        const std::string json2 = serializeCoasterDocument(*result);
        require(json1 == json2, "Multi-segment round-trip deterministic");

        const AuthoredTrackSection& rs = result->section(0);
        require(
            rs.rateProfileRegion().rateProfiles.roll.segments.size() == 3,
            "Roll segment count");
        require(
            rs.rateProfileRegion().rateProfiles.pitch.segments.size() == 2,
            "Pitch segment count");
        require(
            rs.rateProfileRegion().rateProfiles.yaw.segments.size() == 1,
            "Yaw segment count");
    }

    // ================================================================
    // 4. All TransitionType values round-trip
    // ================================================================

    void allTransitionTypes()
    {
        const TransitionType types[] = {
            TransitionType::Linear,
            TransitionType::Smoothstep,
            TransitionType::Smootherstep,
            TransitionType::SeventhOrderSmoothstep,
            TransitionType::CosineEaseInOut,
            TransitionType::SineEaseIn,
            TransitionType::SineEaseOut,
            TransitionType::QuadraticEaseIn,
            TransitionType::QuadraticEaseOut,
            TransitionType::QuadraticEaseInOut,
            TransitionType::CubicEaseIn,
            TransitionType::CubicEaseOut,
            TransitionType::CubicEaseInOut,
            TransitionType::QuarticEaseIn,
            TransitionType::QuarticEaseOut,
            TransitionType::QuarticEaseInOut,
            TransitionType::QuinticEaseIn,
            TransitionType::QuinticEaseOut,
            TransitionType::QuinticEaseInOut,
        };

        AuthoredTrack track;

        for (std::size_t i = 0; i < std::size(types); ++i)
        {
            const double len = 60.0 + static_cast<double>(i) * 10.0;

            AuthoredTrackSection section;
            section.length = len;
            section.rateProfileRegion().rateProfiles.roll =
                singleSegmentChannel(len, 0.0, 0.01, types[i]);
            section.rateProfileRegion().rateProfiles.pitch =
                singleSegmentChannel(len, 0.0, 0.0, TransitionType::Linear);
            section.rateProfileRegion().rateProfiles.yaw =
                singleSegmentChannel(len, 0.0, 0.0, TransitionType::Linear);

            if (track.sectionCount() == 0)
            {
                track.appendSection();
                track.section(0) = std::move(section);
            }
            else
            {
                track.insertSectionAfter(
                    track.sectionCount() - 1, section);
            }
        }

        require(
            track.sectionCount() == std::size(types),
            "Section count for all transition types");

        const std::string json1 = serializeCoasterDocument(track);
        auto result = deserializeCoasterDocument(json1);
        requireValidDocument(result, "deserialization");

        const std::string json2 = serializeCoasterDocument(*result);
        require(
            json1 == json2,
            "All-transition-type round-trip deterministic");
    }

    // ================================================================
    // 5. Planar Arc parameters round-trip
    // ================================================================

    void planarArcRoundTrip()
    {
        AuthoredTrack track;
        track.appendSection();
        track.section(0) = planarArcSection(
            40.0, 2.5, 0.75, 1.2);

        const std::string json1 = serializeCoasterDocument(track);
        auto result = deserializeCoasterDocument(json1);
        requireValidDocument(result, "deserialization");

        require(
            result->sectionCount() == 1,
            "Planar arc section count");

        const AuthoredTrackSection& rs = result->section(0);
        require(rs.kind == RegionKind::Geometry, "Geometry kind");

        const PlanarArcRegion& arc = std::get<PlanarArcRegion>(
            std::get<GeometryRegion>(rs.region).construction);

        const double tolerance = 1e-12;
        require(
            std::abs(arc.radius - 40.0) < tolerance,
            "Radius round-trip");
        require(
            std::abs(arc.sweptAngle - 2.5) < tolerance,
            "SweptAngle round-trip");
        require(
            std::abs(arc.planeTilt - 0.75) < tolerance,
            "PlaneTilt round-trip");
        require(
            std::abs(arc.bankChange - 1.2) < tolerance,
            "BankChange round-trip");
    }

    // ================================================================
    // 6. SegmentId preservation
    // ================================================================

    void segmentIdPreservation()
    {
        AuthoredTrackSection section;
        section.length = 100.0;

        ChannelProfile roll;
        roll.segments.push_back(ProfileSegment{
            42,
            ScalarTransition{0.0, 50.0, 0.0, 0.01, TransitionType::Linear}
        });
        roll.segments.push_back(ProfileSegment{
            99,
            ScalarTransition{50.0, 100.0, 0.01, 0.02, TransitionType::Smoothstep}
        });
        roll.nextSegmentId = 100;

        section.rateProfileRegion().rateProfiles.roll = std::move(roll);
        section.rateProfileRegion().rateProfiles.pitch =
            singleSegmentChannel(100.0, 0.0, 0.0, TransitionType::Linear);
        section.rateProfileRegion().rateProfiles.yaw =
            singleSegmentChannel(100.0, 0.0, 0.0, TransitionType::Linear);

        AuthoredTrack track;
        track.appendSection();
        track.section(0) = std::move(section);

        const std::string json1 = serializeCoasterDocument(track);
        auto result = deserializeCoasterDocument(json1);
        requireValidDocument(result, "deserialization");

        const ChannelProfile& restoredRoll =
            result->section(0).rateProfileRegion().rateProfiles.roll;

        require(
            restoredRoll.segments.size() == 2,
            "Roll segment count");
        require(
            restoredRoll.segments[0].id == 42,
            "First segment id preserved");
        require(
            restoredRoll.segments[1].id == 99,
            "Second segment id preserved");
    }

    // ================================================================
    // 7. nextSegmentId preservation
    // ================================================================

    void nextSegmentIdPreservation()
    {
        AuthoredTrackSection section;
        section.length = 80.0;

        ChannelProfile roll;
        roll.segments.push_back(ProfileSegment{
            5,
            ScalarTransition{0.0, 80.0, 0.0, 0.01, TransitionType::Linear}
        });
        roll.nextSegmentId = 17;

        section.rateProfileRegion().rateProfiles.roll = std::move(roll);
        section.rateProfileRegion().rateProfiles.pitch =
            singleSegmentChannel(80.0, 0.0, 0.0, TransitionType::Linear);
        section.rateProfileRegion().rateProfiles.yaw =
            singleSegmentChannel(80.0, 0.0, 0.0, TransitionType::Linear);

        AuthoredTrack track;
        track.appendSection();
        track.section(0) = std::move(section);

        const std::string json1 = serializeCoasterDocument(track);
        auto result = deserializeCoasterDocument(json1);
        requireValidDocument(result, "deserialization");

        const ChannelProfile& restoredRoll =
            result->section(0).rateProfileRegion().rateProfiles.roll;

        require(
            restoredRoll.nextSegmentId == 17,
            "nextSegmentId preserved through round-trip");
    }

    // ================================================================
    // 8. Split after load uses expected next ID
    // ================================================================

    void splitAfterLoad()
    {
        AuthoredTrackSection section;
        section.length = 100.0;

        ChannelProfile roll;
        roll.segments.push_back(ProfileSegment{
            1,
            ScalarTransition{0.0, 100.0, 0.0, 0.02, TransitionType::Linear}
        });
        roll.nextSegmentId = 2;

        section.rateProfileRegion().rateProfiles.roll = std::move(roll);
        section.rateProfileRegion().rateProfiles.pitch =
            singleSegmentChannel(100.0, 0.0, 0.0, TransitionType::Linear);
        section.rateProfileRegion().rateProfiles.yaw =
            singleSegmentChannel(100.0, 0.0, 0.0, TransitionType::Linear);

        AuthoredTrack track;
        track.appendSection();
        track.section(0) = std::move(section);

        const std::string json = serializeCoasterDocument(track);
        auto result = deserializeCoasterDocument(json);
        requireValidDocument(result, "deserialization");

        // Split the loaded roll channel at distance 40.
        ChannelProfile& loadedRoll =
            result->section(0).rateProfileRegion().rateProfiles.roll;

        const SegmentId newId = splitChannelSegment(
            loadedRoll, 1, 40.0);

        // The new segment should have received id == 2
        // (the value of nextSegmentId before the split).
        require(
            newId == 2,
            "Split after load produces expected next segment id");

        // nextSegmentId should now be 3.
        require(
            loadedRoll.nextSegmentId == 3,
            "nextSegmentId incremented after split");
    }

    // ================================================================
    // 9. Deterministic serialization
    // ================================================================

    void deterministicSerialization()
    {
        const AuthoredTrack track =
            quantum::coaster::createDefaultAuthoredTrack();

        const std::string json1 = serializeCoasterDocument(track);
        const std::string json2 = serializeCoasterDocument(track);
        const std::string json3 = serializeCoasterDocument(track);

        require(
            json1 == json2,
            "First two serializations must be identical");
        require(
            json2 == json3,
            "Second and third serializations must be identical");
    }

    // ================================================================
    // 10. serialize -> deserialize -> serialize identical JSON
    // ================================================================

    void serializeDeserializeSerialize()
    {
        AuthoredTrack track;
        track.appendSection();
        track.section(0) = rateProfileSection(
            150.0, 0.02, 0.01, -0.005, 0.008);

        track.insertSectionAfter(0,
            planarArcSection(25.0, 3.141592653589793, 0.3, 0.5));

        const std::string json1 = serializeCoasterDocument(track);

        auto result = deserializeCoasterDocument(json1);
        requireValidDocument(result, "deserialization");

        const std::string json2 = serializeCoasterDocument(*result);

        require(
            json1 == json2,
            "serialize -> deserialize -> serialize must produce "
            "identical JSON");
    }

    // ================================================================
    // 11. Malformed JSON rejection
    // ================================================================

    void malformedJsonRejection()
    {
        auto result = deserializeCoasterDocument("{{{{not json}}}}");
        require(!result.has_value(), "Malformed JSON must be rejected");
        requireContains(
            result.error(),
            "parse error",
            "Malformed JSON error message");
    }

    // ================================================================
    // 12. Missing formatVersion rejection
    // ================================================================

    void missingFormatVersionRejection()
    {
        auto result = deserializeCoasterDocument(
            R"({"sections": [{"kind": "RateProfiles", "length": 60.0, "rateProfiles": {"roll": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}, "pitch": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}, "yaw": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}}}]})");
        require(
            !result.has_value(),
            "Missing formatVersion must be rejected");
        requireContains(
            result.error(),
            "formatVersion",
            "Missing formatVersion error");
    }

    // ================================================================
    // 13. Unsupported version rejection
    // ================================================================

    void unsupportedVersionRejection()
    {
        auto result = deserializeCoasterDocument(
            R"({"formatVersion": 99, "sections": []})");
        require(
            !result.has_value(),
            "Unsupported version must be rejected");
        requireContains(
            result.error(),
            "99",
            "Unsupported version error mentions the version");
    }

    // ================================================================
    // 14. Missing required field rejection
    // ================================================================

    void missingRequiredFieldRejection()
    {
        auto result = deserializeCoasterDocument(
            R"({"formatVersion": 1})");
        require(
            !result.has_value(),
            "Missing sections must be rejected");
        requireContains(
            result.error(),
            "sections",
            "Missing sections error");
    }

    // ================================================================
    // 15. Wrong JSON type rejection
    // ================================================================

    void wrongJsonTypeRejection()
    {
        auto result = deserializeCoasterDocument(
            R"({"formatVersion": "one", "sections": []})");
        require(
            !result.has_value(),
            "Wrong type for formatVersion must be rejected");
        requireContains(
            result.error(),
            "formatVersion",
            "Wrong type error mentions the field");
    }

    // ================================================================
    // 16. Unknown Region kind rejection
    // ================================================================

    void unknownRegionKindRejection()
    {
        auto result = deserializeCoasterDocument(
            R"({"formatVersion": 1, "sections": [{"kind": "Helix", "length": 60.0}]})");
        require(
            !result.has_value(),
            "Unknown region kind must be rejected");
        requireContains(
            result.error(),
            "Helix",
            "Unknown kind error mentions the bad kind");
    }

    // ================================================================
    // 17. Unknown TransitionType rejection
    // ================================================================

    void unknownTransitionTypeRejection()
    {
        auto result = deserializeCoasterDocument(
            R"({"formatVersion": 1, "sections": [{"kind": "RateProfiles", "length": 60.0, "rateProfiles": {"roll": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "BogusTransition"}}]}, "pitch": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}, "yaw": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}}}]})");
        require(
            !result.has_value(),
            "Unknown TransitionType must be rejected");
        requireContains(
            result.error(),
            "BogusTransition",
            "Unknown type error mentions the bad type");
    }

    // ================================================================
    // 18. Unknown version-1 field rejection
    // ================================================================

    void unknownFieldRejection()
    {
        // Unknown field at root level.
        auto r1 = deserializeCoasterDocument(
            R"({"formatVersion": 1, "sections": [{"kind": "RateProfiles", "length": 60.0, "rateProfiles": {"roll": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}, "pitch": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}, "yaw": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}}}], "author": "test"})");
        require(!r1.has_value(), "Unknown root field must be rejected");
        requireContains(
            r1.error(),
            "author",
            "Unknown root field error");

        // Unknown field at section level.
        auto r2 = deserializeCoasterDocument(
            R"({"formatVersion": 1, "sections": [{"kind": "RateProfiles", "length": 60.0, "color": "red", "rateProfiles": {"roll": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}, "pitch": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}, "yaw": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}}}]})");
        require(
            !r2.has_value(),
            "Unknown section field must be rejected");
        requireContains(
            r2.error(),
            "color",
            "Unknown section field error");

        // Misspelled field (the bankChnage test case).
        auto r3 = deserializeCoasterDocument(
            R"({"formatVersion": 1, "sections": [{"kind": "Geometry", "length": 78.53981633974483, "planarArc": {"radius": 25.0, "sweptAngle": 3.141592653589793, "planeTilt": 0.0, "bankChnage": 0.0}}]})");
        require(
            !r3.has_value(),
            "Misspelled field 'bankChnage' must be rejected");
        requireContains(
            r3.error(),
            "bankChnage",
            "Misspelled field error mentions the typo");

        // Unknown field inside transition.
        auto r4 = deserializeCoasterDocument(
            R"({"formatVersion": 1, "sections": [{"kind": "RateProfiles", "length": 60.0, "rateProfiles": {"roll": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear", "extra": true}}]}, "pitch": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}, "yaw": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}}}]})");
        require(
            !r4.has_value(),
            "Unknown field inside transition must be rejected");
        requireContains(
            r4.error(),
            "extra",
            "Unknown transition field error");
    }

    // ================================================================
    // 19. Invalid channel/domain/C0 data rejection
    // ================================================================

    void invalidChannelDataRejection()
    {
        // C0 discontinuity: adjacent segments have mismatched boundary values.
        auto result = deserializeCoasterDocument(
            R"({"formatVersion": 1, "sections": [{"kind": "RateProfiles", "length": 100.0, "rateProfiles": {"roll": {"nextSegmentId": 3, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 50.0, "valueBegin": 0.0, "valueEnd": 0.01, "type": "Linear"}}, {"id": 2, "transition": {"domainBegin": 50.0, "domainEnd": 100.0, "valueBegin": 0.05, "valueEnd": 0.02, "type": "Linear"}}]}, "pitch": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 100.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}, "yaw": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 100.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}}}]})");
        require(
            !result.has_value(),
            "C0 discontinuity must be rejected");
    }

    // ================================================================
    // 20. Invalid Planar Arc data rejection
    // ================================================================

    void invalidPlanarArcDataRejection()
    {
        // Negative radius.
        auto r1 = deserializeCoasterDocument(
            R"({"formatVersion": 1, "sections": [{"kind": "Geometry", "length": 25.0, "planarArc": {"radius": -10.0, "sweptAngle": 1.0, "planeTilt": 0.0, "bankChange": 0.0}}]})");
        require(
            !r1.has_value(),
            "Negative radius must be rejected");

        // Zero swept angle.
        auto r2 = deserializeCoasterDocument(
            R"({"formatVersion": 1, "sections": [{"kind": "Geometry", "length": 25.0, "planarArc": {"radius": 25.0, "sweptAngle": 0.0, "planeTilt": 0.0, "bankChange": 0.0}}]})");
        require(
            !r2.has_value(),
            "Zero swept angle must be rejected");

        // Length mismatch: |sweptAngle| * radius != length.
        auto r3 = deserializeCoasterDocument(
            R"({"formatVersion": 1, "sections": [{"kind": "Geometry", "length": 100.0, "planarArc": {"radius": 25.0, "sweptAngle": 1.0, "planeTilt": 0.0, "bankChange": 0.0}}]})");
        require(
            !r3.has_value(),
            "Length/sweep/radius mismatch must be rejected");
    }

    // ================================================================
    // 21. Empty sections rejection
    // ================================================================

    void emptySectionsRejection()
    {
        auto result = deserializeCoasterDocument(
            R"({"formatVersion": 1, "sections": []})");
        require(
            !result.has_value(),
            "Empty sections array must be rejected");
        requireContains(
            result.error(),
            "sections",
            "Empty sections error");
    }

    // ----------------------------------------------------------------
    // Test runner
    // ----------------------------------------------------------------

    struct Test
    {
        std::string_view name;
        std::function<void()> function;
    };

    int runTests()
    {
        const std::vector<Test> tests = {
            {"DefaultTrackRoundTrip",           defaultTrackRoundTrip},
            {"MixedOrdering",                   mixedOrdering},
            {"MultiSegmentProfiles",            multiSegmentProfiles},
            {"AllTransitionTypes",              allTransitionTypes},
            {"PlanarArcRoundTrip",              planarArcRoundTrip},
            {"SegmentIdPreservation",           segmentIdPreservation},
            {"NextSegmentIdPreservation",       nextSegmentIdPreservation},
            {"SplitAfterLoad",                  splitAfterLoad},
            {"DeterministicSerialization",      deterministicSerialization},
            {"SerializeDeserializeSerialize",   serializeDeserializeSerialize},
            {"MalformedJsonRejection",          malformedJsonRejection},
            {"MissingFormatVersionRejection",   missingFormatVersionRejection},
            {"UnsupportedVersionRejection",     unsupportedVersionRejection},
            {"MissingRequiredFieldRejection",   missingRequiredFieldRejection},
            {"WrongJsonTypeRejection",          wrongJsonTypeRejection},
            {"UnknownRegionKindRejection",      unknownRegionKindRejection},
            {"UnknownTransitionTypeRejection",  unknownTransitionTypeRejection},
            {"UnknownFieldRejection",           unknownFieldRejection},
            {"InvalidChannelDataRejection",     invalidChannelDataRejection},
            {"InvalidPlanarArcDataRejection",   invalidPlanarArcDataRejection},
            {"EmptySectionsRejection",           emptySectionsRejection},
        };

        std::size_t failures = 0;

        for (const auto& test : tests)
        {
            try
            {
                test.function();
                std::cout << "[PASS] " << test.name << '\n';
            }
            catch (const std::exception& exception)
            {
                ++failures;
                std::cerr << "[FAIL] " << test.name << ": "
                          << exception.what() << '\n';
            }
            catch (...)
            {
                ++failures;
                std::cerr << "[FAIL] " << test.name
                          << ": unknown exception\n";
            }
        }

        if (failures != 0)
        {
            std::cerr << failures << " test group(s) failed.\n";
            return 1;
        }

        std::cout << std::size(tests) << " test groups passed.\n";
        return 0;
    }
}

int main()
{
    return runTests();
}
