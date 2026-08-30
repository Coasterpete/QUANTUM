#include <quantum/coaster/CoasterDocument.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace quantum::coaster
{
    namespace
    {
        using json = nlohmann::json;

        // ----------------------------------------------------------------
        // TransitionType <-> stable string mapping
        // ----------------------------------------------------------------

        struct TransitionTypeMapping
        {
            math::TransitionType type;
            const char* name;
        };

        // Ordered by enum value for clarity; the lookup functions use
        // linear scan, which is fine for 19 entries.
        constexpr std::array<TransitionTypeMapping, 19>
            transitionTypeMap = {{
                {math::TransitionType::Linear,                "Linear"},
                {math::TransitionType::Smoothstep,            "Smoothstep"},
                {math::TransitionType::Smootherstep,          "Smootherstep"},
                {math::TransitionType::SeventhOrderSmoothstep,
                    "SeventhOrderSmoothstep"},
                {math::TransitionType::CosineEaseInOut,       "CosineEaseInOut"},
                {math::TransitionType::SineEaseIn,            "SineEaseIn"},
                {math::TransitionType::SineEaseOut,           "SineEaseOut"},
                {math::TransitionType::QuadraticEaseIn,       "QuadraticEaseIn"},
                {math::TransitionType::QuadraticEaseOut,      "QuadraticEaseOut"},
                {math::TransitionType::QuadraticEaseInOut,    "QuadraticEaseInOut"},
                {math::TransitionType::CubicEaseIn,           "CubicEaseIn"},
                {math::TransitionType::CubicEaseOut,          "CubicEaseOut"},
                {math::TransitionType::CubicEaseInOut,        "CubicEaseInOut"},
                {math::TransitionType::QuarticEaseIn,         "QuarticEaseIn"},
                {math::TransitionType::QuarticEaseOut,        "QuarticEaseOut"},
                {math::TransitionType::QuarticEaseInOut,      "QuarticEaseInOut"},
                {math::TransitionType::QuinticEaseIn,         "QuinticEaseIn"},
                {math::TransitionType::QuinticEaseOut,        "QuinticEaseOut"},
                {math::TransitionType::QuinticEaseInOut,      "QuinticEaseInOut"},
            }};

        [[nodiscard]] const char* transitionTypeToString(
            const math::TransitionType type)
        {
            for (const auto& entry : transitionTypeMap)
            {
                if (entry.type == type)
                {
                    return entry.name;
                }
            }

            // Unreachable when the enum is fully covered; the cast
            // below will catch missing entries at runtime.
            return "Unknown";
        }

        [[nodiscard]] math::TransitionType transitionTypeFromString(
            const std::string& name)
        {
            for (const auto& entry : transitionTypeMap)
            {
                if (name == entry.name)
                {
                    return entry.type;
                }
            }

            throw std::runtime_error(
                "Unknown transition type '" + name + "'");
        }

        // ----------------------------------------------------------------
        // Strict unknown-field detection
        // ----------------------------------------------------------------

        void requireNoUnknownFields(
            const json& object,
            const std::vector<std::string>& allowedKeys,
            const std::string& path)
        {
            for (auto it = object.begin(); it != object.end(); ++it)
            {
                const bool found = std::find(
                    allowedKeys.begin(),
                    allowedKeys.end(),
                    it.key()) != allowedKeys.end();

                if (!found)
                {
                    throw std::runtime_error(
                        path + ": unexpected field '" + it.key() + "'");
                }
            }
        }

        // ----------------------------------------------------------------
        // Type-checking helpers
        // ----------------------------------------------------------------

        void requireObject(
            const json& parent,
            const std::string& key,
            const std::string& path)
        {
            if (!parent.contains(key) || !parent[key].is_object())
            {
                throw std::runtime_error(
                    path + ": missing or non-object '" + key + "'");
            }
        }

        void requireArray(
            const json& parent,
            const std::string& key,
            const std::string& path)
        {
            if (!parent.contains(key) || !parent[key].is_array())
            {
                throw std::runtime_error(
                    path + ": missing or non-array '" + key + "'");
            }
        }

        void requireNumber(
            const json& parent,
            const std::string& key,
            const std::string& path)
        {
            if (!parent.contains(key) || !parent[key].is_number())
            {
                throw std::runtime_error(
                    path + ": missing or non-numeric '" + key + "'");
            }
        }

        void requireString(
            const json& parent,
            const std::string& key,
            const std::string& path)
        {
            if (!parent.contains(key) || !parent[key].is_string())
            {
                throw std::runtime_error(
                    path + ": missing or non-string '" + key + "'");
            }
        }

        void requireInteger(
            const json& parent,
            const std::string& key,
            const std::string& path)
        {
            if (!parent.contains(key) || !parent[key].is_number_integer())
            {
                throw std::runtime_error(
                    path + ": missing or non-integer '" + key + "'");
            }
        }

        // ----------------------------------------------------------------
        // Serialization
        // ----------------------------------------------------------------

        json serializeScalarTransition(
            const math::ScalarTransition& t)
        {
            json result;
            result["domainBegin"] = t.domainBegin;
            result["domainEnd"] = t.domainEnd;
            result["valueBegin"] = t.valueBegin;
            result["valueEnd"] = t.valueEnd;
            result["type"] = transitionTypeToString(t.transitionType);
            return result;
        }

        json serializeProfileSegment(const ProfileSegment& segment)
        {
            json result;
            result["id"] = segment.id;
            result["transition"] = serializeScalarTransition(
                segment.transition);
            return result;
        }

        json serializeChannelProfile(const ChannelProfile& channel)
        {
            json result;
            result["nextSegmentId"] = channel.nextSegmentId;

            json segmentsJson = json::array();
            for (const auto& segment : channel.segments)
            {
                segmentsJson.push_back(serializeProfileSegment(segment));
            }
            result["segments"] = std::move(segmentsJson);

            return result;
        }

        json serializeGeometricSection(const GeometricSection& gs)
        {
            json result;
            result["roll"] = serializeChannelProfile(gs.roll);
            result["pitch"] = serializeChannelProfile(gs.pitch);
            result["yaw"] = serializeChannelProfile(gs.yaw);
            return result;
        }

        json serializePlanarArcRegion(const PlanarArcRegion& arc)
        {
            json result;
            result["radius"] = arc.radius;
            result["sweptAngle"] = arc.sweptAngle;
            result["planeTilt"] = arc.planeTilt;
            result["bankChange"] = arc.bankChange;
            return result;
        }

        json serializeStartPose(const AuthoredStartPose& pose)
        {
            json result;
            result["position"] = {
                {"x", pose.position.x},
                {"y", pose.position.y},
                {"z", pose.position.z}
            };
            result["orientation"] = {
                {"w", pose.orientation.w},
                {"x", pose.orientation.x},
                {"y", pose.orientation.y},
                {"z", pose.orientation.z}
            };
            return result;
        }

        json serializeSection(const AuthoredTrackSection& section)
        {
            json result;
            result["length"] = section.length;

            if (section.kind == RegionKind::RateProfiles)
            {
                result["kind"] = "RateProfiles";
                result["rateProfiles"] = serializeGeometricSection(
                    section.rateProfileRegion().rateProfiles);
            }
            else
            {
                result["kind"] = "Geometry";

                const PlanarArcRegion& arc =
                    std::get<PlanarArcRegion>(
                        std::get<GeometryRegion>(section.region)
                            .construction);

                result["planarArc"] = serializePlanarArcRegion(arc);
            }

            return result;
        }

        // ----------------------------------------------------------------
        // Deserialization
        // ----------------------------------------------------------------

        math::ScalarTransition deserializeScalarTransition(
            const json& object,
            const std::string& path)
        {
            static const std::vector<std::string> allowed = {
                "domainBegin", "domainEnd",
                "valueBegin", "valueEnd",
                "type"
            };
            requireNoUnknownFields(object, allowed, path);

            requireNumber(object, "domainBegin", path);
            requireNumber(object, "domainEnd", path);
            requireNumber(object, "valueBegin", path);
            requireNumber(object, "valueEnd", path);
            requireString(object, "type", path);

            const math::TransitionType tt = transitionTypeFromString(
                object["type"].get<std::string>());

            return math::ScalarTransition{
                object["domainBegin"].get<double>(),
                object["domainEnd"].get<double>(),
                object["valueBegin"].get<double>(),
                object["valueEnd"].get<double>(),
                tt
            };
        }

        ProfileSegment deserializeProfileSegment(
            const json& object,
            const std::string& path)
        {
            static const std::vector<std::string> allowed = {
                "id", "transition"
            };
            requireNoUnknownFields(object, allowed, path);

            requireInteger(object, "id", path);
            requireObject(object, "transition", path);

            const auto id = object["id"].get<std::uint32_t>();

            if (id == invalidSegmentId)
            {
                throw std::runtime_error(
                    path + ".id: segment id must be nonzero");
            }

            return ProfileSegment{
                id,
                deserializeScalarTransition(
                    object["transition"],
                    path + ".transition")
            };
        }

        ChannelProfile deserializeChannelProfile(
            const json& object,
            const std::string& path)
        {
            static const std::vector<std::string> allowed = {
                "nextSegmentId", "segments"
            };
            requireNoUnknownFields(object, allowed, path);

            requireInteger(object, "nextSegmentId", path);
            requireArray(object, "segments", path);

            ChannelProfile channel;
            channel.nextSegmentId =
                object["nextSegmentId"].get<SegmentId>();

            const auto& segmentsJson = object["segments"];
            channel.segments.reserve(segmentsJson.size());

            for (std::size_t i = 0; i < segmentsJson.size(); ++i)
            {
                const std::string segPath =
                    path + ".segments[" + std::to_string(i) + "]";

                if (!segmentsJson[i].is_object())
                {
                    throw std::runtime_error(
                        segPath + ": expected an object");
                }

                channel.segments.push_back(
                    deserializeProfileSegment(segmentsJson[i], segPath));
            }

            return channel;
        }

        GeometricSection deserializeGeometricSection(
            const json& object,
            const std::string& path)
        {
            static const std::vector<std::string> allowed = {
                "roll", "pitch", "yaw"
            };
            requireNoUnknownFields(object, allowed, path);

            requireObject(object, "roll", path);
            requireObject(object, "pitch", path);
            requireObject(object, "yaw", path);

            return GeometricSection{
                deserializeChannelProfile(
                    object["pitch"], path + ".pitch"),
                deserializeChannelProfile(
                    object["yaw"], path + ".yaw"),
                deserializeChannelProfile(
                    object["roll"], path + ".roll")
            };
        }

        PlanarArcRegion deserializePlanarArcRegion(
            const json& object,
            const std::string& path)
        {
            static const std::vector<std::string> allowed = {
                "radius", "sweptAngle", "planeTilt", "bankChange"
            };
            requireNoUnknownFields(object, allowed, path);

            requireNumber(object, "radius", path);
            requireNumber(object, "sweptAngle", path);
            requireNumber(object, "planeTilt", path);
            requireNumber(object, "bankChange", path);

            return PlanarArcRegion{
                object["radius"].get<double>(),
                object["sweptAngle"].get<double>(),
                object["planeTilt"].get<double>(),
                object["bankChange"].get<double>()
            };
        }

        AuthoredTrackSection deserializeSection(
            const json& object,
            const std::string& path)
        {
            // Read kind first to determine allowed keys.
            requireString(object, "kind", path);

            const std::string kindStr =
                object["kind"].get<std::string>();

            std::vector<std::string> allowed;

            if (kindStr == "RateProfiles")
            {
                allowed = {"kind", "length", "rateProfiles"};
            }
            else if (kindStr == "Geometry")
            {
                allowed = {"kind", "length", "planarArc"};
            }
            else
            {
                throw std::runtime_error(
                    path + ".kind: unknown region kind '" + kindStr
                    + "' (expected 'RateProfiles' or 'Geometry')");
            }

            requireNoUnknownFields(object, allowed, path);
            requireNumber(object, "length", path);

            AuthoredTrackSection section;
            section.length = object["length"].get<double>();

            if (kindStr == "RateProfiles")
            {
                requireObject(object, "rateProfiles", path);

                section.kind = RegionKind::RateProfiles;
                section.region = RateProfileRegion{
                    deserializeGeometricSection(
                        object["rateProfiles"],
                        path + ".rateProfiles")
                };
            }
            else
            {
                requireObject(object, "planarArc", path);

                section.kind = RegionKind::Geometry;
                section.region = GeometryRegion{
                    deserializePlanarArcRegion(
                        object["planarArc"],
                        path + ".planarArc")
                };
            }

            return section;
        }

        AuthoredStartPose deserializeStartPose(
            const json& object,
            const std::string& path)
        {
            static const std::vector<std::string> allowed = {
                "position", "orientation"
            };
            requireNoUnknownFields(object, allowed, path);
            requireObject(object, "position", path);
            requireObject(object, "orientation", path);

            const json& position = object["position"];
            const json& orientation = object["orientation"];
            static const std::vector<std::string> positionAllowed = {
                "x", "y", "z"
            };
            static const std::vector<std::string> orientationAllowed = {
                "w", "x", "y", "z"
            };
            requireNoUnknownFields(
                position,
                positionAllowed,
                path + ".position"
            );
            requireNoUnknownFields(
                orientation,
                orientationAllowed,
                path + ".orientation"
            );
            for (const char* const axis : {"x", "y", "z"})
            {
                requireNumber(position, axis, path + ".position");
            }
            for (const char* const component : {"w", "x", "y", "z"})
            {
                requireNumber(
                    orientation,
                    component,
                    path + ".orientation"
                );
            }

            return {
                {position["x"].get<double>(),
                 position["y"].get<double>(),
                 position["z"].get<double>()},
                {orientation["w"].get<double>(),
                 orientation["x"].get<double>(),
                 orientation["y"].get<double>(),
                 orientation["z"].get<double>()}
            };
        }

        // ----------------------------------------------------------------
        // nextSegmentId consistency
        // ----------------------------------------------------------------

        void validateNextSegmentId(
            const ChannelProfile& channel,
            const std::string& channelPath)
        {
            SegmentId maxId = invalidSegmentId;

            for (const auto& segment : channel.segments)
            {
                if (segment.id > maxId)
                {
                    maxId = segment.id;
                }
            }

            if (channel.nextSegmentId <= maxId)
            {
                throw std::runtime_error(
                    channelPath
                    + ".nextSegmentId: must be greater than all "
                    "segment ids (max id = "
                    + std::to_string(maxId)
                    + ", nextSegmentId = "
                    + std::to_string(channel.nextSegmentId)
                    + ")");
            }
        }

        void validateNextSegmentIds(
            const GeometricSection& gs,
            const std::string& sectionPath)
        {
            validateNextSegmentId(gs.roll, sectionPath + ".roll");
            validateNextSegmentId(gs.pitch, sectionPath + ".pitch");
            validateNextSegmentId(gs.yaw, sectionPath + ".yaw");
        }
    }

    // ----------------------------------------------------------------
    // Public API
    // ----------------------------------------------------------------

    std::string serializeCoasterDocument(const AuthoredTrack& track)
    {
        json root;
        root["formatVersion"] = currentFormatVersion;
        root["layoutMode"] = layoutModeToString(track.layoutMode());
        root["startPose"] = serializeStartPose(track.startPose());

        json sectionsJson = json::array();

        for (std::size_t i = 0; i < track.sectionCount(); ++i)
        {
            sectionsJson.push_back(serializeSection(track.section(i)));
        }

        root["sections"] = std::move(sectionsJson);

        return root.dump(4);
    }

    std::expected<AuthoredTrack, std::string>
    deserializeCoasterDocument(const std::string& jsonString)
    {
        try
        {
            // 1. Parse JSON.
            json root = json::parse(jsonString);

            // 2. Root must be an object.
            if (!root.is_object())
            {
                return std::unexpected(
                    std::string("Root document must be a JSON object"));
            }

            // 3. Strict root-level fields.
            static const std::vector<std::string> rootAllowed = {
                "formatVersion", "sections", "layoutMode", "startPose"
            };
            requireNoUnknownFields(root, rootAllowed, "root");

            // 4. formatVersion required, must equal 1.
            if (!root.contains("formatVersion"))
            {
                return std::unexpected(
                    std::string("Missing 'formatVersion'"));
            }

            if (!root["formatVersion"].is_number_integer())
            {
                return std::unexpected(
                    std::string("'formatVersion' must be an integer"));
            }

            const int version =
                root["formatVersion"].get<int>();

            if (version != currentFormatVersion)
            {
                return std::unexpected(
                    "Document format version " + std::to_string(version)
                    + " is not supported by this build (maximum "
                    "supported: "
                    + std::to_string(currentFormatVersion) + ")");
            }

            // 5. sections required, must be non-empty array.
            if (!root.contains("sections") || !root["sections"].is_array())
            {
                return std::unexpected(
                    std::string("Missing or non-array 'sections'"));
            }

            const auto& sectionsJson = root["sections"];

            if (sectionsJson.empty())
            {
                return std::unexpected(
                    std::string(
                        "'sections' must contain at least one section"));
            }

            // 6. Deserialize each section with strict validation.
            std::vector<AuthoredTrackSection> sections;
            sections.reserve(sectionsJson.size());

            for (std::size_t i = 0; i < sectionsJson.size(); ++i)
            {
                const std::string sectionPath =
                    "sections[" + std::to_string(i) + "]";

                if (!sectionsJson[i].is_object())
                {
                    return std::unexpected(
                        sectionPath + ": expected a JSON object");
                }

                AuthoredTrackSection section = deserializeSection(
                    sectionsJson[i], sectionPath);

                // Core section-domain validation. Dispatch on kind
                // using the public validation entry points.
                if (section.kind == RegionKind::RateProfiles)
                {
                    validateGeometricSection(
                        section.rateProfileRegion().rateProfiles,
                        section.length);
                }
                else
                {
                    validatePlanarArcRegion(
                        std::get<PlanarArcRegion>(
                            std::get<GeometryRegion>(section.region)
                                .construction),
                        section.length);
                }

                // nextSegmentId consistency for rate-profile regions.
                if (section.kind == RegionKind::RateProfiles)
                {
                    validateNextSegmentIds(
                        section.rateProfileRegion().rateProfiles,
                        sectionPath);
                }

                sections.push_back(std::move(section));
            }

            // 7. Parse optional layoutMode (default Circuit for backward
            //    compatibility with formatVersion 1 documents that lack
            //    this field).
            LayoutMode layoutMode = LayoutMode::Circuit;

            if (root.contains("layoutMode"))
            {
                if (!root["layoutMode"].is_string())
                {
                    return std::unexpected(
                        std::string(
                            "'layoutMode' must be a string"));
                }

                const std::string modeStr =
                    root["layoutMode"].get<std::string>();

                try
                {
                    layoutMode = layoutModeFromString(modeStr);
                }
                catch (const std::invalid_argument&)
                {
                    return std::unexpected(
                        "Unknown layoutMode '" + modeStr
                        + "' (expected 'Circuit' or 'Shuttle')");
                }
            }

            // Documents written before authored start poses were introduced
            // keep the canonical origin/identity initial condition.
            AuthoredStartPose startPose;
            if (root.contains("startPose"))
            {
                if (!root["startPose"].is_object())
                {
                    return std::unexpected(
                        std::string("'startPose' must be an object"));
                }
                startPose = deserializeStartPose(
                    root["startPose"],
                    "startPose"
                );
            }

            // 8. Build the AuthoredTrack.
            AuthoredTrack track;

            for (std::size_t i = 0; i < sections.size(); ++i)
            {
                if (track.sectionCount() == 0)
                {
                    track.appendSection();
                    track.section(0) = std::move(sections[i]);
                }
                else
                {
                    track.insertSectionAfter(
                        track.sectionCount() - 1,
                        sections[i]);
                }
            }

            track.setLayoutMode(layoutMode);
            track.setStartPose(startPose);

            return track;
        }
        catch (const json::parse_error& e)
        {
            return std::unexpected(
                std::string("JSON parse error: ") + e.what());
        }
        catch (const std::exception& e)
        {
            return std::unexpected(std::string(e.what()));
        }
    }
}
