#include <quantum/coaster/RideHardwareSerialization.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace quantum::coaster
{
    namespace
    {
        using nlohmann::json;
        void fields(const json& j, std::initializer_list<const char*> names)
        {
            if (!j.is_object()) throw std::invalid_argument("Hardware configuration must be an object.");
            for (const auto& [key, value] : j.items())
            {
                bool known = false;
                for (auto name : names) if (key == name) known = true;
                if (!known) throw std::invalid_argument("Unknown hardware configuration field: " + key);
            }
            for (auto name : names)
                if (!j.contains(name)) throw std::invalid_argument(std::string("Missing hardware field: ") + name);
        }
        template<class T> json write(const T& value);
        template<class T> T read(const json& j);
        template<class T> json write(const T& value)
        {
            if constexpr (std::is_enum_v<T>) return static_cast<std::underlying_type_t<T>>(value);
            else return json(value);
        }
        template<class T> T read(const json& j)
        {
            if constexpr (std::is_enum_v<T>) return static_cast<T>(read<std::underlying_type_t<T>>(j));
            else if constexpr (std::is_integral_v<T> && !std::is_same_v<T, bool>)
            {
                if (!j.is_number_integer() || j < std::numeric_limits<T>::lowest() || j > std::numeric_limits<T>::max())
                    throw std::invalid_argument("Hardware integer/ID is out of range.");
                return j.get<T>();
            }
            else if constexpr (std::is_floating_point_v<T>)
            {
                if (!j.is_number()) throw std::invalid_argument("Hardware field must be a number.");
                const auto value = j.get<T>();
                if (!std::isfinite(value)) throw std::invalid_argument("Hardware field must be finite.");
                return value;
            }
            else return j.get<T>();
        }
        template<> json write(const std::optional<double>& value) { return value ? json(*value) : json(nullptr); }
        template<> std::optional<double> read(const json& j) { return j.is_null() ? std::nullopt : std::optional{read<double>(j)}; }
        template<> std::vector<TrackDeviceId> read(const json& j)
        {
            if (!j.is_array()) throw std::invalid_argument("Access gates must be an ID array.");
            std::vector<TrackDeviceId> ids;
            for (const auto& value : j) ids.push_back(read<TrackDeviceId>(value));
            return ids;
        }
        template<> json write(const LaunchConfiguration& value)
        {
            json j = json::object();
            j["targetSpeedMetersPerSecond"] = write(value.targetSpeedMetersPerSecond);
            j["direction"] = write(value.direction);
            return j;
        }
        template<> LaunchConfiguration read(const json& j)
        {
            fields(j, {"targetSpeedMetersPerSecond", "direction"});
            LaunchConfiguration value;
            value.targetSpeedMetersPerSecond = read<decltype(value.targetSpeedMetersPerSecond)>(j.at("targetSpeedMetersPerSecond"));
            value.direction = read<decltype(value.direction)>(j.at("direction"));
            return value;
        }
        template<> json write(const BrakeConfiguration& value)
        {
            json j = json::object();
            j["targetSpeedMetersPerSecond"] = write(value.targetSpeedMetersPerSecond);
            j["holding"] = write(value.holding);
            return j;
        }
        template<> BrakeConfiguration read(const json& j)
        {
            fields(j, {"targetSpeedMetersPerSecond", "holding"});
            BrakeConfiguration value;
            value.targetSpeedMetersPerSecond = read<decltype(value.targetSpeedMetersPerSecond)>(j.at("targetSpeedMetersPerSecond"));
            value.holding = read<decltype(value.holding)>(j.at("holding"));
            return value;
        }
        template<> json write(const TransportConfiguration& value)
        {
            json j = json::object();
            j["speedMetersPerSecond"] = write(value.speedMetersPerSecond);
            j["direction"] = write(value.direction);
            return j;
        }
        template<> TransportConfiguration read(const json& j)
        {
            fields(j, {"speedMetersPerSecond", "direction"});
            TransportConfiguration value;
            value.speedMetersPerSecond = read<decltype(value.speedMetersPerSecond)>(j.at("speedMetersPerSecond"));
            value.direction = read<decltype(value.direction)>(j.at("direction"));
            return value;
        }
        template<> json write(const ChainLiftConfiguration& value)
        {
            json j = json::object();
            j["speedMetersPerSecond"] = write(value.speedMetersPerSecond);
            j["antiRollback"] = write(value.antiRollback);
            return j;
        }
        template<> ChainLiftConfiguration read(const json& j)
        {
            fields(j, {"speedMetersPerSecond", "antiRollback"});
            ChainLiftConfiguration value;
            value.speedMetersPerSecond = read<decltype(value.speedMetersPerSecond)>(j.at("speedMetersPerSecond"));
            value.antiRollback = read<decltype(value.antiRollback)>(j.at("antiRollback"));
            return value;
        }
        template<> json write(const CableLiftConfiguration& value)
        {
            json j = json::object();
            j["speedMetersPerSecond"] = write(value.speedMetersPerSecond);
            j["returnSeconds"] = write(value.returnSeconds);
            j["initiallyAvailable"] = write(value.initiallyAvailable);
            return j;
        }
        template<> CableLiftConfiguration read(const json& j)
        {
            fields(j, {"speedMetersPerSecond", "returnSeconds", "initiallyAvailable"});
            CableLiftConfiguration value;
            value.speedMetersPerSecond = read<decltype(value.speedMetersPerSecond)>(j.at("speedMetersPerSecond"));
            value.returnSeconds = read<decltype(value.returnSeconds)>(j.at("returnSeconds"));
            value.initiallyAvailable = read<decltype(value.initiallyAvailable)>(j.at("initiallyAvailable"));
            return value;
        }
        template<> json write(const StationConfiguration& value)
        {
            json j = json::object();
            j["stopStationMeters"] = write(value.stopStationMeters);
            j["alignmentToleranceMeters"] = write(value.alignmentToleranceMeters);
            j["stoppedToleranceMetersPerSecond"] = write(value.stoppedToleranceMetersPerSecond);
            j["transportSpeedMetersPerSecond"] = write(value.transportSpeedMetersPerSecond);
            j["downstreamBlock"] = write(value.downstreamBlock);
            j["accessGates"] = write(value.accessGates);
            j["requireLockedGates"] = write(value.requireLockedGates);
            return j;
        }
        template<> StationConfiguration read(const json& j)
        {
            fields(j, {"stopStationMeters", "alignmentToleranceMeters", "stoppedToleranceMetersPerSecond", "transportSpeedMetersPerSecond", "downstreamBlock", "accessGates", "requireLockedGates"});
            StationConfiguration value;
            value.stopStationMeters = read<decltype(value.stopStationMeters)>(j.at("stopStationMeters"));
            value.alignmentToleranceMeters = read<decltype(value.alignmentToleranceMeters)>(j.at("alignmentToleranceMeters"));
            value.stoppedToleranceMetersPerSecond = read<decltype(value.stoppedToleranceMetersPerSecond)>(j.at("stoppedToleranceMetersPerSecond"));
            value.transportSpeedMetersPerSecond = read<decltype(value.transportSpeedMetersPerSecond)>(j.at("transportSpeedMetersPerSecond"));
            value.downstreamBlock = read<decltype(value.downstreamBlock)>(j.at("downstreamBlock"));
            value.accessGates = read<decltype(value.accessGates)>(j.at("accessGates"));
            value.requireLockedGates = read<decltype(value.requireLockedGates)>(j.at("requireLockedGates"));
            return value;
        }
        template<> json write(const BlockConfiguration& value)
        {
            json j = json::object();
            return j;
        }
        template<> BlockConfiguration read(const json& j)
        {
            fields(j, {});
            BlockConfiguration value;
            return value;
        }
        template<> json write(const TransferAlignment& value)
        {
            json j = json::object();
            j["id"] = write(value.id);
            j["name"] = write(value.name);
            j["lateralOffsetMeters"] = write(value.lateralOffsetMeters);
            j["destination"] = write(value.destination);
            return j;
        }
        template<> TransferAlignment read(const json& j)
        {
            fields(j, {"id", "name", "lateralOffsetMeters", "destination"});
            TransferAlignment value;
            value.id = read<decltype(value.id)>(j.at("id"));
            value.name = read<decltype(value.name)>(j.at("name"));
            value.lateralOffsetMeters = read<decltype(value.lateralOffsetMeters)>(j.at("lateralOffsetMeters"));
            value.destination = read<decltype(value.destination)>(j.at("destination"));
            return value;
        }
        template<> json write(const std::vector<TransferAlignment>& values)
        {
            json j = json::array();
            for (const auto& value : values) j.push_back(write(value));
            return j;
        }
        template<> std::vector<TransferAlignment> read(const json& j)
        {
            if (!j.is_array()) throw std::invalid_argument("Hardware list must be an array.");
            std::vector<TransferAlignment> values;
            for (const auto& value : j) values.push_back(read<TransferAlignment>(value));
            return values;
        }
        template<> json write(const TransferConfiguration& value)
        {
            json j = json::object();
            j["alignments"] = write(value.alignments);
            j["homeAlignment"] = write(value.homeAlignment);
            j["nextAlignmentId"] = write(value.nextAlignmentId);
            j["movementSpeedMetersPerSecond"] = write(value.movementSpeedMetersPerSecond);
            j["stoppedToleranceMetersPerSecond"] = write(value.stoppedToleranceMetersPerSecond);
            j["accessGates"] = write(value.accessGates);
            return j;
        }
        template<> TransferConfiguration read(const json& j)
        {
            // Older typed configurations derive an allocator ahead of their
            // persistent alignment IDs, without renaming existing destinations.
            json fieldsJson = j;
            if (!fieldsJson.contains("nextAlignmentId"))
            {
                std::uint64_t next = 1;
                for (const auto& a : read<std::vector<TransferAlignment>>(j.at("alignments")))
                {
                    if (a.id == std::numeric_limits<std::uint64_t>::max()) throw std::invalid_argument("Transfer alignment ID space is exhausted.");
                    next = std::max(next, a.id + 1);
                }
                fieldsJson["nextAlignmentId"] = next;
            }
            fields(fieldsJson, {"alignments", "homeAlignment", "nextAlignmentId", "movementSpeedMetersPerSecond", "stoppedToleranceMetersPerSecond", "accessGates"});
            TransferConfiguration value;
            value.nextAlignmentId = read<std::uint64_t>(fieldsJson.at("nextAlignmentId"));
            value.alignments = read<decltype(value.alignments)>(j.at("alignments"));
            value.homeAlignment = read<decltype(value.homeAlignment)>(j.at("homeAlignment"));
            value.movementSpeedMetersPerSecond = read<decltype(value.movementSpeedMetersPerSecond)>(j.at("movementSpeedMetersPerSecond"));
            value.stoppedToleranceMetersPerSecond = read<decltype(value.stoppedToleranceMetersPerSecond)>(j.at("stoppedToleranceMetersPerSecond"));
            value.accessGates = read<decltype(value.accessGates)>(j.at("accessGates"));
            return value;
        }
        template<> json write(const AccessConfiguration& value)
        {
            json j = json::object();
            j["role"] = write(value.role);
            j["motion"] = write(value.motion);
            j["travelDirection"] = write(value.travelDirection);
            j["openAngleDegrees"] = write(value.openAngleDegrees);
            j["travelDistanceMeters"] = write(value.travelDistanceMeters);
            j["movementSeconds"] = write(value.movementSeconds);
            j["lateralOffsetMeters"] = write(value.lateralOffsetMeters);
            j["verticalOffsetMeters"] = write(value.verticalOffsetMeters);
            j["yawDegrees"] = write(value.yawDegrees);
            j["initiallyOpen"] = write(value.initiallyOpen);
            j["initiallyLocked"] = write(value.initiallyLocked);
            j["initiallyInterlocked"] = write(value.initiallyInterlocked);
            j["meshAsset"] = write(value.meshAsset);
            if (value.infill != AccessInfill::Mesh) j["infill"] = write(value.infill);
            if (value.widthMeters != 1.0) j["widthMeters"] = value.widthMeters;
            if (value.heightMeters != 1.1) j["heightMeters"] = value.heightMeters;
            if (value.surface) j["surface"] = json::parse(serializeSupportSurface(*value.surface));
            return j;
        }
        template<> AccessConfiguration read(const json& j)
        {
            auto legacy = j;
            for (const char* key : {"infill", "widthMeters", "heightMeters", "surface"}) legacy.erase(key);
            fields(legacy, {"role", "motion", "travelDirection", "openAngleDegrees", "travelDistanceMeters", "movementSeconds", "lateralOffsetMeters", "verticalOffsetMeters", "yawDegrees", "initiallyOpen", "initiallyLocked", "initiallyInterlocked", "meshAsset"});
            AccessConfiguration value;
            value.role = read<decltype(value.role)>(j.at("role"));
            value.motion = read<decltype(value.motion)>(j.at("motion"));
            value.travelDirection = read<decltype(value.travelDirection)>(j.at("travelDirection"));
            value.openAngleDegrees = read<decltype(value.openAngleDegrees)>(j.at("openAngleDegrees"));
            value.travelDistanceMeters = read<decltype(value.travelDistanceMeters)>(j.at("travelDistanceMeters"));
            value.movementSeconds = read<decltype(value.movementSeconds)>(j.at("movementSeconds"));
            value.lateralOffsetMeters = read<decltype(value.lateralOffsetMeters)>(j.at("lateralOffsetMeters"));
            value.verticalOffsetMeters = read<decltype(value.verticalOffsetMeters)>(j.at("verticalOffsetMeters"));
            value.yawDegrees = read<decltype(value.yawDegrees)>(j.at("yawDegrees"));
            value.initiallyOpen = read<decltype(value.initiallyOpen)>(j.at("initiallyOpen"));
            value.initiallyLocked = read<decltype(value.initiallyLocked)>(j.at("initiallyLocked"));
            value.initiallyInterlocked = read<decltype(value.initiallyInterlocked)>(j.at("initiallyInterlocked"));
            value.meshAsset = read<decltype(value.meshAsset)>(j.at("meshAsset"));
            if (j.contains("infill")) value.infill = read<AccessInfill>(j["infill"]);
            if (j.contains("widthMeters")) value.widthMeters = read<double>(j["widthMeters"]);
            if (j.contains("heightMeters")) value.heightMeters = read<double>(j["heightMeters"]);
            if (j.contains("surface") && !j["surface"].is_null()) value.surface = deserializeSupportSurface(j["surface"].dump());
            return value;
        }
        template<> json write(const RailingConfiguration& value)
        {
            json j = json::object();
            j["inner"] = write(value.inner);
            j["outer"] = write(value.outer);
            j["heightMeters"] = write(value.heightMeters);
            if (value.postSpacingMeters != 1.5) j["postSpacingMeters"] = value.postSpacingMeters;
            return j;
        }
        template<> RailingConfiguration read(const json& j)
        {
            auto legacy = j; legacy.erase("postSpacingMeters");
            fields(legacy, {"inner", "outer", "heightMeters"});
            RailingConfiguration value;
            value.inner = read<decltype(value.inner)>(j.at("inner"));
            value.outer = read<decltype(value.outer)>(j.at("outer"));
            value.heightMeters = read<decltype(value.heightMeters)>(j.at("heightMeters"));
            if (j.contains("postSpacingMeters")) value.postSpacingMeters = read<double>(j["postSpacingMeters"]);
            return value;
        }
        template<> json write(const TrackInfrastructure& value)
        {
            json j = json::object();
            j["id"] = write(value.id);
            j["name"] = write(value.name);
            j["kind"] = write(value.kind);
            j["startStationMeters"] = write(value.startStationMeters);
            j["endStationMeters"] = write(value.endStationMeters);
            j["side"] = write(value.side);
            j["lateralOffsetMeters"] = write(value.lateralOffsetMeters);
            j["verticalOffsetMeters"] = write(value.verticalOffsetMeters);
            j["widthMeters"] = write(value.widthMeters);
            j["deck"] = write(value.deck);
            j["railings"] = write(value.railings);
            j["accessGates"] = write(value.accessGates);
            j["meshAsset"] = write(value.meshAsset);
            if (value.surface) j["surface"] = json::parse(serializeSupportSurface(*value.surface));
            if (value.worldPlacement)
            {
                j["worldPlacement"] = true;
                j["worldStartMeters"] = {value.worldStartMeters.x, value.worldStartMeters.y, value.worldStartMeters.z};
                j["worldEndMeters"] = {value.worldEndMeters.x, value.worldEndMeters.y, value.worldEndMeters.z};
            }
            if (value.stepRiseMeters != .18) j["stepRiseMeters"] = value.stepRiseMeters;
            return j;
        }
        template<> TrackInfrastructure read(const json& j)
        {
            auto legacy = j;
            for (const char* key : {"surface", "worldPlacement", "worldStartMeters", "worldEndMeters", "stepRiseMeters"}) legacy.erase(key);
            fields(legacy, {"id", "name", "kind", "startStationMeters", "endStationMeters", "side", "lateralOffsetMeters", "verticalOffsetMeters", "widthMeters", "deck", "railings", "accessGates", "meshAsset"});
            TrackInfrastructure value;
            value.id = read<decltype(value.id)>(j.at("id"));
            value.name = read<decltype(value.name)>(j.at("name"));
            value.kind = read<decltype(value.kind)>(j.at("kind"));
            value.startStationMeters = read<decltype(value.startStationMeters)>(j.at("startStationMeters"));
            value.endStationMeters = read<decltype(value.endStationMeters)>(j.at("endStationMeters"));
            value.side = read<decltype(value.side)>(j.at("side"));
            value.lateralOffsetMeters = read<decltype(value.lateralOffsetMeters)>(j.at("lateralOffsetMeters"));
            value.verticalOffsetMeters = read<decltype(value.verticalOffsetMeters)>(j.at("verticalOffsetMeters"));
            value.widthMeters = read<decltype(value.widthMeters)>(j.at("widthMeters"));
            value.deck = read<decltype(value.deck)>(j.at("deck"));
            value.railings = read<decltype(value.railings)>(j.at("railings"));
            value.accessGates = read<decltype(value.accessGates)>(j.at("accessGates"));
            value.meshAsset = read<decltype(value.meshAsset)>(j.at("meshAsset"));
            if (j.contains("surface") && !j["surface"].is_null()) value.surface = deserializeSupportSurface(j["surface"].dump());
            if (j.contains("worldPlacement")) value.worldPlacement = read<bool>(j["worldPlacement"]);
            const auto vector = [&](const char* key, glm::dvec3 fallback)
            {
                if (!j.contains(key)) return fallback;
                const auto& a = j[key];
                if (!a.is_array() || a.size() != 3) throw std::invalid_argument("Infrastructure world endpoint needs three coordinates.");
                return glm::dvec3{read<double>(a[0]), read<double>(a[1]), read<double>(a[2])};
            };
            value.worldStartMeters = vector("worldStartMeters", value.worldStartMeters);
            value.worldEndMeters = vector("worldEndMeters", value.worldEndMeters);
            if (j.contains("stepRiseMeters")) value.stepRiseMeters = read<double>(j["stepRiseMeters"]);
            return value;
        }
        template<> json write(const std::vector<TrackInfrastructure>& values)
        {
            json j = json::array();
            for (const auto& value : values) j.push_back(write(value));
            return j;
        }
        template<> std::vector<TrackInfrastructure> read(const json& j)
        {
            if (!j.is_array()) throw std::invalid_argument("Hardware list must be an array.");
            std::vector<TrackInfrastructure> values;
            for (const auto& value : j) values.push_back(read<TrackInfrastructure>(value));
            return values;
        }
    }
    std::string serializeRideHardwareConfiguration(const TrackDevice& device)
    {
        return std::visit([](const auto& value) -> std::string
        {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<T, std::monostate>) return "null";
            else return write(value).dump();
        }, device.configuration);
    }
    HardwareConfiguration deserializeRideHardwareConfiguration(TrackDeviceKind kind, std::string_view text)
    {
        const auto j = json::parse(text);
        if (j.is_null() && (kind == TrackDeviceKind::Launch || kind == TrackDeviceKind::Brake)) return {};
        switch (kind)
        {
        case TrackDeviceKind::Launch: return read<LaunchConfiguration>(j);
        case TrackDeviceKind::Brake: return read<BrakeConfiguration>(j);
        case TrackDeviceKind::Transport: return read<TransportConfiguration>(j);
        case TrackDeviceKind::ChainLift: return read<ChainLiftConfiguration>(j);
        case TrackDeviceKind::CableLift: return read<CableLiftConfiguration>(j);
        case TrackDeviceKind::Station: return read<StationConfiguration>(j);
        case TrackDeviceKind::Block: return read<BlockConfiguration>(j);
        case TrackDeviceKind::Transfer: return read<TransferConfiguration>(j);
        case TrackDeviceKind::Access: return read<AccessConfiguration>(j);
        }
        throw std::invalid_argument("Unknown hardware configuration kind.");
    }
    std::string serializeTrackInfrastructure(const std::vector<TrackInfrastructure>& items)
    {
        return write(items).dump();
    }
    std::vector<TrackInfrastructure> deserializeTrackInfrastructure(std::string_view text)
    {
        return read<std::vector<TrackInfrastructure>>(json::parse(text));
    }
}
