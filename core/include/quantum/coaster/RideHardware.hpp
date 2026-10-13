#pragma once

#include <quantum/coaster/SupportSurface.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace quantum::coaster
{
    using TrackDeviceId = std::uint64_t;
    enum class HardwareDirection : std::int8_t { Reverse = -1, Forward = 1 };
    struct LaunchConfiguration
    {
        std::optional<double> targetSpeedMetersPerSecond;
        HardwareDirection direction = HardwareDirection::Forward;
        bool operator==(const LaunchConfiguration&) const = default;
    };
    struct BrakeConfiguration
    {
        double targetSpeedMetersPerSecond = 0;
        bool holding = false;
        bool operator==(const BrakeConfiguration&) const = default;
    };
    struct TransportConfiguration
    {
        double speedMetersPerSecond = 2;
        HardwareDirection direction = HardwareDirection::Forward;
        bool operator==(const TransportConfiguration&) const = default;
    };
    struct ChainLiftConfiguration
    {
        double speedMetersPerSecond = 3.5;
        bool antiRollback = true;
        bool operator==(const ChainLiftConfiguration&) const = default;
    };
    struct CableLiftConfiguration
    {
        double speedMetersPerSecond = 5;
        double returnSeconds = 8;
        bool initiallyAvailable = true;
        bool operator==(const CableLiftConfiguration&) const = default;
    };
    struct StationConfiguration
    {
        double stopStationMeters = 0;
        double alignmentToleranceMeters = .15;
        double stoppedToleranceMetersPerSecond = .05;
        double transportSpeedMetersPerSecond = 2;
        TrackDeviceId downstreamBlock = 0;
        std::vector<TrackDeviceId> accessGates;
        bool requireLockedGates = true;
        bool operator==(const StationConfiguration&) const = default;
    };
    struct BlockConfiguration
    {
        bool operator==(const BlockConfiguration&) const = default;
    };
    enum class DestinationKind : std::uint8_t { MainTrack, Storage, Maintenance };
    struct TransferAlignment
    {
        std::uint64_t id = 1;
        std::string name = "Main track";
        double lateralOffsetMeters = 0;
        DestinationKind destination = DestinationKind::MainTrack;
        bool operator==(const TransferAlignment&) const = default;
    };
    struct TransferConfiguration
    {
        std::vector<TransferAlignment> alignments{{}, {2, "Storage 1", 4, DestinationKind::Storage}};
        std::uint64_t homeAlignment = 1;
        std::uint64_t nextAlignmentId = 3;
        double movementSpeedMetersPerSecond = .5;
        double stoppedToleranceMetersPerSecond = .05;
        std::vector<TrackDeviceId> accessGates;
        bool operator==(const TransferConfiguration&) const = default;
    };
    // Role and appearance do not define mechanism behavior. A station mesh gate
    // and a sliding storage door share the same operation/telemetry contract.
    enum class AccessRole : std::uint8_t { Station, Queue, Maintenance, Catwalk, Transfer, Storage, Barrier };
    enum class AccessMotion : std::uint8_t { Swing, Slide };
    enum class AccessInfill : std::uint8_t { Mesh, Solid };
    struct AccessConfiguration
    {
        AccessRole role = AccessRole::Station;
        AccessMotion motion = AccessMotion::Swing;
        HardwareDirection travelDirection = HardwareDirection::Forward;
        double openAngleDegrees = 90;
        double travelDistanceMeters = 1;
        double movementSeconds = 1;
        double lateralOffsetMeters = 2;
        double verticalOffsetMeters = 0;
        double yawDegrees = 0;
        bool initiallyOpen = false;
        bool initiallyLocked = true;
        bool initiallyInterlocked = false;
        std::string meshAsset;
        AccessInfill infill = AccessInfill::Mesh;
        double widthMeters = 1.0, heightMeters = 1.1;
        std::optional<SupportSurface> surface;
        bool operator==(const AccessConfiguration&) const = default;
    };
    // monostate preserves legacy Launch/Brake semantics and serialized fields.
    using HardwareConfiguration = std::variant<std::monostate, LaunchConfiguration,
        BrakeConfiguration, TransportConfiguration, ChainLiftConfiguration,
        CableLiftConfiguration, StationConfiguration, BlockConfiguration,
        TransferConfiguration, AccessConfiguration>;

    enum class InfrastructureKind : std::uint8_t { Catwalk, Platform, Guardrail, Stairs, Landing };
    enum class TrackSide : std::int8_t { Left = -1, Right = 1 };
    enum class DeckStyle : std::uint8_t { Solid, Grating };
    struct RailingConfiguration
    {
        bool inner = false, outer = true;
        double heightMeters = 1.1;
        double postSpacingMeters = 1.5;
        bool operator==(const RailingConfiguration&) const = default;
    };
    struct TrackInfrastructure
    {
        std::uint64_t id = 0;
        std::string name;
        InfrastructureKind kind = InfrastructureKind::Catwalk;
        double startStationMeters = 0, endStationMeters = 0;
        TrackSide side = TrackSide::Right;
        double lateralOffsetMeters = 1.5, verticalOffsetMeters = 0;
        double widthMeters = .8;
        DeckStyle deck = DeckStyle::Grating;
        RailingConfiguration railings;
        std::vector<TrackDeviceId> accessGates;
        std::string meshAsset;
        std::optional<SupportSurface> surface;
        // Explicit world endpoints for stairs / platforms. SI coordinates;
        // track-relative objects retain their existing attachment semantics.
        bool worldPlacement = false;
        glm::dvec3 worldStartMeters{0,0,0}, worldEndMeters{3,0,1.5};
        double stepRiseMeters = .18;
        bool operator==(const TrackInfrastructure&) const = default;
    };
}
