#include <quantum/coaster/TrackDevices.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace quantum::coaster
{
    const char* trackDeviceKindName(TrackDeviceKind kind)
    {
        switch (kind)
        {
        case TrackDeviceKind::Launch: return "Launch";
        case TrackDeviceKind::Brake: return "Brake";
        case TrackDeviceKind::Transport: return "Transport";
        case TrackDeviceKind::ChainLift: return "ChainLift";
        case TrackDeviceKind::CableLift: return "CableLift";
        case TrackDeviceKind::Station: return "Station";
        case TrackDeviceKind::Block: return "Block";
        case TrackDeviceKind::Transfer: return "Transfer";
        case TrackDeviceKind::Access: return "Access";
        }
        throw std::invalid_argument("Unknown ride hardware kind.");
    }
    TrackDeviceKind trackDeviceKindFromName(const std::string& name)
    {
        for (unsigned i = 0; i <= static_cast<unsigned>(TrackDeviceKind::Access); ++i)
            if (name == trackDeviceKindName(static_cast<TrackDeviceKind>(i)))
                return static_cast<TrackDeviceKind>(i);
        throw std::invalid_argument("Unknown ride hardware kind: " + name);
    }
    bool supportsAccelerationProfile(TrackDeviceKind kind) noexcept
    {
        return kind == TrackDeviceKind::Launch || kind == TrackDeviceKind::Brake
            || kind == TrackDeviceKind::CableLift;
    }
    TrackDevice makeTrackDevice(TrackDeviceKind kind)
    {
        TrackDevice result;
        result.kind = kind;
        result.name = trackDeviceKindName(kind);
        switch (kind)
        {
        case TrackDeviceKind::Launch: result.configuration = LaunchConfiguration{}; break;
        case TrackDeviceKind::Brake: result.configuration = BrakeConfiguration{}; break;
        case TrackDeviceKind::Transport: result.configuration = TransportConfiguration{}; break;
        case TrackDeviceKind::ChainLift: result.configuration = ChainLiftConfiguration{}; break;
        case TrackDeviceKind::CableLift: result.configuration = CableLiftConfiguration{}; break;
        case TrackDeviceKind::Station: result.configuration = StationConfiguration{}; break;
        case TrackDeviceKind::Block: result.configuration = BlockConfiguration{}; break;
        case TrackDeviceKind::Transfer: result.configuration = TransferConfiguration{}; break;
        case TrackDeviceKind::Access: result.configuration = AccessConfiguration{}; break;
        }
        return result;
    }
    namespace
    {
        void require(bool value, const char* message)
        {
            if (!value) throw std::invalid_argument(message);
        }
        bool positive(double value) { return std::isfinite(value) && value > 0; }
        bool nonnegative(double value) { return std::isfinite(value) && value >= 0; }
        bool validDirection(HardwareDirection value)
        {
            return value == HardwareDirection::Forward || value == HardwareDirection::Reverse;
        }
        void gatesValid(const std::vector<TrackDeviceId>& gates, const TrackDeviceCollection& collection)
        {
            std::unordered_set<TrackDeviceId> seen;
            for (auto id : gates)
            {
                const auto gate = std::find_if(collection.devices.begin(), collection.devices.end(),
                    [id](const auto& d) { return d.id == id; });
                require(seen.insert(id).second && gate != collection.devices.end()
                    && gate->kind == TrackDeviceKind::Access, "Access interlock references an unknown, duplicate or non-access ID.");
            }
        }
    }
    void validateRideHardware(const TrackDeviceCollection& collection, double length, bool circuit)
    {
        for (const auto& device : collection.devices)
        {
            const auto& config = device.configuration;
            const auto expected = makeTrackDevice(device.kind).configuration.index();
            require(config.index() == expected || (config.index() == 0
                && (device.kind == TrackDeviceKind::Launch || device.kind == TrackDeviceKind::Brake)),
                "Ride hardware configuration does not match its kind.");
            if (const auto* c = std::get_if<LaunchConfiguration>(&config))
                require(validDirection(c->direction) && (!c->targetSpeedMetersPerSecond
                    || positive(*c->targetSpeedMetersPerSecond)), "Invalid launch target or direction.");
            if (const auto* c = std::get_if<BrakeConfiguration>(&config))
                require(nonnegative(c->targetSpeedMetersPerSecond), "Invalid brake trim speed.");
            if (const auto* c = std::get_if<TransportConfiguration>(&config))
                require(positive(c->speedMetersPerSecond) && validDirection(c->direction), "Invalid transport speed/direction.");
            if (const auto* c = std::get_if<ChainLiftConfiguration>(&config))
                require(positive(c->speedMetersPerSecond), "Invalid chain speed.");
            if (const auto* c = std::get_if<CableLiftConfiguration>(&config))
                require(positive(c->speedMetersPerSecond) && positive(c->returnSeconds), "Invalid cable speed/return duration.");
            if (const auto* c = std::get_if<StationConfiguration>(&config))
            {
                require(trackDeviceContainsStation(device, c->stopStationMeters, length, circuit)
                    && positive(c->alignmentToleranceMeters) && positive(c->stoppedToleranceMetersPerSecond)
                    && positive(c->transportSpeedMetersPerSecond), "Invalid station stop target or tolerances.");
                if (c->downstreamBlock)
                {
                    const auto block = std::find_if(collection.devices.begin(), collection.devices.end(),
                        [&](const auto& d) { return d.id == c->downstreamBlock; });
                    require(block != collection.devices.end() && block->kind == TrackDeviceKind::Block,
                        "Station downstream permissive must reference a block.");
                }
                gatesValid(c->accessGates, collection);
            }
            if (const auto* c = std::get_if<TransferConfiguration>(&config))
            {
                require(!c->alignments.empty() && positive(c->movementSpeedMetersPerSecond)
                    && positive(c->stoppedToleranceMetersPerSecond), "Invalid transfer movement configuration.");
                std::unordered_set<std::uint64_t> ids;
                for (const auto& a : c->alignments)
                    require(a.id && ids.insert(a.id).second && !a.name.empty()
                        && std::isfinite(a.lateralOffsetMeters) && a.destination <= DestinationKind::Maintenance,
                        "Invalid transfer alignment.");
                require(ids.contains(c->homeAlignment), "Transfer home alignment is missing.");
                require(c->nextAlignmentId != 0 && std::all_of(c->alignments.begin(), c->alignments.end(),
                    [&](const auto& a) { return a.id < c->nextAlignmentId; }), "Transfer next alignment ID must exceed allocated IDs.");
                gatesValid(c->accessGates, collection);
            }
            if (const auto* c = std::get_if<AccessConfiguration>(&config))
                require(c->role <= AccessRole::Barrier && c->motion <= AccessMotion::Slide
                    && validDirection(c->travelDirection) && positive(c->openAngleDegrees) && c->openAngleDegrees <= 180
                    && positive(c->travelDistanceMeters) && positive(c->movementSeconds)
                    && std::isfinite(c->lateralOffsetMeters) && std::isfinite(c->verticalOffsetMeters)
                    && std::isfinite(c->yawDegrees) && !(c->initiallyOpen && c->initiallyLocked),
                    "Invalid access motion or initial lock state.");
        }
        std::unordered_set<std::uint64_t> ids;
        std::uint64_t greatest = 0;
        require(collection.nextInfrastructureId != 0, "Infrastructure next ID must be nonzero.");
        for (const auto& item : collection.infrastructure)
        {
            TrackDevice interval;
            interval.startStationMeters = item.startStationMeters; interval.endStationMeters = item.endStationMeters;
            require(item.id && ids.insert(item.id).second && !item.name.empty()
                && item.kind <= InfrastructureKind::Landing && (item.side == TrackSide::Left || item.side == TrackSide::Right)
                && item.deck <= DeckStyle::Grating && positive(item.widthMeters) && positive(item.railings.heightMeters)
                && std::isfinite(item.lateralOffsetMeters) && std::isfinite(item.verticalOffsetMeters)
                && std::isfinite(item.startStationMeters) && std::isfinite(item.endStationMeters)
                && item.startStationMeters >= 0 && item.startStationMeters < length && item.endStationMeters >= 0
                && (circuit ? item.endStationMeters < length : item.endStationMeters <= length)
                && trackDeviceLengthMeters(interval, length, circuit) > 0,
                "Invalid track-relative infrastructure.");
            greatest = std::max(greatest, item.id);
            gatesValid(item.accessGates, collection);
        }
        require(collection.nextInfrastructureId > greatest, "Infrastructure next ID must exceed allocated IDs.");
    }
}
