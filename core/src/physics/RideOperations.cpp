#include <quantum/physics/RideOperations.hpp>
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace quantum::physics
{
    using namespace coaster;
    TrainObservation observeTrain(const TrainPose& pose, double speed, OperationTrainId id)
    {
        TrainObservation result{id, pose.generalizedReferenceLocation().stationMeters, speed, {}};
        result.bogies.reserve(pose.carCount() * 2);
        for (std::size_t car = 0; car < pose.carCount(); ++car)
            for (const auto* bogie : {&pose.cars()[car].carPose().frontBogie(), &pose.cars()[car].carPose().rearBogie()})
                result.bogies.push_back({car, bogie->definitionIndex(), bogie->location().stationMeters});
        return result;
    }
    namespace
    {
        HardwareOccupancy occupancy(const TrackDevice& device, std::span<const TrainObservation> trains,
            const HardwareOccupancy& previous, double length, bool circuit)
        {
            HardwareOccupancy result;
            for (const auto& train : trains)
            {
                std::size_t count = 0;
                std::set<std::size_t> cars;
                for (const auto& bogie : train.bogies)
                    if (trackDeviceContainsStation(device, bogie.stationMeters, length, circuit))
                    {
                        ++count; cars.insert(bogie.car);
                        result.bogies.push_back({train.id, bogie.car, bogie.bogie});
                    }
                if (count) result.trains.push_back({train.id, count, cars.size(), count == train.bogies.size()});
            }
            const auto contains = [](const auto& values, auto id)
            { return std::any_of(values.begin(), values.end(), [id](const auto& value) { return value.id == id; }); };
            for (const auto& train : result.trains)
                if (!contains(previous.trains, train.id)) result.entered.push_back(train.id);
            for (const auto& train : previous.trains)
                if (!contains(result.trains, train.id)) result.exited.push_back(train.id);
            return result;
        }
        double servo(double target, double actual, double limit)
        {
            return std::clamp((target - actual) / defaultFixedTimeStepSeconds, -limit, limit);
        }
        double profileCommand(const TrackDevice& device, double station, double length, bool circuit)
        {
            if (!device.accelerationProfile) return device.targetAccelerationMetersPerSecondSquared;
            double distance = station - device.startStationMeters;
            if (circuit && distance < 0) distance += length;
            return evaluateChannelProfile(*device.accelerationProfile,
                std::clamp(distance, 0.0, trackDeviceLengthMeters(device, length, circuit)));
        }
    }
    void RideOperations::reset(std::span<const TrackDevice> devices)
    {
        authored_.assign(devices.begin(), devices.end());
        hardware_.clear(); hardware_.reserve(devices.size());
        for (const auto& device : devices)
        {
            HardwareRuntime value;
            value.id = device.id; value.enabled = device.enabled; value.forceControl = {device.id, device.enabled};
            switch (device.kind)
            {
            case TrackDeviceKind::ChainLift: value.mechanism = ChainRuntime{}; break;
            case TrackDeviceKind::CableLift:
            {
                CableRuntime cable;
                cable.available = std::get<CableLiftConfiguration>(device.configuration).initiallyAvailable;
                cable.phase = cable.available ? CableRuntime::Phase::Available : CableRuntime::Phase::Unavailable;
                value.mechanism = cable; break;
            }
            case TrackDeviceKind::Station: value.mechanism = StationRuntime{}; break;
            case TrackDeviceKind::Transfer:
            {
                const auto& c = std::get<TransferConfiguration>(device.configuration);
                const auto home = std::find_if(c.alignments.begin(), c.alignments.end(),
                    [&](const auto& a) { return a.id == c.homeAlignment; });
                if (home == c.alignments.end()) throw std::invalid_argument("Transfer home alignment is missing.");
                TransferRuntime transfer;
                transfer.alignment = transfer.target = c.homeAlignment;
                transfer.positionMeters = home->lateralOffsetMeters;
                value.mechanism = transfer; break;
            }
            case TrackDeviceKind::Access:
            {
                const auto& c = std::get<AccessConfiguration>(device.configuration);
                AccessRuntime gate;
                gate.phase = c.initiallyOpen ? AccessRuntime::Phase::Open : AccessRuntime::Phase::Closed;
                gate.fraction = c.initiallyOpen ? 1 : 0; gate.locked = c.initiallyLocked; gate.interlocked = c.initiallyInterlocked;
                value.mechanism = gate; break;
            }
            default: break;
            }
            hardware_.push_back(std::move(value));
        }
    }
    HardwareRuntime& RideOperations::find(TrackDeviceId id)
    {
        auto found = std::find_if(hardware_.begin(), hardware_.end(), [id](const auto& h) { return h.id == id; });
        if (found == hardware_.end()) throw std::invalid_argument("Unknown runtime hardware ID.");
        return *found;
    }
    void RideOperations::command(TrackDeviceId id, HardwareCommand command, double value, std::uint64_t target)
    {
        if (!std::isfinite(value)) throw std::invalid_argument("Hardware command requires a finite value.");
        auto& h = find(id);
        if (command == HardwareCommand::SetEnabled) { h.enabled = value != 0; return; }
        if (!h.enabled) throw std::invalid_argument("Hardware is disabled.");
        if (command == HardwareCommand::SetAvailable)
        {
            if (auto* c = std::get_if<ChainRuntime>(&h.mechanism)) { c->available = value != 0; return; }
            if (auto* c = std::get_if<CableRuntime>(&h.mechanism)) { c->available = value != 0; return; }
        }
        if (command == HardwareCommand::RequestDispatch)
            if (auto* s = std::get_if<StationRuntime>(&h.mechanism)) { s->dispatchRequested = true; return; }
        if (command == HardwareCommand::MoveTransfer)
            if (auto* t = std::get_if<TransferRuntime>(&h.mechanism))
            {
                const auto index = static_cast<std::size_t>(&h - hardware_.data());
                const auto& c = std::get<TransferConfiguration>(authored_[index].configuration);
                auto alignment = std::find_if(c.alignments.begin(), c.alignments.end(),
                    [target](const auto& a) { return a.id == target; });
                if (alignment == c.alignments.end()) throw std::invalid_argument("Unknown transfer alignment.");
                if (t->phase != TransferRuntime::Phase::Locked || !h.movementPermitted)
                    throw std::invalid_argument("Transfer must be locked with a secured train and closed access gates.");
                t->target = alignment->id; t->moveRequested = t->target != t->alignment; return;
            }
        if (auto* gate = std::get_if<AccessRuntime>(&h.mechanism))
        {
            if (command == HardwareCommand::SetAccessInterlocked) { gate->interlocked = value != 0; return; }
            if (command == HardwareCommand::CloseAccess)
            { gate->phase = gate->fraction == 0 ? AccessRuntime::Phase::Closed : AccessRuntime::Phase::Closing; return; }
            if (command == HardwareCommand::LockAccess)
            {
                if (gate->phase != AccessRuntime::Phase::Closed) throw std::invalid_argument("Access hardware can lock only when closed.");
                gate->locked = true; return;
            }
            if (gate->interlocked || gate->protectedByOperation) throw std::invalid_argument("Access hardware is interlocked by ride operations.");
            if (command == HardwareCommand::UnlockAccess) { gate->locked = false; return; }
            if (command == HardwareCommand::OpenAccess)
            {
                if (gate->locked) throw std::invalid_argument("Unlock access hardware before opening.");
                if (gate->phase != AccessRuntime::Phase::Open) gate->phase = AccessRuntime::Phase::Opening;
                return;
            }
        }
        throw std::invalid_argument("Command does not apply to this hardware family.");
    }
    void RideOperations::step(std::span<const TrackDevice> devices, std::span<const TrainObservation> trains,
        double length, bool circuit, double seconds)
    {
        if (devices.size() != hardware_.size() || !std::isfinite(seconds) || seconds < 0
            || !std::isfinite(length) || length <= 0)
            throw std::invalid_argument("Invalid operations step or stale hardware configuration.");
        std::set<OperationTrainId> trainIds;
        for (const auto& train : trains)
        {
            if (!train.id || !trainIds.insert(train.id).second || !std::isfinite(train.referenceStationMeters)
                || !std::isfinite(train.signedSpeedMetersPerSecond) || train.bogies.empty())
                throw std::invalid_argument("Operations requires unique train IDs and finite observations.");
            std::set<std::pair<std::size_t, std::size_t>> bogies;
            for (const auto& b : train.bogies)
                if (!std::isfinite(b.stationMeters) || !bogies.emplace(b.car, b.bogie).second)
                    throw std::invalid_argument("Invalid/duplicate bogie observation.");
        }
        // Occupancy for every family is published before any permissive checks.
        // This makes block/gate references independent of document ordering.
        for (std::size_t i = 0; i < devices.size(); ++i)
        {
            auto& h = hardware_[i];
            if (h.id != devices[i].id) throw std::invalid_argument("Operations hardware IDs are stale.");
            h.occupancy = occupancy(devices[i], trains, h.occupancy, length, circuit);
            h.forceControl = {h.id, false}; h.interlockReason.clear();
            h.commandedAccelerationMetersPerSecondSquared = h.commandedSpeedMetersPerSecond = 0;
            h.actualSpeedMetersPerSecond = 0; h.appliedForceNewtons = 0;
            h.profileAccelerationMetersPerSecondSquared = 0; h.targetSpeedLimited = false;
            h.aligned = h.stopped = h.movementPermitted = false;
            if (auto* gate = std::get_if<AccessRuntime>(&h.mechanism)) gate->protectedByOperation = false;
        }
        // Gate motion first, then operations can reserve closed gates this tick.
        for (std::size_t i = 0; i < devices.size(); ++i)
            if (auto* g = std::get_if<AccessRuntime>(&hardware_[i].mechanism))
            {
                auto& h = hardware_[i];
                const auto& c = std::get<AccessConfiguration>(devices[i].configuration);
                if (h.enabled && seconds > 0)
                {
                    if (g->phase == AccessRuntime::Phase::Opening && !g->interlocked)
                    { g->fraction = std::min(1.0, g->fraction + seconds / c.movementSeconds); if (g->fraction == 1) g->phase = AccessRuntime::Phase::Open; }
                    if (g->phase == AccessRuntime::Phase::Closing)
                    { g->fraction = std::max(0.0, g->fraction - seconds / c.movementSeconds); if (g->fraction == 0) g->phase = AccessRuntime::Phase::Closed; }
                }
                h.state = !h.enabled ? "Disabled" : g->interlocked ? "Interlocked" : g->phase == AccessRuntime::Phase::Closed ? "Closed"
                    : g->phase == AccessRuntime::Phase::Open ? "Open" : g->phase == AccessRuntime::Phase::Opening ? "Opening" : "Closing";
            }
        const auto gatesClosed = [&](const auto& ids, bool requireLocked)
        {
            for (auto id : ids)
            {
                const auto& h = find(id); const auto* g = std::get_if<AccessRuntime>(&h.mechanism);
                if (!h.enabled || !g || g->phase != AccessRuntime::Phase::Closed || g->interlocked || (requireLocked && !g->locked)) return false;
            }
            return true;
        };
        const auto protectGates = [&](const auto& ids)
        { for (auto id : ids) std::get<AccessRuntime>(find(id).mechanism).protectedByOperation = true; };
        for (std::size_t i = 0; i < devices.size(); ++i)
        {
            const auto& d = devices[i]; auto& h = hardware_[i];
            if (d.kind == TrackDeviceKind::Access) continue;
            const bool occupied = !h.occupancy.clear();
            const TrainObservation* train = nullptr;
            if (h.occupancy.trains.size() == 1)
                for (const auto& t : trains) if (t.id == h.occupancy.trains.front().id) train = &t;
            const double speed = train ? train->signedSpeedMetersPerSecond : 0;
            h.actualSpeedMetersPerSecond = speed;
            const bool complete = train && h.occupancy.trains.front().complete;
            if (h.occupancy.trains.size() > 1 && d.kind != TrackDeviceKind::Block)
            {
                h.state = "Interlocked"; h.interlockReason = "Multiple trains in command zone";
                if (auto* station = std::get_if<StationRuntime>(&h.mechanism))
                {
                    station->dispatchPermitted = false;
                    if (station->departing) protectGates(std::get<StationConfiguration>(d.configuration).accessGates);
                }
                if (auto* transfer = std::get_if<TransferRuntime>(&h.mechanism); transfer && transfer->moveRequested)
                {
                    transfer->phase = TransferRuntime::Phase::Interlocked;
                    protectGates(std::get<TransferConfiguration>(d.configuration).accessGates);
                }
                continue;
            }
            h.state = h.enabled ? occupied ? "Occupied" : "Ready" : "Disabled";
            const auto drive = [&](double target, double limit)
            {
                h.commandedSpeedMetersPerSecond = target;
                h.commandedAccelerationMetersPerSecondSquared = servo(target, speed, limit);
                h.forceControl = {h.id, h.enabled && train != nullptr,
                    h.commandedAccelerationMetersPerSecondSquared, true};
            };
            if (d.kind == TrackDeviceKind::Block) { h.state = occupied ? "Occupied" : "Clear"; continue; }
            if (auto* transfer = std::get_if<TransferRuntime>(&h.mechanism))
            {
                const auto& c = std::get<TransferConfiguration>(d.configuration);
                const bool secured = !occupied || (complete && std::abs(speed) <= c.stoppedToleranceMetersPerSecond);
                const bool gates = gatesClosed(c.accessGates, true);
                h.stopped = secured;
                h.movementPermitted = h.enabled && secured && gates;
                if (occupied) drive(0, d.targetAccelerationMetersPerSecondSquared);
                if (transfer->moveRequested && seconds > 0)
                {
                    if (!h.movementPermitted) { transfer->phase = TransferRuntime::Phase::Interlocked; h.interlockReason = "Train not secured or access gate not ready"; }
                    else if (transfer->phase == TransferRuntime::Phase::Locked || transfer->phase == TransferRuntime::Phase::Interlocked)
                        transfer->phase = TransferRuntime::Phase::Unlocking;
                    else if (transfer->phase == TransferRuntime::Phase::Unlocking) transfer->phase = TransferRuntime::Phase::Moving;
                    else if (transfer->phase == TransferRuntime::Phase::Moving)
                    {
                        const auto target = std::find_if(c.alignments.begin(), c.alignments.end(), [&](const auto& a) { return a.id == transfer->target; });
                        const double delta = target->lateralOffsetMeters - transfer->positionMeters;
                        transfer->positionMeters += std::clamp(delta, -c.movementSpeedMetersPerSecond * seconds, c.movementSpeedMetersPerSecond * seconds);
                        if (std::abs(target->lateralOffsetMeters - transfer->positionMeters) <= 1e-9)
                        { transfer->positionMeters = target->lateralOffsetMeters; transfer->phase = TransferRuntime::Phase::Locking; }
                    }
                    else if (transfer->phase == TransferRuntime::Phase::Locking)
                    { transfer->phase = TransferRuntime::Phase::Locked; transfer->alignment = transfer->target; transfer->moveRequested = false; }
                }
                h.aligned = transfer->phase == TransferRuntime::Phase::Locked;
                if (transfer->moveRequested) protectGates(c.accessGates);
                h.state = !h.enabled ? "Disabled" : transfer->phase == TransferRuntime::Phase::Locked ? "Locked"
                    : transfer->phase == TransferRuntime::Phase::Unlocking ? "Unlocking" : transfer->phase == TransferRuntime::Phase::Moving ? "Moving"
                    : transfer->phase == TransferRuntime::Phase::Locking ? "Locking" : "Interlocked";
                continue;
            }
            if (auto* station = std::get_if<StationRuntime>(&h.mechanism))
            {
                const auto& c = std::get<StationConfiguration>(d.configuration);
                double error = c.stopStationMeters - (train ? train->referenceStationMeters : c.stopStationMeters);
                if (circuit && std::abs(error) > length * .5) error -= std::copysign(length, error);
                h.aligned = complete && std::abs(error) <= c.alignmentToleranceMeters;
                h.stopped = occupied && std::abs(speed) <= c.stoppedToleranceMetersPerSecond;
                const auto blockClearForTrain = [&]
                {
                    if (!c.downstreamBlock) return true;
                    const auto& block = find(c.downstreamBlock);
                    if (!block.enabled) return false;
                    for (const auto& present : block.occupancy.trains)
                        if (!station->departing || present.id != station->train) return false;
                    return true;
                };
                const bool blockClear = blockClearForTrain();
                const bool gates = gatesClosed(c.accessGates, c.requireLockedGates);
                station->dispatchPermitted = h.enabled && h.aligned && h.stopped && blockClear && gates;
                if (station->departing && (!train || train->id != station->train)) station->departing = false;
                if (seconds > 0 && station->dispatchRequested && station->dispatchPermitted && !station->departing)
                { station->departing = true; station->train = train->id; station->dispatchRequested = false; }
                if (station->departing) protectGates(c.accessGates);
                if (station->departing && (!h.enabled || !blockClear || !gates))
                { drive(0, d.targetAccelerationMetersPerSecondSquared); h.state = "Interlocked"; h.interlockReason = "Departure permissive lost"; }
                else if (station->departing) { drive(c.transportSpeedMetersPerSecond, d.targetAccelerationMetersPerSecondSquared); h.state = "Leaving"; }
                else if (occupied && h.enabled)
                {
                    double target = h.aligned ? 0 : std::copysign(std::min(c.transportSpeedMetersPerSecond,
                        std::sqrt(2 * d.targetAccelerationMetersPerSecondSquared * std::abs(error))), error);
                    drive(target, d.targetAccelerationMetersPerSecondSquared);
                    h.state = h.aligned && h.stopped ? "Holding" : "Positioning";
                }
                if (station->dispatchRequested && !station->dispatchPermitted)
                {
                    h.interlockReason = !h.enabled ? "Disabled" : !complete ? "Train not fully in station" : !h.aligned || !h.stopped ? "Train not aligned/stopped"
                        : !blockClear ? "Downstream block occupied/unavailable" : "Access gate not closed/locked/enabled";
                }
                continue;
            }
            if (auto* cable = std::get_if<CableRuntime>(&h.mechanism))
            {
                const auto& c = std::get<CableLiftConfiguration>(d.configuration);
                if (seconds > 0)
                {
                    if (!h.enabled || !cable->available) h.interlockReason = "Catch car unavailable";
                    else if (cable->phase == CableRuntime::Phase::Unavailable) cable->phase = CableRuntime::Phase::Available;
                    if (cable->phase == CableRuntime::Phase::Returning)
                    {
                        cable->returnRemainingSeconds = std::max(0.0, cable->returnRemainingSeconds - seconds);
                        if (cable->returnRemainingSeconds == 0) cable->phase = cable->available ? CableRuntime::Phase::Available : CableRuntime::Phase::Unavailable;
                    }
                    else if (cable->phase == CableRuntime::Phase::Releasing)
                    { cable->phase = CableRuntime::Phase::Returning; cable->returnRemainingSeconds = c.returnSeconds; }
                    else if ((cable->phase == CableRuntime::Phase::Pulling || cable->phase == CableRuntime::Phase::Engaging) && !occupied)
                        cable->phase = CableRuntime::Phase::Releasing;
                    else if (h.enabled && cable->available && train && speed >= 0)
                    {
                        if (cable->phase == CableRuntime::Phase::Available) cable->phase = CableRuntime::Phase::Engaging;
                        else if (cable->phase == CableRuntime::Phase::Engaging) cable->phase = CableRuntime::Phase::Pulling;
                    }
                }
                if (h.enabled && cable->available && cable->phase == CableRuntime::Phase::Pulling && train)
                    drive(c.speedMetersPerSecond, profileCommand(d, train->referenceStationMeters, length, circuit));
                h.state = !h.enabled || !cable->available ? "Unavailable" : cable->phase == CableRuntime::Phase::Available ? "Available"
                    : cable->phase == CableRuntime::Phase::Engaging ? "Engaging" : cable->phase == CableRuntime::Phase::Pulling ? "Pulling"
                    : cable->phase == CableRuntime::Phase::Releasing ? "Releasing" : cable->phase == CableRuntime::Phase::Returning ? "Returning" : "Unavailable";
                continue;
            }
            if (auto* chain = std::get_if<ChainRuntime>(&h.mechanism))
            {
                const auto& c = std::get<ChainLiftConfiguration>(d.configuration);
                if (seconds > 0)
                {
                    if (!occupied && (chain->phase == ChainRuntime::Phase::Pulling || chain->phase == ChainRuntime::Phase::Engaging)) chain->phase = ChainRuntime::Phase::Releasing;
                    else if (!occupied) chain->phase = ChainRuntime::Phase::Available;
                    else if (h.enabled && chain->available && train)
                    {
                        if (chain->phase == ChainRuntime::Phase::Available || chain->phase == ChainRuntime::Phase::Releasing) chain->phase = ChainRuntime::Phase::Engaging;
                        else if (chain->phase == ChainRuntime::Phase::Engaging) chain->phase = ChainRuntime::Phase::Pulling;
                    }
                }
                if (train && h.enabled && chain->available && chain->phase == ChainRuntime::Phase::Pulling)
                    drive(c.speedMetersPerSecond, d.targetAccelerationMetersPerSecondSquared);
                else if (train && c.antiRollback && speed <= followerRestSpeedToleranceMetersPerSecond && (chain->phase == ChainRuntime::Phase::Pulling || chain->phase == ChainRuntime::Phase::Engaging))
                { drive(0, d.targetAccelerationMetersPerSecondSquared); h.forceControl.armed = true; }
                h.state = !h.enabled || !chain->available ? "Unavailable" : chain->phase == ChainRuntime::Phase::Available ? "Available"
                    : chain->phase == ChainRuntime::Phase::Engaging ? "Engaging" : chain->phase == ChainRuntime::Phase::Pulling ? "Pulling" : "Releasing";
                continue;
            }
            if (!h.enabled || !train) continue;
            if (const auto* c = std::get_if<TransportConfiguration>(&d.configuration))
            { drive(c->speedMetersPerSecond * static_cast<int>(c->direction), d.targetAccelerationMetersPerSecondSquared); h.state = "Transporting"; }
            else if (d.kind == TrackDeviceKind::Launch)
            {
                const auto* c = std::get_if<LaunchConfiguration>(&d.configuration);
                if (c && c->targetSpeedMetersPerSecond) h.commandedSpeedMetersPerSecond = *c->targetSpeedMetersPerSecond * static_cast<int>(c->direction);
                const double directedSpeed = speed * (c ? static_cast<int>(c->direction) : 1);
                if (c && c->targetSpeedMetersPerSecond && directedSpeed >= *c->targetSpeedMetersPerSecond)
                { h.state = "Complete"; h.targetSpeedLimited = true; }
                else { h.forceControl.armed = true; h.state = "Launching"; h.commandedAccelerationMetersPerSecondSquared = profileCommand(d, train->referenceStationMeters, length, circuit); }
            }
            else if (d.kind == TrackDeviceKind::Brake)
            {
                const auto* c = std::get_if<BrakeConfiguration>(&d.configuration);
                const double target = c ? c->targetSpeedMetersPerSecond : 0;
                h.commandedSpeedMetersPerSecond = std::copysign(target, speed);
                if (c && c->holding && target == 0) { drive(0, profileCommand(d, train->referenceStationMeters, length, circuit)); h.state = "Holding"; }
                else if (std::abs(speed) > target + followerRestSpeedToleranceMetersPerSecond)
                { h.forceControl.armed = true; h.state = "Braking"; h.commandedAccelerationMetersPerSecondSquared = profileCommand(d, train->referenceStationMeters, length, circuit); }
                else h.state = "At target";
            }
            if (h.occupancy.trains.size() > 1) { h.forceControl.armed = false; h.state = "Interlocked"; h.interlockReason = "Multiple trains in command zone"; }
        }
        for (auto& h : hardware_)
            if (auto* gate = std::get_if<AccessRuntime>(&h.mechanism); gate && gate->protectedByOperation) h.state = "Interlocked";
    }
    std::vector<TrackDeviceRuntimeState> RideOperations::forceControls() const
    {
        std::vector<TrackDeviceRuntimeState> result; result.reserve(hardware_.size());
        for (const auto& h : hardware_) result.push_back(h.forceControl);
        return result;
    }
    void RideOperations::recordForces(const TrackDeviceForceResult& forces)
    {
        if (forces.devices.size() != hardware_.size()) throw std::invalid_argument("Hardware force telemetry is stale.");
        for (std::size_t i = 0; i < hardware_.size(); ++i)
        {
            const auto& force = forces.devices[i]; auto& h = hardware_[i];
            if (h.id != force.id) throw std::invalid_argument("Hardware force telemetry IDs are stale.");
            h.appliedForceNewtons = force.appliedForceNewtons;
            h.profileAccelerationMetersPerSecondSquared = force.profileAccelerationMetersPerSecondSquared;
            h.targetSpeedLimited = force.targetSpeedLimited
                || (h.targetSpeedLimited && force.profileAccelerationMetersPerSecondSquared > 0);
            if (!h.forceControl.signedAccelerationCommand)
                h.commandedAccelerationMetersPerSecondSquared = force.commandedAccelerationMetersPerSecondSquared;
        }
    }
}
