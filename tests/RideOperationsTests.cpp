#include <quantum/physics/RideOperations.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/RideHardwareSerialization.hpp>
#include <quantum/coaster/TrainConfiguration.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string_view>

namespace
{
    using namespace quantum;
    using namespace quantum::coaster;
    using namespace quantum::physics;
    int checks = 0;
    void require(bool value, const char* message) { ++checks; if (!value) throw std::runtime_error(message); }
    template<class F> void rejects(F f)
    {
        try { f(); } catch (const std::invalid_argument&) { ++checks; return; }
        throw std::runtime_error("Expected an invalid hardware operation to be rejected");
    }
    TrackDevice device(TrackDeviceKind kind, TrackDeviceId id, double start, double end)
    {
        auto d = makeTrackDevice(kind); d.id = id; d.startStationMeters = start; d.endStationMeters = end;
        d.targetAccelerationMetersPerSecondSquared = 4; d.maximumForceNewtons = 100000;
        return d;
    }
    const HardwareRuntime& state(const RideOperations& operations, TrackDeviceId id)
    {
        for (const auto& h : operations.hardware()) if (h.id == id) return h;
        throw std::runtime_error("Hardware runtime missing");
    }
    void sensorsAndInterlocks()
    {
        auto block = device(TrackDeviceKind::Block, 1, 90, 10);
        RideOperations operations; operations.reset(std::span(&block, 1));
        TrainObservation first{7, 95, -2, {{0, 0, 95}, {0, 1, 80}}};
        operations.step(std::span(&block, 1), std::span(&first, 1), 100, true);
        const auto& partial = state(operations, 1);
        require(partial.occupancy.bogies.size() == 1 && !partial.occupancy.trains[0].complete, "Wrapped partial bogie occupancy");
        require(partial.occupancy.entered == std::vector<OperationTrainId>{7} && partial.actualSpeedMetersPerSecond == -2, "Entry identity and rollback direction");
        first.bogies[1].stationMeters = 5;
        operations.step(std::span(&block, 1), std::span(&first, 1), 100, true);
        require(state(operations, 1).occupancy.trains[0].complete && state(operations, 1).occupancy.entered.empty(), "Complete occupancy and single entry event");
        TrainObservation second{8, 4, 2, {{3, 0, 4}, {3, 1, 3}}};
        const std::array trains{first, second};
        operations.step(std::span(&block, 1), trains, 100, true);
        require(state(operations, 1).occupancy.trains.size() == 2, "Block supports multiple train identities");
        operations.step(std::span(&block, 1), {}, 100, true);
        require(state(operations, 1).occupancy.clear() && state(operations, 1).occupancy.exited.size() == 2, "Clear/exit for both trains");

        auto station = device(TrackDeviceKind::Station, 2, 0, 30);
        std::get<StationConfiguration>(station.configuration) = {20, .15, .05, 2, 3, {4}, true};
        auto ahead = device(TrackDeviceKind::Block, 3, 30, 70);
        auto gate = device(TrackDeviceKind::Access, 4, 0, 2);
        std::vector hardware{station, ahead, gate}; operations.reset(hardware);
        TrainObservation parked{7, 20, 0, {{0, 0, 22}, {0, 1, 18}}};
        operations.step(hardware, std::span(&parked, 1), 100, false);
        require(std::get<StationRuntime>(state(operations, 2).mechanism).dispatchPermitted, "Station permits aligned/stopped train with closed gates and clear block");
        operations.command(4, HardwareCommand::UnlockAccess); operations.command(4, HardwareCommand::OpenAccess);
        operations.step(hardware, std::span(&parked, 1), 100, false, 1);
        operations.command(2, HardwareCommand::RequestDispatch);
        operations.step(hardware, std::span(&parked, 1), 100, false);
        require(!std::get<StationRuntime>(state(operations, 2).mechanism).dispatchPermitted && !state(operations, 2).interlockReason.empty(), "Open access gate blocks dispatch");
        rejects([&] { operations.command(4, HardwareCommand::LockAccess); });
        operations.command(4, HardwareCommand::CloseAccess); operations.step(hardware, std::span(&parked, 1), 100, false, 1);
        operations.command(4, HardwareCommand::LockAccess);
        TrainObservation obstruction{8, 40, 0, {{0, 0, 42}, {0, 1, 38}}};
        const std::array blocked{parked, obstruction};
        operations.step(hardware, blocked, 100, false);
        require(!std::get<StationRuntime>(state(operations, 2).mechanism).dispatchPermitted, "Occupied downstream block prevents dispatch");
        operations.step(hardware, std::span(&parked, 1), 100, false);
        require(state(operations, 2).state == "Leaving", "Pending dispatch starts after all permissives clear");
        rejects([&] { operations.command(4, HardwareCommand::UnlockAccess); });
        parked.referenceStationMeters = 29; parked.bogies = {{0, 0, 31}, {0, 1, 27}}; parked.signedSpeedMetersPerSecond = 2;
        operations.step(hardware, std::span(&parked, 1), 100, false);
        require(state(operations, 2).state == "Leaving", "Own departing train may enter downstream block");
        TrainObservation intruder{8, 20, 0, {{0, 0, 22}, {0, 1, 18}}};
        const std::array collision{parked, intruder};
        operations.step(hardware, collision, 100, false);
        require(!std::get<StationRuntime>(state(operations, 2).mechanism).dispatchPermitted && !state(operations, 2).forceControl.armed,
            "Multiple trains revoke station dispatch and drive");
        rejects([&] { operations.command(4, HardwareCommand::UnlockAccess); });
        operations.reset(hardware); operations.step(hardware, std::span(&parked, 1), 100, false, 0);
        require(!std::get<StationRuntime>(state(operations, 2).mechanism).dispatchRequested && std::get<AccessRuntime>(state(operations, 4).mechanism).locked, "Reset restores authored gates and clears dispatch");
    }
    struct Scenario
    {
        CompiledPhysicsTrack track;
        TrainDefinition train;
        TrainDynamicsState dynamics;
        TrainPose pose;
        std::vector<TrackDevice> hardware;
        RideOperations operations;
        std::set<std::pair<TrackDeviceId, std::string>> observed;
        static CompiledPhysicsTrack makeTrack(double slope = 0)
        {
            const glm::dvec3 tangent{std::cos(slope), 0, std::sin(slope)};
            const geometry::CurveFrame frame{tangent, {0, 1, 0}, {-std::sin(slope), 0, std::cos(slope)}};
            return {std::vector<TrackKinematicState>{{0, {0, 0, 0}, frame, {0, 0, 0}},
                {300, 300.0 * tangent, frame, {0, 0, 0}}}, 1, TopologyKind::OpenLinear};
        }
        static TrainDefinition makeTrain()
        { auto c = createDefaultTrainConfiguration(); c.carCount = 1; return resolveTrainConfiguration(c); }
        Scenario(std::vector<TrackDevice> devices, double station = 20, double speed = 0, double slope = 0)
            : track(makeTrack(slope)), train(makeTrain()), dynamics{{primaryTrackPathId, station, TravelDirection::IncreasingStation}, speed, 0, 0, FollowerRunState::Running},
              pose(solveTrainPose(track, train, dynamics.generalizedReferenceLocation)), hardware(std::move(devices))
        {
            TrackDeviceCollection collection; collection.devices = hardware;
            for (const auto& d : hardware) collection.nextId = std::max(collection.nextId, d.id + 1);
            validateTrackDevices(collection, 300, false);
            operations.reset(hardware); refresh(0);
        }
        void refresh(double seconds)
        {
            const auto observation = observeTrain(pose, dynamics.signedVelocityMetersPerSecond);
            operations.step(hardware, std::span(&observation, 1), 300, false, seconds);
        }
        void tick()
        {
            refresh(defaultFixedTimeStepSeconds);
            const auto forces = evaluateTrackDeviceForces(hardware, operations.forceControls(), track, train, pose, dynamics.signedVelocityMetersPerSecond);
            operations.recordForces(forces);
            for (const auto& h : operations.hardware())
            {
                observed.emplace(h.id, h.state);
                require(std::abs(h.appliedForceNewtons) <= hardware[&h - operations.hardware().data()].maximumForceNewtons + 1e-6, "Integrated device force cap");
            }
            auto result = stepTrain(track, train, {}, dynamics, {}, forces.applications);
            dynamics = result.state; pose = std::move(result.telemetry.pose);
        }
    };
    void targetSpeedComparisons()
    {
        // Only target changes between these runs. The same short device,
        // spatial Custom profile, loaded train and production solver are used.
        auto launch = device(TrackDeviceKind::Launch, 1, 30, 100);
        ChannelProfile profile;
        profile.nextSegmentId = 2;
        profile.segments.push_back({1, {0, 70, 2, 3, math::TransitionType::Linear}});
        launch.accelerationProfile = profile;
        for (const auto direction : {HardwareDirection::Forward, HardwareDirection::Reverse})
        {
            const int sign = static_cast<int>(direction);
            std::vector<double> exitSpeeds;
            for (const auto target : {std::optional<double>{}, std::optional<double>{10}, std::optional<double>{25}})
            {
                launch.configuration = LaunchConfiguration{target, direction};
                Scenario s({launch}, sign > 0 ? 40 : 90, sign * 5);
                bool exited = false;
                double maximumSpeed = 5;
                for (int tick = 0; tick < 6000; ++tick)
                {
                    s.tick();
                    const auto& h = state(s.operations, 1);
                    const double speed = sign * s.dynamics.signedVelocityMetersPerSecond;
                    maximumSpeed = std::max(maximumSpeed, speed);
                    require(sign * h.appliedForceNewtons >= 0, "Launch target must never command braking");
                    if (!target) require(!h.targetSpeedLimited, "Unset target never reports active limiting");
                    if (h.occupancy.clear()) { exited = true; break; }
                }
                require(exited, "Controlled launch train must exit the short interval");
                const double speed = sign * s.dynamics.signedVelocityMetersPerSecond;
                exitSpeeds.push_back(speed);
                if (target == 10)
                    require(maximumSpeed <= 10.02 && speed > 9.8, "10 m/s target constrains native speed in either direction");
                if (target == 25)
                    require(speed > 12 && speed < 25, "Insufficient distance may leave a launch below target");
                std::cout << "Launch comparison direction=" << sign << " target="
                    << (target ? std::to_string(*target) : "unset") << " exit=" << speed
                    << " max=" << maximumSpeed << " m/s\n";
            }
            require(exitSpeeds[0] > exitSpeeds[1] + 2, "Changing only target must change native train response");
            require(exitSpeeds[0] == exitSpeeds[2], "Unreached target preserves the same Custom propulsion trajectory");

            launch.configuration = LaunchConfiguration{3, direction};
            Scenario above({launch}, 60, sign * 5);
            for (int tick = 0; tick < 120; ++tick)
            {
                above.tick();
                require(state(above.operations, 1).appliedForceNewtons == 0,
                    "Below-current-speed target releases launch propulsion");
                require(sign * above.dynamics.signedVelocityMetersPerSecond > 3,
                    "Below-current-speed target does not assign velocity");
                require(state(above.operations, 1).targetSpeedLimited
                    && state(above.operations, 1).profileAccelerationMetersPerSecondSquared > 0,
                    "Telemetry retains the profile request when target releases propulsion");
            }

            // Inspect the final fraction of a fixed tick: profile is sampled at
            // each bogie, then constrained by directed speed error, then force.
            launch.configuration = LaunchConfiguration{10, direction};
            Scenario near({launch}, 60, sign * 9.999);
            auto forces = evaluateTrackDeviceForces(near.hardware, near.operations.forceControls(),
                near.track, near.train, near.pose, near.dynamics.signedVelocityMetersPerSecond);
            require(std::abs(forces.devices[0].commandedAccelerationMetersPerSecondSquared - .24) < 1e-9,
                "Target constrains Custom acceleration by remaining speed / fixed timestep");
            require(forces.devices[0].targetSpeedLimited && forces.devices[0].profileAccelerationMetersPerSecondSquared > 2,
                "Telemetry distinguishes profile request from target-limited command");
            require(near.hardware[0].accelerationProfile == profile, "Limiter leaves authored Custom profile intact");
            near.hardware[0].maximumForceNewtons = 100;
            forces = evaluateTrackDeviceForces(near.hardware, near.operations.forceControls(),
                near.track, near.train, near.pose, near.dynamics.signedVelocityMetersPerSecond);
            require(std::abs(forces.devices[0].appliedForceNewtons - sign * 100) < 1e-9,
                "Whole-device force cap still applies after target limiting");

            auto heavier = near.train;
            heavier.cars[0].loadout.massKilograms += 1000;
            const auto heavierPose = solveTrainPose(near.track, heavier, near.dynamics.generalizedReferenceLocation);
            const auto heavierForces = evaluateTrackDeviceForces(near.hardware, near.operations.forceControls(),
                near.track, heavier, heavierPose, near.dynamics.signedVelocityMetersPerSecond);
            require(std::abs(heavierForces.devices[0].appliedForceNewtons - sign * 100) < 1e-9,
                "Loaded mass cannot bypass the whole-device force cap");
            const auto lightStep = stepTrain(near.track, near.train, {}, near.dynamics, {}, forces.applications);
            const auto heavyStep = stepTrain(near.track, heavier, {}, near.dynamics, {}, heavierForces.applications);
            const auto lightCoast = stepTrain(near.track, near.train, {}, near.dynamics);
            const auto heavyCoast = stepTrain(near.track, heavier, {}, near.dynamics);
            require(sign * (heavyStep.state.signedVelocityMetersPerSecond - heavyCoast.state.signedVelocityMetersPerSecond)
                < sign * (lightStep.state.signedVelocityMetersPerSecond - lightCoast.state.signedVelocityMetersPerSecond),
                "The train solver retains loaded-mass authority under force saturation");

            Scenario partial({launch}, 99.9, sign * 9.999);
            partial.tick();
            require(state(partial.operations, 1).occupancy.bogies.size() == 1,
                "Target limiter operates with partial occupancy");
            require(sign * partial.dynamics.signedVelocityMetersPerSecond <= 10.02,
                "Partial occupancy does not bypass target limiting");

            launch.accelerationProfile.reset();
            Scenario constant({launch}, 60, sign * 9.999);
            constant.tick();
            require(state(constant.operations, 1).targetSpeedLimited
                && state(constant.operations, 1).profileAccelerationMetersPerSecondSquared == 4,
                "Constant acceleration uses the same target limiter");
            launch.accelerationProfile = profile;
            for (auto& segment : launch.accelerationProfile->segments)
                segment.transition.valueBegin = segment.transition.valueEnd = 0;
            Scenario zero({launch}, 60, sign * 5);
            zero.tick();
            require(state(zero.operations, 1).appliedForceNewtons == 0 && !state(zero.operations, 1).targetSpeedLimited,
                "A target does not replace a zero Custom profile with propulsion");
            launch.accelerationProfile = profile;
        }

        // Gravity can exceed a propulsion target. The launch is not a trim brake.
        launch.configuration = LaunchConfiguration{10, HardwareDirection::Forward};
        Scenario downhill({launch}, 60, 12, -.3);
        for (int tick = 0; tick < 120; ++tick)
        {
            downhill.tick();
            require(state(downhill.operations, 1).appliedForceNewtons == 0, "Above-target downhill launch supplies no force");
        }
        require(downhill.dynamics.signedVelocityMetersPerSecond > 12,
            "Gravity remains authoritative above a launch propulsion target");

        // Transport and lifts use the shared signed-error servo, not the
        // one-sided launch limiter. They can slow an overspeed train.
        for (const auto kind : {TrackDeviceKind::Transport, TrackDeviceKind::ChainLift, TrackDeviceKind::CableLift})
        {
            std::vector<double> speeds;
            for (const double target : {2., 6.})
            {
                auto d = device(kind, 1, 0, 200);
                if (kind == TrackDeviceKind::Transport) d.configuration = TransportConfiguration{target};
                if (kind == TrackDeviceKind::ChainLift) d.configuration = ChainLiftConfiguration{target};
                if (kind == TrackDeviceKind::CableLift) d.configuration = CableLiftConfiguration{target};
                Scenario s({d}, 20, 5);
                for (int tick = 0; tick < 480; ++tick) s.tick();
                speeds.push_back(s.dynamics.signedVelocityMetersPerSecond);
                require(std::abs(speeds.back() - target) < .08, "Transport/lift speed reaches native solver through bounded forces");
            }
            require(speeds[1] - speeds[0] > 3.8, "Transport/lift authored speed changes physical response");
        }
        auto reverseTires = device(TrackDeviceKind::Transport, 1, 0, 200);
        reverseTires.configuration = TransportConfiguration{2, HardwareDirection::Reverse};
        Scenario reverse({reverseTires}, 80, -5);
        for (int tick = 0; tick < 480; ++tick) reverse.tick();
        require(std::abs(reverse.dynamics.signedVelocityMetersPerSecond + 2) < .08,
            "Shared transport servo compares signed reverse speed correctly");
    }
    void liftScenario()
    {
        auto station = device(TrackDeviceKind::Station, 1, 0, 30);
        std::get<StationConfiguration>(station.configuration) = {20, .15, .05, 2, 6, {7}, true};
        auto tires = device(TrackDeviceKind::Transport, 2, 25, 45);
        auto chain = device(TrackDeviceKind::ChainLift, 3, 45, 75);
        auto trim = device(TrackDeviceKind::Brake, 4, 95, 115); std::get<BrakeConfiguration>(trim.configuration).targetSpeedMetersPerSecond = 2;
        auto arrival = device(TrackDeviceKind::Station, 5, 120, 160); std::get<StationConfiguration>(arrival.configuration).stopStationMeters = 140;
        auto block = device(TrackDeviceKind::Block, 6, 35, 95);
        auto gate = device(TrackDeviceKind::Access, 7, 0, 2);
        auto finalTires = device(TrackDeviceKind::Transport, 8, 105, 125);
        Scenario s({station, tires, chain, trim, arrival, block, gate, finalTires}, 20, 0, -.1);
        s.operations.command(1, HardwareCommand::RequestDispatch);
        bool arrived = false;
        for (int tick = 0; tick < 14000; ++tick)
        {
            s.tick(); const auto& a = state(s.operations, 5);
            if (a.aligned && a.stopped) { arrived = true; break; }
        }
        if (!arrived)
        {
            std::cerr << "Scenario A final station=" << s.dynamics.generalizedReferenceLocation.stationMeters << " speed=" << s.dynamics.signedVelocityMetersPerSecond << '\n';
            for (const auto& h : s.operations.hardware()) std::cerr << h.id << " " << h.state << " bogies=" << h.occupancy.bogies.size() << " command=" << h.commandedSpeedMetersPerSecond << " force=" << h.appliedForceNewtons << " reason=" << h.interlockReason << '\n';
        }
        require(arrived && std::abs(s.dynamics.generalizedReferenceLocation.stationMeters - 140) <= .15, "Scenario A native station arrival");
        require(s.observed.contains({2, "Transporting"}) && s.observed.contains({3, "Engaging"}) && s.observed.contains({3, "Pulling"})
            && s.observed.contains({3, "Releasing"}) && s.observed.contains({4, "Braking"}), "Scenario A transport/lift/brake transitions");
        // An inclined track proves lift commands reach gravity-aware train dynamics.
        Scenario incline({device(TrackDeviceKind::ChainLift, 1, 0, 100)}, 20, 0, .3);
        for (int i = 0; i < 300; ++i) incline.tick();
        require(incline.dynamics.signedVelocityMetersPerSecond > 3 && incline.dynamics.generalizedReferenceLocation.stationMeters > 20, "Chain lift pulls up a real incline");
        incline.operations.command(1, HardwareCommand::SetAvailable, 0); incline.refresh(defaultFixedTimeStepSeconds);
        require(state(incline.operations, 1).state == "Unavailable", "Unavailable chain reports state");
        incline.dynamics.signedVelocityMetersPerSecond = -2; incline.refresh(defaultFixedTimeStepSeconds);
        require(state(incline.operations, 1).forceControl.signedAccelerationCommand.value_or(0) > 0, "Engaged anti-rollback opposes rollback");
        std::cout << "Scenario A: native station/tires/chain/trim/station and inclined lift passed\n";
    }
    void launchScenario()
    {
        auto station = device(TrackDeviceKind::Station, 1, 0, 30); std::get<StationConfiguration>(station.configuration).stopStationMeters = 20;
        auto holding = device(TrackDeviceKind::Brake, 2, 32, 40); std::get<BrakeConfiguration>(holding.configuration).holding = true;
        auto launch = device(TrackDeviceKind::Launch, 3, 40, 80);
        std::get<LaunchConfiguration>(launch.configuration).targetSpeedMetersPerSecond = 10;
        ChannelProfile profile; profile.nextSegmentId = 2; profile.segments.push_back({1, {0, 40, 3, 4, math::TransitionType::Linear}}); launch.accelerationProfile = profile;
        auto brake = device(TrackDeviceKind::Brake, 4, 110, 135); std::get<BrakeConfiguration>(brake.configuration).targetSpeedMetersPerSecond = 2;
        auto arrival = device(TrackDeviceKind::Station, 5, 135, 170); std::get<StationConfiguration>(arrival.configuration).stopStationMeters = 150;
        auto releaseTires = device(TrackDeviceKind::Transport, 6, 32, 45); releaseTires.enabled = false;
        Scenario s({station, holding, launch, brake, arrival, releaseTires}, 20, 0, -.1); s.operations.command(1, HardwareCommand::RequestDispatch);
        bool released = false, arrived = false; double peak = 0;
        for (int i = 0; i < 18000; ++i)
        {
            s.tick(); peak = std::max(peak, s.dynamics.signedVelocityMetersPerSecond);
            if (!released && !state(s.operations, 2).occupancy.clear() && std::abs(s.dynamics.signedVelocityMetersPerSecond) < .05)
            { s.operations.command(2, HardwareCommand::SetEnabled, 0); s.operations.command(6, HardwareCommand::SetEnabled, 1); released = true; }
            if (state(s.operations, 5).aligned && state(s.operations, 5).stopped) { arrived = true; break; }
        }
        if (!released || peak < 9.9 || !arrived)
        {
            std::cerr << "Scenario B released=" << released << " peak=" << peak << " arrived=" << arrived << " station=" << s.dynamics.generalizedReferenceLocation.stationMeters << " speed=" << s.dynamics.signedVelocityMetersPerSecond << '\n';
            for (const auto& h : s.operations.hardware()) std::cerr << h.id << " " << h.state << " bogies=" << h.occupancy.bogies.size() << " force=" << h.appliedForceNewtons << '\n';
        }
        require(released && peak >= 9.9 && arrived, "Scenario B holding/release/custom launch/brake/station");
        auto rollback = device(TrackDeviceKind::Brake, 1, 0, 100);
        Scenario reverse({rollback}, 30, -5); const double before = reverse.dynamics.signedVelocityMetersPerSecond; reverse.tick();
        require(reverse.dynamics.signedVelocityMetersPerSecond > before && state(reverse.operations, 1).appliedForceNewtons > 0, "Rollback brake reaches real physics and opposes motion");
        auto disabled = launch; disabled.enabled = false; Scenario noLaunch({disabled}, 60, 2); noLaunch.tick();
        require(state(noLaunch.operations, 3).appliedForceNewtons == 0 && !state(noLaunch.operations, 3).occupancy.clear(), "Disabled launch still senses occupancy");
        auto weak = launch; weak.maximumForceNewtons = 1; Scenario limited({weak}, 60, 2); limited.tick();
        require(std::abs(state(limited.operations, 3).appliedForceNewtons - 1) < 1e-9 && limited.dynamics.signedVelocityMetersPerSecond < 2.001, "Insufficient force produces limited physical response");
        auto reverseTires = device(TrackDeviceKind::Transport, 1, 0, 100);
        std::get<TransportConfiguration>(reverseTires.configuration).direction = HardwareDirection::Reverse;
        Scenario reverseTransport({reverseTires}, 30, 0); reverseTransport.tick();
        require(state(reverseTransport.operations, 1).appliedForceNewtons < 0 && reverseTransport.dynamics.signedVelocityMetersPerSecond < 0,
            "Reverse transport direction reaches native train forces");
        auto reverseLaunch = device(TrackDeviceKind::Launch, 1, 0, 100);
        std::get<LaunchConfiguration>(reverseLaunch.configuration).direction = HardwareDirection::Reverse;
        Scenario reversePowered({reverseLaunch}, 30, 0); reversePowered.tick();
        require(state(reversePowered.operations, 1).appliedForceNewtons < 0 && reversePowered.dynamics.signedVelocityMetersPerSecond < 0,
            "Reverse launch direction reaches native train forces");
        std::cout << "Scenario B: hold/release/custom launch/brake/arrival, rollback and force caps passed\n";
    }
    void controlledCableComparison()
    {
        auto cable = device(TrackDeviceKind::CableLift, 1, 27, 150);
        std::get<CableLiftConfiguration>(cable.configuration) = {10, .5, true};
        cable.maximumForceNewtons = 1000000;
        ChannelProfile profile; profile.nextSegmentId = 2;
        profile.segments.push_back({1, {0, 123, 2, 3, math::TransitionType::Linear}});
        cable.accelerationProfile = profile;
        auto off = cable; off.enabled = false;
        Scenario enabled({cable}, 20, 6), disabled({off}, 20, 6);
        // Build the same straight track from three actual connected authored
        // Regions. The device interval crosses the 100 m Region boundary.
        auto document = createNewDocument(); document.setLayoutMode(LayoutMode::Shuttle);
        while (document.sectionCount() > 1) document.removeSection(document.sectionCount() - 1);
        document.section(0) = createRateProfileSection(100);
        document.appendSection(); document.section(1) = createRateProfileSection(100);
        document.appendSection(); document.section(2) = createRateProfileSection(100);
        const auto samples = integrateAuthoredTrackKinematics(document, .5);
        enabled.track = CompiledPhysicsTrack(samples, 1, TopologyKind::OpenLinear);
        disabled.track = enabled.track;
        bool entered = false, exited = false, boundaryDriven = false;
        double maximumApplied = 0, maximumEnabledSpeed = 6, maximumDisabledSpeed = 6;
        double exitSpeed = 0, baselineSpeedAtExit = 0;
        for (int tick = 0; tick < 8400; ++tick)
        {
            enabled.tick(); disabled.tick();
            const auto& h = state(enabled.operations, 1);
            entered |= !h.occupancy.entered.empty(); exited |= !h.occupancy.exited.empty();
            maximumApplied = std::max(maximumApplied, h.appliedForceNewtons);
            maximumEnabledSpeed = std::max(maximumEnabledSpeed, enabled.dynamics.signedVelocityMetersPerSecond);
            maximumDisabledSpeed = std::max(maximumDisabledSpeed, disabled.dynamics.signedVelocityMetersPerSecond);
            if (!h.occupancy.exited.empty())
            {
                exitSpeed = enabled.dynamics.signedVelocityMetersPerSecond;
                baselineSpeedAtExit = disabled.dynamics.signedVelocityMetersPerSecond;
            }
            if (enabled.dynamics.generalizedReferenceLocation.stationMeters > 98
                && enabled.dynamics.generalizedReferenceLocation.stationMeters < 102)
                boundaryDriven |= h.appliedForceNewtons != 0 || h.forceControl.armed;
            require(state(disabled.operations, 1).appliedForceNewtons == 0,
                "Disabled CableLift must never apply drive force");
        }
        std::cout << "Cable reproduction evidence: entered=" << entered << " exited=" << exited
            << " boundaryDriven=" << boundaryDriven << " maxForce=" << maximumApplied
            << " station=" << enabled.dynamics.generalizedReferenceLocation.stationMeters
            << " speed=" << enabled.dynamics.signedVelocityMetersPerSecond << " state="
            << state(enabled.operations, 1).state << "\n";
        require(entered && exited && boundaryDriven && maximumApplied > 100,
            "Cable custom profile enters, drives across Region boundary, then exits actual train occupancy");
        require(maximumEnabledSpeed > maximumDisabledSpeed + 2.5 && exitSpeed > baselineSpeedAtExit + 5,
            "Enabled CableLift changes actual native train speed during drive and at exit compared with identical disabled run");
        require(enabled.dynamics.generalizedReferenceLocation.stationMeters
            > disabled.dynamics.generalizedReferenceLocation.stationMeters + 50,
            "Enabled CableLift changes actual native travel, not only telemetry");
        Scenario toggled({cable}, 50, 2);
        for (int i = 0; i < 100; ++i) toggled.tick();
        require(state(toggled.operations, 1).appliedForceNewtons > 0, "Occupied cable initially drives");
        toggled.operations.command(1, HardwareCommand::SetEnabled, 0); toggled.tick();
        require(state(toggled.operations, 1).appliedForceNewtons == 0, "Runtime disabled cable releases drive next fixed tick");
        toggled.operations.command(1, HardwareCommand::SetEnabled, 1); toggled.tick();
        require(state(toggled.operations, 1).appliedForceNewtons > 0, "Runtime re-enabled cable resumes engaged drive");
        Scenario reverse({cable}, 50, -2); reverse.tick(); reverse.tick();
        require(state(reverse.operations, 1).state == "Available"
            && state(reverse.operations, 1).appliedForceNewtons == 0,
            "Reverse entry cannot acquire a forward-only cable catch car");
        const auto id = document.addTrackDevice(cable);
        const auto serialized = serializeCoasterDocument(document);
        const auto reopened = deserializeCoasterDocument(serialized);
        require(reopened && serializeCoasterDocument(*reopened) == serialized
            && reopened->trackDevices().devices.front().id == id,
            "Controlled CableLift configuration/profile/enabled state survives save and reopen");
        std::cout << "Controlled CableLift: peak enabled speed=" << maximumEnabledSpeed
            << " peak disabled speed=" << maximumDisabledSpeed << " exit speed=" << exitSpeed
            << " baseline speed at same tick=" << baselineSpeedAtExit
            << " max force=" << maximumApplied << " N; entry/exit/Region/profile/toggle/reverse/persistence passed\n";
    }
    void cableAndTransfer()
    {
        auto cable = device(TrackDeviceKind::CableLift, 1, 10, 35);
        auto& config = std::get<CableLiftConfiguration>(cable.configuration); config.returnSeconds = .5; config.initiallyAvailable = false;
        Scenario s({cable}, 20, 0); s.tick(); require(state(s.operations, 1).state == "Unavailable", "Catch-car availability interlock");
        s.operations.command(1, HardwareCommand::SetAvailable, 1);
        for (int i = 0; i < 1500; ++i) s.tick();
        require(s.observed.contains({1, "Engaging"}) && s.observed.contains({1, "Pulling"}) && s.observed.contains({1, "Releasing"})
            && s.observed.contains({1, "Returning"}) && state(s.operations, 1).state == "Available", "Scenario C real cable pull and catch return cycle");
        auto transfer = device(TrackDeviceKind::Transfer, 1, 10, 30); auto& c = std::get<TransferConfiguration>(transfer.configuration);
        c.movementSpeedMetersPerSecond = 4; c.accessGates = {2};
        auto gate = device(TrackDeviceKind::Access, 2, 0, 2); std::get<AccessConfiguration>(gate.configuration).motion = AccessMotion::Slide;
        Scenario t({transfer, gate}, 20, 2);
        rejects([&] { t.operations.command(1, HardwareCommand::MoveTransfer, 0, 2); });
        for (int i = 0; i < 200; ++i) t.tick();
        require(state(t.operations, 1).movementPermitted, "Transfer physically secures train");
        rejects([&] { t.operations.command(1, HardwareCommand::MoveTransfer, 0, 999); });
        t.operations.command(1, HardwareCommand::MoveTransfer, 0, 2); t.tick();
        rejects([&] { t.operations.command(2, HardwareCommand::UnlockAccess); });
        rejects([&] { t.operations.command(1, HardwareCommand::MoveTransfer, 0, 1); });
        for (int i = 0; i < 300; ++i) t.tick();
        const auto& done = std::get<TransferRuntime>(state(t.operations, 1).mechanism);
        require(done.phase == TransferRuntime::Phase::Locked && done.alignment == 2 && done.positionMeters == 4, "Scenario D actual native movement/alignment/lock");
        require(t.dynamics.generalizedReferenceLocation.path == primaryTrackPathId, "Transfer proof does not pretend alternate track traversal");
        t.operations.command(2, HardwareCommand::UnlockAccess); t.operations.command(2, HardwareCommand::OpenAccess);
        t.refresh(1); require(std::get<AccessRuntime>(state(t.operations, 2).mechanism).fraction == 1, "Sliding access gate runtime opens");
        rejects([&] { t.operations.command(1, HardwareCommand::MoveTransfer, 0, 1); });
        t.operations.reset(t.hardware); t.refresh(0);
        require(std::get<TransferRuntime>(state(t.operations, 1).mechanism).alignment == 1 && std::get<AccessRuntime>(state(t.operations, 2).mechanism).locked, "Reset homes mechanism and restores gate lock");
        std::cout << "Scenarios C/D: cable cycle and transfer/storage/gate interlocks passed; alternate routing remains pending\n";
    }
    void persistenceAndPerformance()
    {
        auto track = createNewDocument(); auto collection = track.trackDevices();
        const double length = track.trackLengthMeters();
        for (unsigned kind = 0; kind <= static_cast<unsigned>(TrackDeviceKind::Access); ++kind)
        {
            auto d = device(static_cast<TrackDeviceKind>(kind), ++collection.nextId, 0, length * .75);
            if (auto* station = std::get_if<StationConfiguration>(&d.configuration)) station->stopStationMeters = length * .5;
            collection.devices.push_back(d);
        }
        ++collection.nextId;
        TrackInfrastructure catwalk; catwalk.id = collection.nextInfrastructureId++; catwalk.name = "Lift access";
        catwalk.startStationMeters = 0; catwalk.endStationMeters = length * .75; catwalk.accessGates = {10};
        collection.infrastructure.push_back(catwalk); track.setTrackDevices(collection);
        const auto text = serializeCoasterDocument(track); auto loaded = deserializeCoasterDocument(text);
        require(loaded && loaded->trackDevices() == collection, "All typed configs and infrastructure roundtrip");
        auto json = nlohmann::json::parse(text); json["trackDevices"]["schemaVersion"] = 9;
        require(!deserializeCoasterDocument(json.dump()), "Reject unknown hardware schema");
        json = nlohmann::json::parse(text); json["trackDevices"]["devices"][2]["configuration"]["futureField"] = 1;
        require(!deserializeCoasterDocument(json.dump()), "Reject unknown typed configuration field");
        json = nlohmann::json::parse(text);
        json["trackDevices"]["devices"][7]["configuration"].erase("nextAlignmentId");
        const auto legacyAlignments = deserializeCoasterDocument(json.dump());
        require(legacyAlignments && std::get<TransferConfiguration>(legacyAlignments->trackDevices().devices[7].configuration).nextAlignmentId == 3,
            "Missing alignment allocator migrates without changing alignment IDs");
        auto invalid = collection; invalid.devices[3].configuration = AccessConfiguration{};
        rejects([&] { track.setTrackDevices(invalid); });
        invalid = collection; invalid.infrastructure[0].widthMeters = -1; rejects([&] { track.setTrackDevices(invalid); });
        invalid = collection; invalid.infrastructure[0].accessGates = {999}; rejects([&] { track.setTrackDevices(invalid); });
        TrainObservation observation{1, 20, 2, {{0, 0, 21}, {0, 1, 19}}};
        for (int count : {10, 50, 100})
        {
            std::vector<TrackDevice> hardware;
            for (int i = 0; i < count; ++i) hardware.push_back(device(TrackDeviceKind::Block, i + 1, 10, 30));
            RideOperations operations; operations.reset(hardware);
            const auto begin = std::chrono::steady_clock::now();
            for (int i = 0; i < 1000; ++i) operations.step(hardware, std::span(&observation, 1), 100, false);
            const double milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count() / 1000;
            std::cout << count << " occupied hardware: " << milliseconds << " ms/operations tick (no train solver)\n";
        }
    }
}
int main(int argc, char** argv)
{
    try {
        if (argc == 2 && std::string_view(argv[1]) == "--target-speed") targetSpeedComparisons();
        else { sensorsAndInterlocks(); liftScenario(); launchScenario(); cableAndTransfer(); controlledCableComparison(); persistenceAndPerformance(); }
        std::cout << "RideOperationsTests passed " << checks << " checks\n"; return 0; }
    catch (const std::exception& e) { std::cerr << "RideOperationsTests failed: " << e.what() << '\n'; return 1; }
}
