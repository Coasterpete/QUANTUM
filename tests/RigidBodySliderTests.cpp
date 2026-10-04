#include <quantum/physics/RigidBodyWorld.hpp>
#include <quantum/physics/TrackFollower.hpp>

#include <glm/geometric.hpp>
#include <glm/vec2.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
    using namespace quantum::physics;

    void require(const bool condition, const std::string& message)
    {
        if (!condition) throw std::runtime_error(message);
    }

    template<class Function>
    void requireInvalidInput(Function function, const std::string& message)
    {
        try
        {
            function();
        }
        catch (const std::invalid_argument&)
        {
            return;
        }
        throw std::runtime_error(message);
    }

    void advance(RigidBodyWorld& world, const int ticks)
    {
        for (int tick = 0; tick < ticks; ++tick) world.stepFixed();
    }

    void requireFinite(const RigidBodyState& body, const RigidBodySliderState& slider)
    {
        require(std::isfinite(glm::length(body.positionMeters))
            && std::isfinite(glm::length(body.linearVelocityMetersPerSecond))
            && std::isfinite(glm::length(body.angularVelocityRadiansPerSecond))
            && std::isfinite(glm::length(body.orientation))
            && std::isfinite(slider.translationMeters)
            && std::isfinite(slider.linearVelocityMetersPerSecond), "finite slider state");
        require(std::abs(glm::length(body.orientation) - 1.0) < 1.0e-5,
            "unit slider orientation");
    }

    [[nodiscard]] double angularDrift(const glm::dquat& actual, const glm::dquat& initial)
    {
        return 2.0 * std::acos(std::clamp(std::abs(glm::dot(
            glm::normalize(actual), glm::normalize(initial))), 0.0, 1.0));
    }

    struct CarriageSample
    {
        RigidBodyState body;
        RigidBodySliderState slider;
    };

    [[nodiscard]] std::vector<CarriageSample> carriageTrajectory()
    {
        RigidBodyWorld world;
        RigidBodyBoxSettings box;
        box.positionMeters = {2.0, 3.0, 2.0};
        box.massKilograms = 10.0;
        box.orientation = glm::angleAxis(0.3, glm::normalize(glm::dvec3{1.0, 2.0, 3.0}));
        const auto body = world.createBox(box);
        RigidBodySliderSettings settings{body, std::nullopt,
            box.positionMeters, {0.0, 0.0, 3.0}, -0.25, 4.75};
        const auto slider = world.createSlider(settings);
        require(std::abs(world.sliderState(slider).translationMeters) < 1.0e-6,
            "shared creation anchor defines zero translation");
        std::vector<CarriageSample> trajectory;
        trajectory.reserve(24720);
        double maximumOffAxisDrift = 0.0;
        double maximumAngularDrift = 0.0;
        double maximumOffAxisSpeed = 0.0;
        double maximumAngularSpeed = 0.0;
        double lowerOvershoot = 0.0;
        double upperOvershoot = 0.0;
        double maximumSpeedError = 0.0;
        int speedSamples = 0;
        int upperVisits = 0;
        int lowerVisits = 0;
        const auto sample = [&]
        {
            const auto state = world.bodyState(body);
            const auto axial = world.sliderState(slider);
            requireFinite(state, axial);
            maximumOffAxisDrift = std::max(maximumOffAxisDrift,
                glm::length(glm::dvec3{state.positionMeters.x - box.positionMeters.x,
                    state.positionMeters.y - box.positionMeters.y, 0.0}));
            maximumAngularDrift = std::max(maximumAngularDrift,
                angularDrift(state.orientation, box.orientation));
            maximumOffAxisSpeed = std::max(maximumOffAxisSpeed,
                glm::length(glm::dvec3{state.linearVelocityMetersPerSecond.x,
                    state.linearVelocityMetersPerSecond.y, 0.0}));
            maximumAngularSpeed = std::max(maximumAngularSpeed,
                glm::length(state.angularVelocityRadiansPerSecond));
            lowerOvershoot = std::max(lowerOvershoot, -0.25 - axial.translationMeters);
            upperOvershoot = std::max(upperOvershoot, axial.translationMeters - 4.75);
            require(maximumOffAxisDrift < 0.02, "carriage stays on vertical guide within 2 cm");
            require(maximumAngularDrift < 1.0e-4,
                "carriage preserves creation orientation: tick=" + std::to_string(world.tick())
                    + " drift=" + std::to_string(maximumAngularDrift));
            require(maximumOffAxisSpeed < 0.01 && maximumAngularSpeed < 0.01,
                "carriage suppresses unwanted linear/angular velocity");
            require(lowerOvershoot < 0.02 && upperOvershoot < 0.02,
                "carriage respects both limits within 2 cm");
            require(std::abs(axial.translationMeters - (state.positionMeters.z - 2.0)) < 0.001,
                "translation query follows body displacement from creation pose");
            require(std::abs(axial.linearVelocityMetersPerSecond
                - state.linearVelocityMetersPerSecond.z) < 0.01, "world slider axial speed query");
            trajectory.push_back({state, axial});
        };
        // Four 24-second cycles: travel and push against each stop for 12 seconds.
        for (int tick = 0; tick < 23040; ++tick)
        {
            const int phase = tick % 5760;
            if (phase == 0) world.setSliderMotor(slider, {true, 1.5, 200.0});
            if (phase == 2880) world.setSliderMotor(slider, {true, -1.5, 200.0});
            world.stepFixed();
            sample();
            const auto& state = trajectory.back().slider;
            if (phase == 2879)
            {
                require(std::abs(state.translationMeters - 4.75) < 0.02, "carriage reaches upper stop");
                ++upperVisits;
            }
            if (phase == 5759)
            {
                require(std::abs(state.translationMeters + 0.25) < 0.02, "carriage reaches lower stop");
                ++lowerVisits;
            }
            if (phase % 2880 >= 240 && state.translationMeters > -0.1
                && state.translationMeters < 4.6)
            {
                const double target = phase < 2880 ? 1.5 : -1.5;
                maximumSpeedError = std::max(maximumSpeedError,
                    std::abs(state.linearVelocityMetersPerSecond - target));
                require(maximumSpeedError < 0.05, "carriage reaches signed target away from stops");
                ++speedSamples;
            }
        }
        require(speedSamples > 1000 && upperVisits == 4 && lowerVisits == 4,
            "both directions and all repeated limit visits were exercised");
        world.setSliderMotor(slider, {true, 1.5, 200.0});
        for (int tick = 0; tick < 480; ++tick) { world.stepFixed(); sample(); }
        world.setSliderMotor(slider, {true, 0.0, 200.0});
        double heldTranslation = 0.0;
        double maximumHoldDrift = 0.0;
        double maximumHoldSpeed = 0.0;
        for (int tick = 0; tick < 480; ++tick)
        {
            world.stepFixed();
            sample();
            const auto& state = trajectory.back().slider;
            if (tick == 120) heldTranslation = state.translationMeters;
            if (tick >= 120)
            {
                maximumHoldDrift = std::max(maximumHoldDrift,
                    std::abs(state.translationMeters - heldTranslation));
                maximumHoldSpeed = std::max(maximumHoldSpeed,
                    std::abs(state.linearVelocityMetersPerSecond));
            }
        }
        require(maximumHoldDrift < 0.01 && maximumHoldSpeed < 0.01,
            "zero-speed drive holds a free interior position against gravity");
        require(heldTranslation > 1.0 && heldTranslation < 4.0, "hold occurs away from limits");
        world.setSliderMotor(slider, {false, 1.5, 200.0});
        double passiveSpeed = 0.0;
        for (int tick = 0; tick < 720; ++tick)
        {
            world.stepFixed();
            sample();
            if (tick == 119) passiveSpeed = trajectory.back().slider.linearVelocityMetersPerSecond;
        }
        require(passiveSpeed < -3.0, "disabled drive ignores upward target and falls under gravity");
        require(std::abs(trajectory.back().slider.translationMeters + 0.25) < 0.02,
            "passive carriage settles at lower limit");
        require(world.tick() == trajectory.size(), "one sample per existing fixed tick");
        std::cout << "Carriage: ticks=" << world.tick() << " seconds=103"
            << " maxOffAxisDrift=" << maximumOffAxisDrift
            << " maxAngularDrift=" << maximumAngularDrift
            << " maxOffAxisSpeed=" << maximumOffAxisSpeed
            << " maxAngularSpeed=" << maximumAngularSpeed
            << " lowerOvershoot=" << lowerOvershoot << " upperOvershoot=" << upperOvershoot
            << " maxSpeedError=" << maximumSpeedError << " speedSamples=" << speedSamples
            << " maxHoldDrift=" << maximumHoldDrift << " maxHoldSpeed=" << maximumHoldSpeed
            << " passiveSpeedAt0.5s=" << passiveSpeed << '\n';
        return trajectory;
    }

    void carriageAndRepeatability()
    {
        const auto first = carriageTrajectory();
        const auto second = carriageTrajectory();
        require(first.size() == second.size(), "matching trajectory lengths");
        double maximumPositionDifference = 0.0;
        double maximumVelocityDifference = 0.0;
        double maximumTranslationDifference = 0.0;
        double maximumAxialVelocityDifference = 0.0;
        for (std::size_t index = 0; index < first.size(); ++index)
        {
            maximumPositionDifference = std::max(maximumPositionDifference,
                glm::length(first[index].body.positionMeters - second[index].body.positionMeters));
            maximumVelocityDifference = std::max(maximumVelocityDifference,
                glm::length(first[index].body.linearVelocityMetersPerSecond
                    - second[index].body.linearVelocityMetersPerSecond));
            maximumTranslationDifference = std::max(maximumTranslationDifference,
                std::abs(first[index].slider.translationMeters - second[index].slider.translationMeters));
            maximumAxialVelocityDifference = std::max(maximumAxialVelocityDifference,
                std::abs(first[index].slider.linearVelocityMetersPerSecond
                    - second[index].slider.linearVelocityMetersPerSecond));
            require(maximumPositionDifference <= 1.0e-6 && maximumVelocityDifference <= 1.0e-6
                && maximumTranslationDifference <= 1.0e-6 && maximumAxialVelocityDifference <= 1.0e-6
                && std::abs(glm::dot(first[index].body.orientation, second[index].body.orientation)
                    - 1.0) <= 1.0e-6
                && glm::length(first[index].body.angularVelocityRadiansPerSecond
                    - second[index].body.angularVelocityRadiansPerSecond) <= 1.0e-6
                && first[index].body.active == second[index].body.active,
                "same executable/configuration carriage trajectory repeats within tolerance");
        }
        std::cout << "Carriage repeatability: maxPositionDifference=" << maximumPositionDifference
            << " maxVelocityDifference=" << maximumVelocityDifference
            << " maxTranslationDifference=" << maximumTranslationDifference
            << " maxAxialVelocityDifference=" << maximumAxialVelocityDifference << '\n';
    }

    void horizontalActuatorForceDisableAndWake()
    {
        RigidBodyWorld world;
        RigidBodyBoxSettings box;
        box.positionMeters = {0.0, 2.0, 5.0};
        box.massKilograms = 2.0;
        const auto body = world.createBox(box);
        const auto slider = world.createSlider(
            {body, std::nullopt, box.positionMeters, {2.0, 0.0, 0.0}, -2.0, 2.0});
        world.setSliderMotor(slider, {true, 1.0, 0.0});
        advance(world, 2400);
        require(!world.bodyState(body).active, "undriven horizontal carriage sleeps");
        require(std::abs(world.sliderState(slider).translationMeters) < 1.0e-5,
            "zero force cannot start horizontal motion");
        world.setSliderMotor(slider, {true, 10.0, 2.0});
        require(world.bodyState(body).active, "drive change wakes sleeping carriage immediately");
        world.stepFixed();
        const double limitedSpeed = world.sliderState(slider).linearVelocityMetersPerSecond;
        const double expectedSpeed = (2.0 / box.massKilograms) * defaultFixedTimeStepSeconds;
        require(std::abs(limitedSpeed - expectedSpeed) < 1.0e-6,
            "finite force budget limits first-tick acceleration to F/m");
        world.setSliderMotor(slider, {true, 1.0, 100.0});
        advance(world, 120);
        const double drivenSpeed = world.sliderState(slider).linearVelocityMetersPerSecond;
        require(std::abs(drivenSpeed - 1.0) < 0.05, "positive horizontal target");
        world.setSliderMotor(slider, {false, -1.0, 100.0});
        advance(world, 120);
        const double disabledSpeed = world.sliderState(slider).linearVelocityMetersPerSecond;
        require(disabledSpeed > 0.9 && disabledSpeed < drivenSpeed,
            "disabled drive preserves forward damped motion and ignores reverse target");
        double maximumDrift = 0.0;
        double maximumRotation = 0.0;
        double overshoot = 0.0;
        double maximumSpeedError = 0.0;
        world.setSliderMotor(slider, {true, 1.0, 100.0});
        for (int tick = 0; tick < 2400; ++tick)
        {
            if (tick == 1200) world.setSliderMotor(slider, {true, -1.0, 100.0});
            world.stepFixed();
            const auto state = world.bodyState(body);
            const auto axial = world.sliderState(slider);
            requireFinite(state, axial);
            maximumDrift = std::max(maximumDrift,
                glm::length(glm::dvec3{0.0, state.positionMeters.y - 2.0, state.positionMeters.z - 5.0}));
            maximumRotation = std::max(maximumRotation, angularDrift(state.orientation, box.orientation));
            overshoot = std::max(overshoot, std::abs(axial.translationMeters) - 2.0);
            require(maximumDrift < 0.02 && maximumRotation < 1.0e-4 && overshoot < 0.02,
                "X guide suppresses gravity/rotation and respects both stops");
            if (tick % 1200 >= 240 && std::abs(axial.translationMeters) < 1.9)
            {
                maximumSpeedError = std::max(maximumSpeedError,
                    std::abs(axial.linearVelocityMetersPerSecond - (tick < 1200 ? 1.0 : -1.0)));
                require(maximumSpeedError < 0.05, "signed horizontal speed away from stops");
            }
            if (tick == 1199) require(std::abs(axial.translationMeters - 2.0) < 0.02, "X upper stop reached");
        }
        require(std::abs(world.sliderState(slider).translationMeters + 2.0) < 0.02, "X lower stop reached");
        // An insufficient upward force cannot hold a 10 kg vertical mass.
        RigidBodyBoxSettings heavy;
        heavy.positionMeters = {10.0, 0.0, 5.0};
        heavy.massKilograms = 10.0;
        const auto falling = world.createBox(heavy);
        const auto vertical = world.createSlider(
            {falling, std::nullopt, heavy.positionMeters, {0.0, 0.0, 1.0}, std::nullopt, std::nullopt});
        world.setSliderMotor(vertical, {true, 0.0, 50.0});
        advance(world, 120);
        const double insufficientForceSpeed = world.sliderState(vertical).linearVelocityMetersPerSecond;
        require(insufficientForceSpeed < -2.0, "zero target holds only within available force budget");
        std::cout << "Horizontal: firstTickSpeedAt2N=" << limitedSpeed
            << " drivenSpeed=" << drivenSpeed << " disabledSpeed=" << disabledSpeed
            << " maxOffAxisDrift=" << maximumDrift << " maxAngularDrift=" << maximumRotation
            << " maxOvershoot=" << overshoot << " maxSpeedError=" << maximumSpeedError
            << " verticalSpeedAt50N=" << insufficientForceSpeed << '\n';
    }

    void twoBodySlider()
    {
        RigidBodyWorld world;
        RigidBodyBoxSettings box;
        box.halfExtentsMeters = {0.1, 0.1, 0.1};
        box.positionMeters = {0.0, 0.0, 5.0};
        box.massKilograms = 4.0;
        const auto parent = world.createBox(box);
        // Separation along the slide axis keeps colliders apart throughout
        // travel and avoids introducing an eccentric stop-impact load.
        box.positionMeters = {2.0, 0.0, 5.0};
        box.massKilograms = 1.0;
        const auto child = world.createBox(box);
        const auto slider = world.createSlider(
            {child, parent, {1.0, 0.0, 5.0}, {1.0, 0.0, 0.0}, -1.0, 1.0});
        world.setSliderMotor(slider, {true, 1.0, 50.0});
        double maximumDrift = 0.0;
        double maximumRotation = 0.0;
        double maximumSpeedError = 0.0;
        double overshoot = 0.0;
        for (int tick = 0; tick < 960; ++tick)
        {
            if (tick == 480) world.setSliderMotor(slider, {true, -1.0, 50.0});
            world.stepFixed();
            const auto first = world.bodyState(parent);
            const auto second = world.bodyState(child);
            const auto axial = world.sliderState(slider);
            requireFinite(first, axial);
            requireFinite(second, axial);
            const auto relative = glm::conjugate(first.orientation)
                * (second.positionMeters - first.positionMeters);
            maximumDrift = std::max(maximumDrift,
                glm::length(glm::dvec3{0.0, relative.y, relative.z}));
            maximumRotation = std::max(maximumRotation, angularDrift(second.orientation, first.orientation));
            overshoot = std::max(overshoot, std::abs(axial.translationMeters) - 1.0);
            require(maximumDrift < 0.02 && maximumRotation < 1.0e-4 && overshoot < 0.02,
                "two-body slider preserves relative off-axis position/orientation and limits: tick="
                    + std::to_string(tick) + " drift=" + std::to_string(maximumDrift)
                    + " rotation=" + std::to_string(maximumRotation)
                    + " overshoot=" + std::to_string(overshoot));
            require(std::abs(axial.translationMeters - (relative.x - 2.0)) < 0.001,
                "two-body translation follows moving reference frame");
            const auto axis = first.orientation * glm::dvec3{1.0, 0.0, 0.0};
            const auto attachment = second.positionMeters
                + second.orientation * glm::dvec3{-1.0, 0.0, 0.0};
            const auto pointVelocity1 = first.linearVelocityMetersPerSecond
                + glm::cross(first.angularVelocityRadiansPerSecond, attachment - first.positionMeters);
            const auto pointVelocity2 = second.linearVelocityMetersPerSecond
                + glm::cross(second.angularVelocityRadiansPerSecond, attachment - second.positionMeters);
            require(std::abs(axial.linearVelocityMetersPerSecond
                - glm::dot(pointVelocity2 - pointVelocity1, axis)) < 1.0e-5,
                "two-body speed includes attachment and reference-frame rotation");
            if (tick % 480 >= 120 && std::abs(axial.translationMeters) < 0.9)
            {
                maximumSpeedError = std::max(maximumSpeedError,
                    std::abs(axial.linearVelocityMetersPerSecond - (tick < 480 ? 1.0 : -1.0)));
                require(maximumSpeedError < 0.05, "two-body motor drives relative signed velocity");
            }
        }
        require(world.bodyState(parent).positionMeters.z < 0.0,
            "two-body slider leaves the assembly free to fall");
        world.removeBody(parent);
        requireInvalidInput([&] { (void)world.sliderState(slider); }, "removed parent invalidates slider");
        advance(world, 240);
        require(std::isfinite(glm::length(world.bodyState(child).positionMeters)), "surviving child remains usable");
        std::cout << "Two-body slider: maxOffAxisDrift=" << maximumDrift
            << " maxAngularDrift=" << maximumRotation << " maxSpeedError=" << maximumSpeedError
            << " maxOvershoot=" << overshoot << '\n';
    }

    void validationAndLifetime()
    {
        RigidBodyWorld world;
        RigidBodyWorld other;
        RigidBodyBoxSettings box;
        box.positionMeters = {0.0, 0.0, 5.0};
        const auto body = world.createBox(box);
        const auto foreign = other.createBox(box);
        const RigidBodySliderSettings valid{body, std::nullopt, box.positionMeters,
            {1.0, 0.0, 0.0}, -1.0, 1.0};
        auto settings = valid;
        settings.body = {};
        requireInvalidInput([&] { (void)world.createSlider(settings); }, "invalid body rejected");
        settings.body = foreign;
        requireInvalidInput([&] { (void)world.createSlider(settings); }, "foreign body rejected");
        settings = valid;
        settings.connectedBody = RigidBodyHandle{};
        requireInvalidInput([&] { (void)world.createSlider(settings); }, "explicit invalid parent rejected");
        settings.connectedBody = foreign;
        requireInvalidInput([&] { (void)world.createSlider(settings); }, "foreign parent rejected");
        settings.connectedBody = body;
        requireInvalidInput([&] { (void)world.createSlider(settings); }, "self slider rejected");
        for (const auto axis : {glm::dvec3{0.0},
            glm::dvec3{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
            glm::dvec3{0.0, std::numeric_limits<double>::infinity(), 0.0},
            glm::dvec3{0.0, 0.0, std::numeric_limits<double>::max()}})
        {
            settings = valid;
            settings.axis = axis;
            requireInvalidInput([&] { (void)world.createSlider(settings); }, "invalid axis rejected");
        }
        for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::infinity(), std::numeric_limits<double>::max()})
        {
            settings = valid;
            settings.anchorPositionMeters.x = invalid;
            requireInvalidInput([&] { (void)world.createSlider(settings); }, "invalid anchor rejected");
            settings = valid;
            settings.minimumTranslationMeters = invalid;
            requireInvalidInput([&] { (void)world.createSlider(settings); }, "invalid minimum rejected");
            settings = valid;
            settings.maximumTranslationMeters = invalid;
            requireInvalidInput([&] { (void)world.createSlider(settings); }, "invalid maximum rejected");
        }
        for (const auto limits : {glm::dvec2{1.0, 2.0}, glm::dvec2{-2.0, -1.0},
            glm::dvec2{1.0, -1.0}, glm::dvec2{0.0, 0.0}, glm::dvec2{-1.0e-300, 1.0e-300}})
        {
            settings = valid;
            settings.minimumTranslationMeters = limits.x;
            settings.maximumTranslationMeters = limits.y;
            requireInvalidInput([&] { (void)world.createSlider(settings); }, "invalid travel range rejected");
        }
        const auto slider = world.createSlider(valid);
        const auto hinge = world.createHinge({body, std::nullopt, box.positionMeters, {1.0, 0.0, 0.0}});
        require(slider.index != hinge.index, "hinges and sliders share unique constraint slots");
        requireInvalidInput([&] { world.setHingeMotor(slider, {}); }, "slider rejected by hinge API");
        requireInvalidInput([&] { world.setSliderMotor(hinge, {}); }, "hinge rejected by slider API");
        requireInvalidInput([&] { (void)world.sliderState(hinge); }, "hinge rejected by slider query");
        requireInvalidInput([&] { (void)world.sliderState({}); }, "invalid query handle rejected");
        requireInvalidInput([&] { world.setSliderMotor({}, {}); }, "invalid motor handle rejected");
        requireInvalidInput([&] { (void)other.sliderState(slider); }, "foreign query handle rejected");
        requireInvalidInput([&] { other.setSliderMotor(slider, {}); }, "foreign motor handle rejected");
        requireInvalidInput([&] { other.removeConstraint(slider); }, "foreign removal rejected");
        for (const double invalid : {-1.0, std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::infinity(), std::numeric_limits<double>::max(), 1.0e-300})
        {
            requireInvalidInput([&] { world.setSliderMotor(slider, {true, 1.0, invalid}); },
                "invalid motor force rejected");
        }
        for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::infinity(), std::numeric_limits<double>::max()})
        {
            requireInvalidInput([&] { world.setSliderMotor(slider, {true, invalid, 1.0}); },
                "invalid motor target rejected");
        }
        world.removeConstraint(slider);
        requireInvalidInput([&] { world.removeConstraint(slider); }, "double removal rejected");
        requireInvalidInput([&] { (void)world.sliderState(slider); }, "removed query rejected");
        const auto replacement = world.createSlider(valid);
        require(replacement.index != slider.index, "removed slider slot is never reused");
        requireInvalidInput([&] { world.setSliderMotor(slider, {}); }, "stale slider stays rejected");
        world.removeBody(body);
        requireInvalidInput([&] { world.setSliderMotor(replacement, {}); }, "body removal invalidates slider");
        requireInvalidInput([&] { world.setHingeMotor(hinge, {}); }, "body removal invalidates hinge too");
        requireInvalidInput([&] { (void)world.bodyState(body); }, "stale body query rejected");
        requireInvalidInput([&] { (void)world.createSlider(valid); }, "stale body creation rejected");

        box.massKilograms = 0.0;
        box.positionMeters = {10.0, 0.0, 5.0};
        const auto support = world.createBox(box);
        box.positionMeters.x = 20.0;
        const auto secondStatic = world.createBox(box);
        requireInvalidInput([&] { (void)world.createSlider(
            {support, std::nullopt, {}, {1.0, 0.0, 0.0}, {}, {}}); }, "static-world slider rejected");
        requireInvalidInput([&] { (void)world.createSlider(
            {support, secondStatic, {}, {1.0, 0.0, 0.0}, {}, {}}); }, "static-static slider rejected");
        box.massKilograms = 1.0;
        box.positionMeters = {12.0, 0.0, 5.0};
        const auto guided = world.createBox(box);
        const auto supported = world.createSlider(
            {guided, support, box.positionMeters, {1.0e-300, 0.0, 0.0}, {}, 1.0});
        box.positionMeters = {14.0, 0.0, 5.0};
        const auto hingedBody = world.createBox(box);
        const auto supportedHinge = world.createHinge(
            {hingedBody, support, box.positionMeters, {0.0, 0.0, 1.0}});
        box.positionMeters = {30.0, 0.0, 5.0};
        const auto independentBody = world.createBox(box);
        const auto independent = world.createSlider(
            {independentBody, std::nullopt, box.positionMeters, {1.0e30, 0.0, 0.0}, -1.0, {}});
        advance(world, 2400);
        require(!world.bodyState(guided).active, "supported horizontal slider sleeps");
        world.removeBody(support);
        require(world.bodyState(guided).active, "support removal wakes slider body");
        requireInvalidInput([&] { (void)world.sliderState(supported); }, "support invalidates attached slider");
        requireInvalidInput([&] { world.removeConstraint(supportedHinge); }, "support invalidates attached hinge");
        requireInvalidInput([&] { (void)world.createSlider(
            {guided, support, {}, {1.0, 0.0, 0.0}, {}, {}}); }, "stale connected body rejected");
        advance(world, 240);
        require(world.bodyState(guided).positionMeters.z < 1.0, "released slider falls under gravity");
        world.setSliderMotor(independent, {true, 1.0, 20.0});
        advance(world, 240);
        require(world.sliderState(independent).translationMeters > 0.9, "unrelated slider survives support removal");
        world.removeConstraint(independent);
        require(world.bodyState(independentBody).active, "explicit removal wakes body");
        advance(world, 240);
        require(world.bodyState(independentBody).positionMeters.z < 1.0,
            "explicit slider removal releases off-axis motion");
        const auto otherSlider = other.createSlider(
            {foreign, std::nullopt, {0.0, 0.0, 5.0}, {1.0, 0.0, 0.0}, {}, {}});
        {
            RigidBodyWorld temporary;
            const auto temporaryBody = temporary.createBox(box);
            const auto temporarySlider = temporary.createSlider(
                {temporaryBody, std::nullopt, box.positionMeters, {1.0, 0.0, 0.0}, {}, {}});
            temporary.setSliderMotor(temporarySlider, {true, 1.0, 20.0});
            advance(temporary, 240);
            // Leave body and slider alive to exercise populated-world destruction.
        }
        other.setSliderMotor(otherSlider, {true, 1.0, 20.0});
        advance(other, 240);
        require(other.sliderState(otherSlider).translationMeters > 0.9,
            "destroying another populated world preserves this slider");
    }
}

int main()
{
    std::cout << std::setprecision(10);
    try
    {
        validationAndLifetime();
        carriageAndRepeatability();
        horizontalActuatorForceDisableAndWake();
        twoBodySlider();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "RigidBodySliderTests failed: " << exception.what() << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "RigidBodySliderTests passed\n";
    return EXIT_SUCCESS;
}
