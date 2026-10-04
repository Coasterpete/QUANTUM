#include <quantum/physics/RigidBodyWorld.hpp>

#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

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

    void requireFinite(const RigidBodyState& state)
    {
        require(std::isfinite(glm::length(state.positionMeters))
            && std::isfinite(glm::length(state.linearVelocityMetersPerSecond))
            && std::isfinite(glm::length(state.angularVelocityRadiansPerSecond))
            && std::isfinite(glm::length(state.orientation)), "finite mechanical state");
        require(std::abs(glm::length(state.orientation) - 1.0) < 1.0e-5,
            "unit mechanical orientation");
    }

    [[nodiscard]] RigidBodyBoxSettings armBox(const glm::dvec3& position)
    {
        RigidBodyBoxSettings box;
        box.halfExtentsMeters = {2.0, 0.1, 0.1};
        box.positionMeters = position;
        box.massKilograms = 4.0;
        return box;
    }

    [[nodiscard]] double pivotDrift(const RigidBodyState& state,
        const glm::dvec3& localPivot, const glm::dvec3& anchor)
    {
        return glm::length(state.positionMeters + state.orientation * localPivot - anchor);
    }

    void motorizedRotatingArm()
    {
        RigidBodyWorld world;
        const glm::dvec3 anchor{0.0, 0.0, 6.0};
        const glm::dvec3 axis{0.0, 1.0, 0.0};
        const glm::dvec3 localPivot{-2.0, 0.0, 0.0};
        const auto arm = world.createBox(armBox(anchor - localPivot));
        const auto hinge = world.createHinge({arm, std::nullopt, anchor, 3.0 * axis});
        world.setHingeMotor(hinge, {true, 1.5, 200.0});
        double maximumDrift = 0.0;
        double maximumAxisError = 0.0;
        double maximumSpeedError = 0.0;
        double accumulatedAngle = 0.0;
        double previousAngle = 0.0;
        // 30 seconds, then 5 seconds in reverse. Check every 1/240 s tick.
        for (int tick = 0; tick < 8400; ++tick)
        {
            if (tick == 7200) world.setHingeMotor(hinge, {true, -1.5, 200.0});
            world.stepFixed();
            const auto state = world.bodyState(arm);
            requireFinite(state);
            const double drift = pivotDrift(state, localPivot, anchor);
            const double axisError = glm::length(state.orientation * axis - axis);
            maximumDrift = std::max(maximumDrift, drift);
            maximumAxisError = std::max(maximumAxisError, axisError);
            require(drift < 0.02, "motorized pivot remains within 2 cm");
            require(axisError < 1.0e-4, "hinge preserves its world Y axis");
            const auto velocity = state.angularVelocityRadiansPerSecond;
            require(glm::length(velocity - glm::dot(velocity, axis) * axis) < 0.01,
                "hinge suppresses angular velocity off its axis");
            const double target = tick < 7200 ? 1.5 : -1.5;
            if ((tick >= 240 && tick < 7200) || tick >= 7440)
            {
                const double error = std::abs(glm::dot(velocity, axis) - target);
                maximumSpeedError = std::max(maximumSpeedError, error);
                require(error < 0.05, "motor approaches signed target within 0.05 rad/s");
            }
            const auto radial = state.orientation * glm::dvec3{1.0, 0.0, 0.0};
            const double angle = std::atan2(-radial.z, radial.x);
            const double delta = std::atan2(std::sin(angle - previousAngle),
                std::cos(angle - previousAngle));
            if (tick < 7200) accumulatedAngle += delta;
            previousAngle = angle;
        }
        require(accumulatedAngle > 40.0, "transform demonstrates repeated positive rotations");
        require(world.tick() == 8400, "mechanism uses only existing fixed ticks");
        std::cout << "Motorized arm: seconds=35 maxPivotDrift=" << maximumDrift
            << " maxAxisError=" << maximumAxisError
            << " maxSpeedError=" << maximumSpeedError
            << " forwardAngle=" << accumulatedAngle << '\n';

        world.removeConstraint(hinge);
        advance(world, 240);
        require(pivotDrift(world.bodyState(arm), localPivot, anchor) > 0.5,
            "released rotating arm escapes pivot under gravity");
        requireInvalidInput([&] { world.setHingeMotor(hinge, {}); },
            "removed constraint rejects motor operations");
        world.removeBody(arm);
        advance(world, 240);
    }

    void motorDisableTorqueLimitAndWake()
    {
        RigidBodyWorld world;
        const glm::dvec3 anchor{0.0, 0.0, 5.0};
        const auto arm = world.createBox(armBox(anchor));
        const auto hinge = world.createHinge({arm, std::nullopt, anchor, {0.0, 0.0, 1.0}});
        // Gravity has no torque about this centered vertical hinge.
        world.setHingeMotor(hinge, {true, 1.5, 0.0});
        advance(world, 480);
        require(glm::length(world.bodyState(arm).angularVelocityRadiansPerSecond) < 0.01,
            "zero torque motor cannot accelerate the arm");
        advance(world, 2400);
        require(!world.bodyState(arm).active, "stationary constrained arm sleeps");
        world.setHingeMotor(hinge, {true, 1.5, 10.0});
        advance(world, 480);
        const double drivenSpeed = world.bodyState(arm).angularVelocityRadiansPerSecond.z;
        require(std::abs(drivenSpeed - 1.5) < 0.05, "motor wakes and drives sleeping body");
        world.setHingeMotor(hinge, {false, -3.0, 10.0});
        advance(world, 1200);
        const double passiveSpeed = world.bodyState(arm).angularVelocityRadiansPerSecond.z;
        require(passiveSpeed > 0.5 && passiveSpeed < drivenSpeed * 0.9,
            "disabled motor ignores reverse target and leaves damped free rotation");
        world.setHingeMotor(hinge, {true, 0.0, 10.0});
        advance(world, 480);
        const double stoppedSpeed = world.bodyState(arm).angularVelocityRadiansPerSecond.z;
        require(std::abs(stoppedSpeed) < 0.01, "enabled zero target actively brakes");
        std::cout << "Motor control: driven=" << drivenSpeed << " disabled=" << passiveSpeed
            << " braked=" << stoppedSpeed << '\n';
    }

    void passivePendulumAndSleepingRelease()
    {
        RigidBodyWorld world;
        const glm::dvec3 anchor{0.0, 0.0, 6.0};
        const glm::dvec3 localPivot{-2.0, 0.0, 0.0};
        const auto arm = world.createBox(armBox(anchor - localPivot));
        const auto hinge = world.createHinge({arm, std::nullopt, anchor, {0.0, 1.0, 0.0}});
        double maximumDrift = 0.0;
        double earlySpeed = 0.0;
        double lateSpeed = 0.0;
        // Default M0 damping, no motor/friction/spring tuning.
        for (int tick = 0; tick < 57600; ++tick)
        {
            world.stepFixed();
            const auto state = world.bodyState(arm);
            requireFinite(state);
            maximumDrift = std::max(maximumDrift, pivotDrift(state, localPivot, anchor));
            require(maximumDrift < 0.02, "passive pivot remains within 2 cm");
            require(glm::length(state.orientation * glm::dvec3{0.0, 1.0, 0.0}
                - glm::dvec3{0.0, 1.0, 0.0}) < 1.0e-4, "passive hinge preserves Y axis");
            if (tick < 1200) earlySpeed = std::max(earlySpeed,
                glm::length(state.angularVelocityRadiansPerSecond));
            if (tick >= 56400) lateSpeed = std::max(lateSpeed,
                glm::length(state.angularVelocityRadiansPerSecond));
        }
        const auto settled = world.bodyState(arm);
        std::cout << "Passive pendulum: seconds=240 maxPivotDrift=" << maximumDrift
            << " earlySpeed=" << earlySpeed << " lateSpeed=" << lateSpeed
            << " finalZ=" << settled.positionMeters.z << " active=" << settled.active << '\n';
        require(earlySpeed > 1.0, "gravity drives an unpowered swing");
        require(lateSpeed < 0.05 && lateSpeed < earlySpeed * 0.05,
            "default damping settles passive swing");
        require(std::abs(settled.positionMeters.z - 4.0) < 0.02,
            "pendulum settles beneath pivot");
        require(!settled.active, "settled pendulum sleeps");
        world.removeConstraint(hinge);
        advance(world, 240);
        require(world.bodyState(arm).positionMeters.z < settled.positionMeters.z - 4.0,
            "constraint removal wakes a sleeping supported body to fall");
    }

    void twoBodyHinge()
    {
        RigidBodyWorld world;
        RigidBodyBoxSettings box;
        box.halfExtentsMeters = {0.4, 0.1, 0.1};
        // Separate boxes along the hinge axis so their colliders cannot meet
        // during relative rotation. Constraints retain normal collision rules.
        box.positionMeters = {0.0, 0.0, 4.5};
        const auto parent = world.createBox(box);
        box.positionMeters.z = 5.5;
        const auto child = world.createBox(box);
        const glm::dvec3 anchor{0.0, 0.0, 5.0};
        const auto hinge = world.createHinge({child, parent, anchor, {0.0, 0.0, 1.0}});
        world.setHingeMotor(hinge, {true, 1.0, 5.0});
        double maximumDrift = 0.0;
        for (int tick = 0; tick < 480; ++tick)
        {
            world.stepFixed();
            const auto first = world.bodyState(parent);
            const auto second = world.bodyState(child);
            requireFinite(first);
            requireFinite(second);
            const auto firstPivot = first.positionMeters
                + first.orientation * glm::dvec3{0.0, 0.0, 0.5};
            const auto secondPivot = second.positionMeters
                + second.orientation * glm::dvec3{0.0, 0.0, -0.5};
            maximumDrift = std::max(maximumDrift, glm::length(firstPivot - secondPivot));
            require(maximumDrift < 0.02, "two dynamic bodies share their moving pivot");
            if (tick >= 240)
            {
                const double relativeSpeed = second.angularVelocityRadiansPerSecond.z
                    - first.angularVelocityRadiansPerSecond.z;
                require(std::abs(relativeSpeed - 1.0) < 0.05,
                    "two-body motor drives child relative to parent: tick="
                        + std::to_string(tick) + " speed=" + std::to_string(relativeSpeed));
            }
        }
        require(world.bodyState(parent).positionMeters.z < 0.0,
            "two-body hinge does not fix either body to world");
        world.removeBody(parent);
        requireInvalidInput([&] { world.removeConstraint(hinge); },
            "removing connected body also removes hinge");
        advance(world, 240);
        requireFinite(world.bodyState(child));
        std::cout << "Two-body hinge: maxPivotDrift=" << maximumDrift << '\n';
    }

    void validationAndLifetime()
    {
        RigidBodyWorld world;
        const auto body = world.createBox(armBox({2.0, 0.0, 6.0}));
        RigidBodyWorld other;
        const auto foreign = other.createBox(armBox({2.0, 0.0, 6.0}));
        RigidBodyHingeSettings settings{body, std::nullopt, {0.0, 0.0, 6.0}, {0.0, 1.0, 0.0}};
        const auto valid = settings;
        settings.body = {};
        requireInvalidInput([&] { (void)world.createHinge(settings); }, "invalid body rejected");
        settings.body = foreign;
        requireInvalidInput([&] { (void)world.createHinge(settings); }, "foreign body rejected");
        settings = valid;
        settings.connectedBody = RigidBodyHandle{};
        requireInvalidInput([&] { (void)world.createHinge(settings); }, "explicit invalid parent rejected");
        settings.connectedBody = foreign;
        requireInvalidInput([&] { (void)world.createHinge(settings); }, "foreign parent rejected");
        settings.connectedBody = body;
        requireInvalidInput([&] { (void)world.createHinge(settings); }, "self hinge rejected");
        settings = valid;
        settings.axis = {0.0, 0.0, 0.0};
        requireInvalidInput([&] { (void)world.createHinge(settings); }, "zero axis rejected");
        settings.axis.x = std::numeric_limits<double>::quiet_NaN();
        requireInvalidInput([&] { (void)world.createHinge(settings); }, "NaN axis rejected");
        settings = valid;
        settings.anchorPositionMeters.z = std::numeric_limits<double>::infinity();
        requireInvalidInput([&] { (void)world.createHinge(settings); }, "infinite anchor rejected");
        settings.anchorPositionMeters.z = std::numeric_limits<double>::max();
        requireInvalidInput([&] { (void)world.createHinge(settings); }, "out of range anchor rejected");

        RigidBodyBoxSettings support;
        support.massKilograms = 0.0;
        support.positionMeters = {10.0, 0.0, 6.0};
        const auto staticBody = world.createBox(support);
        const auto secondStatic = world.createBox(support);
        requireInvalidInput([&] { (void)world.createHinge(
            {staticBody, std::nullopt, {}, {1.0, 0.0, 0.0}}); }, "static-world hinge rejected");
        requireInvalidInput([&] { (void)world.createHinge(
            {staticBody, secondStatic, {}, {1.0, 0.0, 0.0}}); }, "static-static hinge rejected");
        const auto hinge = world.createHinge(valid);
        requireInvalidInput([&] { world.setHingeMotor(hinge, {true, 1.0, -1.0}); },
            "negative torque rejected");
        requireInvalidInput([&] { world.setHingeMotor(hinge,
            {true, std::numeric_limits<double>::infinity(), 1.0}); }, "infinite target rejected");
        requireInvalidInput([&] { world.setHingeMotor(hinge,
            {true, 1.0, std::numeric_limits<double>::quiet_NaN()}); }, "NaN torque rejected");
        requireInvalidInput([&] { world.setHingeMotor({}, {}); }, "invalid constraint rejected");
        requireInvalidInput([&] { other.setHingeMotor(hinge, {}); }, "foreign constraint rejected");
        requireInvalidInput([&] { other.removeConstraint(hinge); }, "foreign removal rejected");
        requireInvalidInput([&] { other.removeBody(body); }, "foreign body removal rejected");
        requireInvalidInput([&] { world.removeBody({}); }, "invalid body removal rejected");
        world.removeConstraint(hinge);
        requireInvalidInput([&] { world.removeConstraint(hinge); }, "double removal rejected");
        const auto replacement = world.createHinge(valid);
        require(replacement.index != hinge.index, "constraint slots are not reused");
        requireInvalidInput([&] { world.setHingeMotor(hinge, {}); }, "stale hinge stays rejected");
        world.removeBody(body);
        requireInvalidInput([&] { (void)world.bodyState(body); }, "removed body query rejected");
        requireInvalidInput([&] { world.removeBody(body); }, "removed body removal rejected");
        requireInvalidInput([&] { (void)world.createHinge(valid); }, "removed body hinge rejected");
        requireInvalidInput([&] { world.setHingeMotor(replacement, {}); },
            "body removal invalidates attached constraint");
        const auto newBody = world.createBox(armBox({22.0, 0.0, 6.0}));
        require(newBody.index != body.index, "body slots are not reused");
        requireInvalidInput([&] { (void)world.bodyState(body); }, "stale body stays rejected");

        // Removing one support destroys all attached hinges, preserves unrelated
        // ones, and wakes their surviving bodies even when they were asleep.
        auto hangingBox = armBox({0.0, 0.0, 4.0});
        hangingBox.orientation = glm::angleAxis(1.5707963267948966, glm::dvec3{0.0, 1.0, 0.0});
        const auto hanging = world.createBox(hangingBox);
        const auto supported = world.createHinge(
            {hanging, staticBody, {0.0, 0.0, 6.0}, {0.0, 1.0, 0.0}});
        const auto alsoSupported = world.createHinge(
            {newBody, staticBody, {20.0, 0.0, 6.0}, {0.0, 1.0, 0.0}});
        const auto independentBody = world.createBox(armBox({42.0, 0.0, 6.0}));
        const auto independent = world.createHinge(
            {independentBody, std::nullopt, {40.0, 0.0, 6.0}, {0.0, 1.0, 0.0}});
        const auto otherHinge = other.createHinge(
            {foreign, std::nullopt, {0.0, 0.0, 6.0}, {0.0, 1.0, 0.0}});
        advance(world, 2400);
        require(!world.bodyState(hanging).active, "supported hanging body sleeps");
        const double supportedHeight = world.bodyState(hanging).positionMeters.z;
        world.removeBody(staticBody);
        requireInvalidInput([&] { world.setHingeMotor(supported, {}); },
            "support removal invalidates first attached hinge");
        requireInvalidInput([&] { world.removeConstraint(alsoSupported); },
            "support removal invalidates every attached hinge");
        requireInvalidInput([&] { (void)world.createHinge(
            {newBody, staticBody, {}, {0.0, 1.0, 0.0}}); }, "stale parent rejected");
        world.setHingeMotor(independent, {true, 1.0, 200.0});
        advance(world, 240);
        require(world.bodyState(hanging).positionMeters.z < supportedHeight - 4.0,
            "support body removal wakes surviving supported body");
        require(pivotDrift(world.bodyState(independentBody), {-2.0, 0.0, 0.0},
            {40.0, 0.0, 6.0}) < 0.02, "unrelated hinge survives support removal");
        world.removeBody(secondStatic);
        other.setHingeMotor(otherHinge, {true, 1.0, 200.0});
        {
            RigidBodyWorld temporary;
            const auto temporaryArm = temporary.createBox(armBox({2.0, 0.0, 6.0}));
            const auto temporaryHinge = temporary.createHinge(
                {temporaryArm, std::nullopt, {0.0, 0.0, 6.0}, {0.0, 1.0, 0.0}});
            temporary.setHingeMotor(temporaryHinge, {true, 1.0, 200.0});
            advance(temporary, 240);
            // Scope teardown deliberately leaves both body and constraint alive.
        }
        advance(other, 240);
        require(std::abs(other.bodyState(foreign).angularVelocityRadiansPerSecond.y - 1.0)
            < 0.05, "destroying another populated world preserves this motor");
    }
}

int main()
{
    try
    {
        validationAndLifetime();
        motorizedRotatingArm();
        motorDisableTorqueLimitAndWake();
        passivePendulumAndSleepingRelease();
        twoBodyHinge();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "RigidBodyConstraintTests failed: " << exception.what() << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "RigidBodyConstraintTests passed\n";
    return EXIT_SUCCESS;
}
