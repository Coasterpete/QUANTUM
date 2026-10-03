#include <quantum/physics/RigidBodyWorld.hpp>

#include <glm/geometric.hpp>

#include <cmath>
#include <cstdlib>
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

    void requireNear(const double actual, const double expected,
        const double tolerance, const std::string& message)
    {
        require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
            message);
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

    void lifecycleAndConversions()
    {
        // Two simultaneous worlds exercise shared registration and separate
        // body storage; repeated scopes exercise populated-world teardown.
        for (int iteration = 0; iteration < 3; ++iteration)
        {
            RigidBodyWorld world;
            RigidBodyWorld other;
            world.stepFixed();
            require(world.tick() == 1 && other.tick() == 0, "independent world ticks");
            RigidBodyBoxSettings box;
            box.positionMeters = {1.0, 2.0, 3.0};
            box.orientation = glm::angleAxis(0.7, glm::dvec3{0.0, 0.0, 1.0});
            box.massKilograms = 0.0;
            const auto body = world.createBox(box);
            world.stepFixed();
            const auto state = world.bodyState(body);
            require(state.positionMeters == box.positionMeters, "static XYZ preserved");
            requireNear(glm::dot(state.orientation, box.orientation), 1.0, 1.0e-6,
                "quaternion component order preserved");
            requireInvalidInput([&] { (void)other.bodyState(body); },
                "foreign-world handle must be rejected");
        }
    }

    void gravity()
    {
        RigidBodyWorld world;
        RigidBodyBoxSettings box;
        box.positionMeters = {2.0, 3.0, 5.0};
        const auto body = world.createBox(box);
        for (int step = 0; step < 120; ++step) world.stepFixed();
        const auto state = world.bodyState(body);
        require(state.positionMeters.z < 4.0
            && state.linearVelocityMetersPerSecond.z < -4.0,
            "dynamic body must fall under negative-Z gravity");
        requireNear(state.positionMeters.x, 2.0, 1.0e-6, "gravity must preserve X");
        requireNear(state.positionMeters.y, 3.0, 1.0e-6, "gravity must preserve Y");
    }

    [[nodiscard]] std::vector<RigidBodyState> fallingBoxTrajectory()
    {
        RigidBodyWorld world;
        RigidBodyBoxSettings floor;
        floor.halfExtentsMeters = {10.0, 10.0, 0.5};
        floor.positionMeters = {0.0, 0.0, -0.5};
        floor.massKilograms = 0.0;
        const auto floorBody = world.createBox(floor);
        RigidBodyBoxSettings box;
        box.positionMeters = {0.0, 0.0, 5.0};
        const auto body = world.createBox(box);
        std::vector<RigidBodyState> trajectory;
        trajectory.reserve(2400);
        for (int step = 0; step < 2400; ++step)
        {
            world.stepFixed();
            const auto state = world.bodyState(body);
            // Allow 2 cm of solver contact slop, but never floor tunnelling.
            require(state.positionMeters.z >= 0.48, "box must remain above floor");
            trajectory.push_back(state);
        }
        const auto& settled = trajectory.back();
        requireNear(settled.positionMeters.z, 0.5, 0.02, "box must settle on floor top");
        require(glm::length(settled.linearVelocityMetersPerSecond) < 0.01,
            "settled body speed must be below 1 cm/s");
        require(!settled.active, "settled body should sleep");
        require(world.bodyState(floorBody).positionMeters == floor.positionMeters,
            "collision must not move static floor");
        std::cout << "Falling box: tick=" << world.tick()
            << " z=" << settled.positionMeters.z
            << " speed=" << glm::length(settled.linearVelocityMetersPerSecond)
            << " active=" << settled.active << '\n';
        return trajectory;
    }

    void collisionAndRepeatability()
    {
        const auto first = fallingBoxTrajectory();
        const auto second = fallingBoxTrajectory();
        for (std::size_t index = 0; index < first.size(); ++index)
        {
            require(glm::length(first[index].positionMeters - second[index].positionMeters)
                    <= 1.0e-6
                && glm::length(first[index].linearVelocityMetersPerSecond
                    - second[index].linearVelocityMetersPerSecond) <= 1.0e-6
                && std::abs(glm::dot(first[index].orientation, second[index].orientation)
                    - 1.0) <= 1.0e-6
                && first[index].active == second[index].active,
                "same-configuration trajectory must repeat within tolerance");
        }
    }

    void invalidInputsLeaveWorldUsable()
    {
        RigidBodyWorld world;
        RigidBodyBoxSettings box;
        box.halfExtentsMeters.x = 0.0;
        requireInvalidInput([&] { (void)world.createBox(box); }, "zero extent rejected");
        box = {};
        box.massKilograms = -1.0;
        requireInvalidInput([&] { (void)world.createBox(box); }, "negative mass rejected");
        box = {};
        box.positionMeters.z = std::numeric_limits<double>::infinity();
        requireInvalidInput([&] { (void)world.createBox(box); }, "infinite input rejected");
        box = {};
        box.orientation = {0.0, 0.0, 0.0, 0.0};
        requireInvalidInput([&] { (void)world.createBox(box); }, "zero quaternion rejected");
        const auto body = world.createBox({});
        world.stepFixed();
        require(world.bodyState(body).positionMeters.z < 0.0,
            "world remains usable after rejected creation");
    }
}

int main()
{
    try
    {
        lifecycleAndConversions();
        gravity();
        collisionAndRepeatability();
        invalidInputsLeaveWorldUsable();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "RigidBodyWorldTests failed: " << exception.what() << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "RigidBodyWorldTests passed\n";
    return EXIT_SUCCESS;
}
