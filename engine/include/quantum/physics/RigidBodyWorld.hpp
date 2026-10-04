#pragma once

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

namespace quantum::physics
{
    class RigidBodyWorld;

    // Valid only in the creating world, until removal or world destruction.
    // Indices are never reused during a world's lifetime.
    struct RigidBodyHandle
    {
        const RigidBodyWorld* world = nullptr;
        std::size_t index = 0;
    };

    struct RigidBodyBoxSettings
    {
        glm::dvec3 halfExtentsMeters{0.5};
        glm::dvec3 positionMeters{0.0};
        glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
        // Zero creates a static body; a positive value creates a dynamic body.
        double massKilograms = 1.0;
    };

    struct RigidBodyConstraintHandle
    {
        const RigidBodyWorld* world = nullptr;
        std::size_t index = 0;
    };

    struct RigidBodyHingeSettings
    {
        RigidBodyHandle body;
        // Absent attaches to the static world. A supplied handle must be valid.
        std::optional<RigidBodyHandle> connectedBody;
        glm::dvec3 anchorPositionMeters{0.0};
        // World-space direction at creation; normalized internally.
        glm::dvec3 axis{0.0, 1.0, 0.0};
    };

    struct RigidBodyHingeMotorSettings
    {
        bool enabled = false;
        // Positive follows the right-hand rule about the hinge axis for body
        // relative to connectedBody (or the static world).
        double targetAngularVelocityRadiansPerSecond = 0.0;
        double maximumTorqueNewtonMeters = 0.0;
    };

    struct RigidBodyState
    {
        glm::dvec3 positionMeters{0.0};
        glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
        glm::dvec3 linearVelocityMetersPerSecond{0.0};
        bool active = false;
        glm::dvec3 angularVelocityRadiansPerSecond{0.0};
    };

    // Independent general physics; no track, train, document, or renderer state.
    // All operations belong on the owning simulation thread, outside a step.
    // Destruction releases constraints before removing/destroying their bodies.
    class RigidBodyWorld
    {
    public:
        RigidBodyWorld();
        ~RigidBodyWorld();

        RigidBodyWorld(const RigidBodyWorld&) = delete;
        RigidBodyWorld& operator=(const RigidBodyWorld&) = delete;
        RigidBodyWorld(RigidBodyWorld&&) = delete;
        RigidBodyWorld& operator=(RigidBodyWorld&&) = delete;

        [[nodiscard]] RigidBodyHandle createBox(
            const RigidBodyBoxSettings& settings);
        [[nodiscard]] RigidBodyState bodyState(RigidBodyHandle handle) const;

        // Removes attached constraints first; all affected handles become stale.
        void removeBody(RigidBodyHandle handle);

        // Unlimited revolute motion; at least one connected body must be dynamic.
        [[nodiscard]] RigidBodyConstraintHandle createHinge(
            const RigidBodyHingeSettings& settings);
        void setHingeMotor(RigidBodyConstraintHandle handle,
            const RigidBodyHingeMotorSettings& settings);
        // Wakes connected bodies; release preserves their current velocities.
        void removeConstraint(RigidBodyConstraintHandle handle);

        // Exactly one existing QUANTUM 1/240 s tick, with no internal clock.
        void stepFixed();
        [[nodiscard]] std::uint64_t tick() const noexcept;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
