#pragma once

#include <quantum/physics/RigidBodyWorld.hpp>
#include <quantum/renderer/Renderer.hpp>

#include <array>
#include <span>
#include <vector>

namespace quantum::editor
{
    enum class RigidBodyProofRole { Support, DrivenArm };

    // Copied presentation values; no body handles or borrowed physics objects.
    struct RigidBodyProofBox
    {
        glm::dvec3 positionMeters{0.0};
        glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
        glm::dvec3 halfExtentsMeters{0.5};
        RigidBodyProofRole role = RigidBodyProofRole::Support;
    };

    // Editor-only engineering demonstration. Destroying/replacing this owner
    // destroys its isolated world; detach SimulationPreview's pointer first.
    class RigidBodyMechanismProof
    {
    public:
        RigidBodyMechanismProof();

        [[nodiscard]] physics::RigidBodyWorld& world() noexcept { return world_; }
        [[nodiscard]] std::array<RigidBodyProofBox, 2> snapshot() const;
        [[nodiscard]] double armAngularSpeedRadiansPerSecond() const;

    private:
        physics::RigidBodyWorld world_;
        std::array<physics::RigidBodyHandle, 2> bodies_{};
        std::array<glm::dvec3, 2> halfExtents_{};
    };

    // Appends wire boxes to the existing dynamic diagnostic stream. Physics
    // meters are world coordinates here, independent of document scale.
    void appendRigidBodyProofVertices(std::vector<renderer::LineVertex>& vertices,
        std::span<const RigidBodyProofBox> boxes);
}
