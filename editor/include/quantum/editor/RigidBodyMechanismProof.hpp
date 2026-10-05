#pragma once

#include <quantum/physics/RigidBodyWorld.hpp>
#include <quantum/renderer/Renderer.hpp>

#include <array>
#include <span>
#include <vector>

namespace quantum::editor
{
    inline constexpr std::string_view mechanicalArmAssetId =
        "assets://mechanical/rotating-arm-placeholder.glb";
    inline constexpr std::string_view mechanicalGondolaAssetId =
        "assets://mechanical/hanging-carrier-placeholder.glb";

    // Presentation owns this relationship. The handle borrows the proof world;
    // the asset identifier references renderer-owned CPU/GPU caches.
    struct RigidBodyMeshBinding
    {
        physics::RigidBodyHandle body;
        std::string assetIdentifier;
        glm::dmat4 localAssetTransform{1.0};

        [[nodiscard]] renderer::StaticMeshInstance instance(
            const physics::RigidBodyWorld& world) const;
    };

    enum class RigidBodyProofRole { Support, DrivenArm, PassiveGondola };

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
        [[nodiscard]] std::array<RigidBodyProofBox, 3> snapshot() const;
        [[nodiscard]] double armAngularSpeedRadiansPerSecond() const;
        [[nodiscard]] const RigidBodyMeshBinding& armMeshBinding() const noexcept
        { return meshBindings_[0]; }
        [[nodiscard]] renderer::StaticMeshInstance armMeshInstance() const;
        [[nodiscard]] std::span<const RigidBodyMeshBinding> meshBindings() const noexcept
        { return meshBindings_; }
        [[nodiscard]] std::array<renderer::StaticMeshInstance, 2> meshInstances() const;

    private:
        // This world contains only the two connected dynamic parts. Their box
        // colliders overlap at the joint; other worlds keep default collision.
        physics::RigidBodyWorld world_{{.dynamicBodyCollisions = false}};
        std::array<physics::RigidBodyHandle, 3> bodies_{};
        std::array<glm::dvec3, 3> halfExtents_{};
        std::array<RigidBodyMeshBinding, 2> meshBindings_;
    };

    // Appends wire boxes to the existing dynamic diagnostic stream. Physics
    // meters are world coordinates here, independent of document scale.
    void appendRigidBodyProofVertices(std::vector<renderer::LineVertex>& vertices,
        std::span<const RigidBodyProofBox> boxes);
}
