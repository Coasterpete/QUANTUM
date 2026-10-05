#include <quantum/editor/RigidBodyMechanismProof.hpp>
#include <quantum/editor/SimulationPreview.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <numbers>
#include <stdexcept>

#if defined(_MSC_VER) && defined(_DEBUG)
#include <crtdbg.h>
#endif

namespace
{
    using namespace quantum::editor;
    using namespace quantum::renderer;

    void require(const bool condition, const char* message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

    template<typename Operation>
    void requireRejected(Operation operation)
    {
        bool rejected = false;
        try { operation(); }
        catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "Invalid/removed body or constraint must be rejected.");
    }

    void assemblyAndTrajectory(const std::filesystem::path& root)
    {
        StaticMeshAssetCache cache{root};
        const std::array assets{cache.load(mechanicalArmAssetId), cache.load(mechanicalGondolaAssetId)};
        glm::vec3 minimum{1e9F}, maximum{-1e9F};
        for (const auto& vertex : assets[1]->vertices)
        {
            minimum = glm::min(minimum, vertex.position);
            maximum = glm::max(maximum, vertex.position);
        }
        require(glm::length(minimum - glm::vec3{-0.65F, -0.28F, -2.0F}) < 1e-5F
            && glm::length(maximum - glm::vec3{0.65F, 0.28F, 0.0F}) < 1e-5F,
            "Carrier must use meters/+Z-up, with origin at the upper hinge and a narrow upper bracket.");
        require(assets[1]->submeshes.size() == 1 && assets[1]->submeshes[0].material.has_value(),
            "Carrier must use the existing one-mesh PBR path.");

        RigidBodyMechanismProof proof, repeat;
        const auto initial = proof.snapshot();
        const auto bindings = proof.meshBindings();
        require(initial.size() == 3 && initial[2].role == RigidBodyProofRole::PassiveGondola,
            "Expected static support, driven arm and passive carrier.");
        require(bindings.size() == 2 && bindings[0].body.index == 1 && bindings[1].body.index == 2
            && bindings[0].body.world == &proof.world() && bindings[1].body.world == &proof.world(),
            "Two bindings must reference distinct bodies in the owning world.");
        requireRejected([&] { (void)proof.world().bodyState({&proof.world(), 3}); });
        requireRejected([&] { proof.world().removeConstraint({&proof.world(), 2}); });

        StaticMeshGpuHandleCache gpuCache;
        int uploads = 0;
        double primaryDrift = 0, secondarySeparation = 0;
        double axisError = 0;
        double relativeAngle = 0, previousAngle = 0, minimumAngle = 0, maximumAngle = 0;
        double carrierAngle = 0, previousCarrierAngle = 0, minimumCarrierAngle = 0, maximumCarrierAngle = 0;
        double speedDifference = 0;
        // 30 seconds, two complete trajectories. Default solver settings stay intact.
        for (int tick = 0; tick < 7200; ++tick)
        {
            proof.world().stepFixed();
            repeat.world().stepFixed();
            const auto boxes = proof.snapshot();
            const auto instances = proof.meshInstances();
            require(boxes[0].positionMeters == initial[0].positionMeters
                && boxes[0].orientation == initial[0].orientation, "Support must remain fixed.");
            const auto arm = proof.world().bodyState(bindings[0].body);
            const auto carrier = proof.world().bodyState(bindings[1].body);
            axisError = std::max(axisError, glm::length(arm.orientation * glm::dvec3{0, 1, 0}
                - glm::dvec3{0, 1, 0}));
            const auto primary = arm.positionMeters + arm.orientation * glm::dvec3{-2, 0, 0};
            const auto armEnd = arm.positionMeters + arm.orientation * glm::dvec3{2, 0, 0};
            const auto carrierPivot = carrier.positionMeters + carrier.orientation * glm::dvec3{0, 0, 1};
            primaryDrift = std::max(primaryDrift, glm::length(primary - glm::dvec3{0, -12, 6}));
            secondarySeparation = std::max(secondarySeparation, glm::length(armEnd - carrierPivot));
            const auto relativeAxis = glm::inverse(arm.orientation) * carrier.orientation * glm::dvec3{1, 0, 0};
            const double angle = std::atan2(-relativeAxis.z, relativeAxis.x);
            relativeAngle += std::remainder(angle - previousAngle, 2.0 * std::numbers::pi);
            previousAngle = angle;
            minimumAngle = std::min(minimumAngle, relativeAngle);
            maximumAngle = std::max(maximumAngle, relativeAngle);
            const auto carrierAxis = carrier.orientation * glm::dvec3{1, 0, 0};
            const double absoluteAngle = std::atan2(-carrierAxis.z, carrierAxis.x);
            carrierAngle += std::remainder(absoluteAngle - previousCarrierAngle, 2.0 * std::numbers::pi);
            previousCarrierAngle = absoluteAngle;
            minimumCarrierAngle = std::min(minimumCarrierAngle, carrierAngle);
            maximumCarrierAngle = std::max(maximumCarrierAngle, carrierAngle);
            speedDifference = std::max(speedDifference, std::abs(
                arm.angularVelocityRadiansPerSecond.y - carrier.angularVelocityRadiansPerSecond.y));

            for (std::size_t index = 0; index < bindings.size(); ++index)
            {
                const auto state = proof.world().bodyState(bindings[index].body);
                const auto repeated = repeat.world().bodyState(repeat.meshBindings()[index].body);
                require(glm::length(state.positionMeters - repeated.positionMeters) < 1e-6
                    && glm::length(state.linearVelocityMetersPerSecond - repeated.linearVelocityMetersPerSecond) < 1e-6
                    && glm::length(state.angularVelocityRadiansPerSecond - repeated.angularVelocityRadiansPerSecond) < 1e-6
                    && std::abs(std::abs(glm::dot(state.orientation, repeated.orientation)) - 1.0) < 1e-6
                    && state.active == repeated.active,
                    "Assembly trajectories must repeat on this executable/configuration.");
                require(instances[index].assetIdentifier == bindings[index].assetIdentifier,
                    "Each instance must preserve its own asset identity.");
                for (const auto& vertex : assets[index]->vertices)
                {
                    const auto local = glm::dvec3{bindings[index].localAssetTransform
                        * glm::dvec4{vertex.position, 1.0}};
                    const auto rendered = glm::dvec3{instances[index].transform
                        * glm::vec4{vertex.position, 1.0F}};
                    require(glm::length(rendered - (state.positionMeters + state.orientation * local)) < 3e-6,
                        "Every mesh vertex must follow its own bodyState, without a parent transform.");
                }
                (void)gpuCache.getOrUpload(*cache.load(instances[index].assetIdentifier),
                    [&uploads](const StaticMeshAsset&) { return StaticMeshGpuHandle{static_cast<std::uint32_t>(uploads++)}; });
            }
            require(instances[0].transform != instances[1].transform, "Bindings must remain independent.");
        }
        require(maximumAngle - minimumAngle > 1.0 && speedDifference > 0.5,
            "Passive carrier must rotate relative to the motor-driven arm.");
        require(maximumCarrierAngle - minimumCarrierAngle > 0.2,
            "Passive carrier must actually swing in world space, not remain at a fixed orientation.");
        require(std::abs(proof.armAngularSpeedRadiansPerSecond() - 1.5) < 0.05,
            "Arm must retain its motor target under carrier load.");
        require(uploads == 2 && cache.size() == 2, "Two immutable assets must each load/upload only once.");
        std::vector<LineVertex> bounds;
        appendRigidBodyProofVertices(bounds, proof.snapshot());
        require(bounds.size() == 72 && bounds[24].color != bounds[48].color,
            "Both moving collider bounds must have distinguishable diagnostic colors.");
        std::cout << "M5: 7200 ticks / 30 s, twice; primary drift " << primaryDrift
            << " m; secondary separation " << secondarySeparation << " m; relative range "
            << maximumAngle - minimumAngle << " rad; peak speed difference " << speedDifference
            << " rad/s; carrier world-angle range " << maximumCarrierAngle - minimumCarrierAngle
            << " rad; primary axis error " << axisError << "; two cached uploads; carrier " << assets[1]->vertices.size()
            << " vertices / " << assets[1]->triangleIndices.size() << " indices.\n";
        // 2 mm is the existing visible M3/M4 pivot bound. Report the whole
        // trajectory before evaluating it so failures include both measurements.
        require(primaryDrift < 0.002 && secondarySeparation < 0.002,
            "Both hinge attachment errors must remain below 2 mm.");
        require(axisError < 1e-5, "Primary hinge must preserve the M3 axis alignment bound.");
    }

    void isolatedCollisionPolicy()
    {
        using namespace quantum::physics;
        RigidBodyWorld normal, mechanism{{.dynamicBodyCollisions = false}};
        RigidBodyBoxSettings box;
        box.positionMeters = {0, 0, 5};
        const auto normalFirst = normal.createBox(box);
        const auto mechanismFirst = mechanism.createBox(box);
        box.positionMeters.x = 0.8;
        const auto normalSecond = normal.createBox(box);
        const auto mechanismSecond = mechanism.createBox(box);
        // The two 1 m boxes initially overlap by 0.2 m.
        for (int tick = 0; tick < 60; ++tick)
        {
            normal.stepFixed();
            mechanism.stepFixed();
        }
        // Default contact slop can retain about 2 cm penetration. This still
        // clearly distinguishes separation from the unchanged 0.8 m overlap.
        require(glm::length(normal.bodyState(normalFirst).positionMeters
            - normal.bodyState(normalSecond).positionMeters) > 0.97,
            "Default worlds must retain dynamic/dynamic collision separation.");
        require(std::abs(glm::length(mechanism.bodyState(mechanismFirst).positionMeters
            - mechanism.bodyState(mechanismSecond).positionMeters) - 0.8) < 1e-6,
            "Only the opted-out world must exclude dynamic/dynamic contact.");
        box.positionMeters = {0, 0, -0.5};
        box.halfExtentsMeters = {10, 10, 0.5};
        box.massKilograms = 0;
        (void)mechanism.createBox(box);
        for (int tick = 0; tick < 1200; ++tick)
            mechanism.stepFixed();
        require(std::abs(mechanism.bodyState(mechanismFirst).positionMeters.z - 0.5) < 0.02,
            "Opt-out must preserve collision with static bodies.");
    }

    void lifecycle(const std::filesystem::path& root)
    {
        StaticMeshAssetCache cache{root};
        StaticMeshGpuHandleCache gpuCache;
        int uploads = 0;
        auto proof = std::make_unique<RigidBodyMechanismProof>();
        SimulationPreview preview;
        auto track = quantum::coaster::createNewDocument();
        quantum::coaster::setSectionLength(track.section(0), 1000.0);
        require(preview.rebuild(track), "Train preview must remain available.");
        preview.setRigidBodyWorld(&proof->world());
        const auto initial = proof->meshInstances();
        preview.play();
        preview.update(0.25);
        require(proof->world().tick() == 60, "Both bodies must use accepted 1/240 s preview ticks.");
        preview.pause();
        const auto paused = proof->meshInstances();
        preview.update(0.25);
        require(proof->meshInstances()[0].transform == paused[0].transform
            && proof->meshInstances()[1].transform == paused[1].transform, "Pause must freeze both GLBs.");
        preview.play();
        preview.update(0.25);
        require(proof->meshInstances()[0].transform != paused[0].transform
            && proof->meshInstances()[1].transform != paused[1].transform, "Resume must move both GLBs.");
        for (int reset = 0; reset < 140; ++reset)
        {
            preview.setRigidBodyWorld(nullptr);
            proof = std::make_unique<RigidBodyMechanismProof>();
            preview.setRigidBodyWorld(&proof->world());
            const auto instances = proof->meshInstances();
            for (std::size_t index = 0; index < instances.size(); ++index)
            {
                require(instances[index].transform == initial[index].transform,
                    "Reset must restore both initial body/asset relationships.");
                (void)gpuCache.getOrUpload(*cache.load(instances[index].assetIdentifier),
                    [&uploads](const StaticMeshAsset&) { return StaticMeshGpuHandle{static_cast<std::uint32_t>(uploads++)}; });
            }
            preview.update(1.0 / 240.0);
        }
        require(uploads == 2 && cache.size() == 2, "Reset must retain both geometry caches.");
        const auto copied = proof->meshInstances();
        preview.setRigidBodyWorld(nullptr);
        proof.reset();
        preview.update(0.25);
        require(preview.isAvailable() && copied[1].assetIdentifier == mechanicalGondolaAssetId,
            "Copied renderer values and the train must survive proof teardown.");

        for (int cycle = 0; cycle < 20; ++cycle)
        {
            RigidBodyMechanismProof mechanism;
            auto& world = mechanism.world();
            const auto bindings = mechanism.meshBindings();
            const auto removed = bindings[cycle % 2].body;
            const auto survivor = bindings[1 - cycle % 2].body;
            world.stepFixed();
            world.removeBody(removed);
            requireRejected([&] { (void)world.bodyState(removed); });
            requireRejected([&] { (void)bindings[cycle % 2].instance(world); });
            requireRejected([&] { world.removeConstraint({&world, 1}); });
            if (cycle % 2 == 0)
                requireRejected([&] { world.setHingeMotor({&world, 0}, {}); });
            else
                world.setHingeMotor({&world, 0}, {true, 1.5, 200});
            const auto replacement = world.createBox({});
            require(replacement.index == 3, "Removed handles must not alias replacement bodies.");
            world.stepFixed();
            (void)world.bodyState(survivor);
        }
        std::cout << "M5 lifecycle: pause/resume, 140 fresh-world resets, 20 alternating body removals passed.\n";
    }
}

int main(int argc, char* argv[])
{
#if defined(_MSC_VER) && defined(_DEBUG)
    _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ASSERT, _CRTDBG_FILE_STDERR);
#endif
    try
    {
        require(argc == 2, "Expected repository asset root.");
        isolatedCollisionPolicy();
        assemblyAndTrajectory(argv[1]);
        lifecycle(argv[1]);
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
