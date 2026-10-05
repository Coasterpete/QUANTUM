#include <quantum/editor/RigidBodyMechanismProof.hpp>
#include <quantum/editor/SimulationPreview.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
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

    glm::dvec3 transformPoint(const glm::mat4& transform, const glm::dvec3& point)
    {
        return glm::dvec3{transform * glm::vec4{glm::vec3{point}, 1.0F}};
    }

    void localTransformComposition()
    {
        quantum::physics::RigidBodyWorld world;
        quantum::physics::RigidBodyBoxSettings box;
        box.massKilograms = 0.0;
        box.positionMeters = {10, 20, 30};
        box.orientation = glm::angleAxis(std::numbers::pi / 2.0, glm::dvec3{0, 0, 1});
        const RigidBodyMeshBinding binding{world.createBox(box), std::string{mechanicalArmAssetId},
            glm::translate(glm::dmat4{1.0}, glm::dvec3{-2, 0, 0})
                * glm::rotate(glm::dmat4{1.0}, std::numbers::pi / 2.0, glm::dvec3{1, 0, 0})
                * glm::scale(glm::dmat4{1.0}, glm::dvec3{2, 3, 4})};
        const auto instance = binding.instance(world);
        require(instance.assetIdentifier == mechanicalArmAssetId, "Binding must preserve asset identity.");
        require(glm::length(transformPoint(instance.transform, {1, 1, 1})
            - glm::dvec3{14, 20, 33}) < 1e-5,
            "Local scale/rotation/offset must precede body rotation/translation.");
        quantum::physics::RigidBodyWorld foreign;
        bool rejected = false;
        try { (void)binding.instance(foreign); }
        catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "Binding must reject a foreign world.");
        world.removeBody(binding.body);
        rejected = false;
        try { (void)binding.instance(world); }
        catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "Removed body must not produce a visual pose.");
    }

    void assetAndMotion(const std::filesystem::path& root)
    {
        StaticMeshAssetCache cache{root};
        const auto asset = cache.load(mechanicalArmAssetId);
        require(asset == cache.load(mechanicalArmAssetId) && cache.size() == 1,
            "Immutable GLB must be shared by normalized identity.");
        require(asset->submeshes.size() == 1 && asset->submeshes[0].material.has_value(),
            "Placeholder must import through the existing one-mesh material path.");
        glm::vec3 minimum{1e9F}, maximum{-1e9F};
        for (const auto& vertex : asset->vertices)
        {
            minimum = glm::min(minimum, vertex.position);
            maximum = glm::max(maximum, vertex.position);
        }
        require(glm::length(minimum - glm::vec3{0, -0.22F, -0.22F}) < 1e-5F
            && glm::length(maximum - glm::vec3{4, 0.22F, 0.22F}) < 1e-5F,
            "GLB must have a pivot-end origin and fit the M3 collider in meters/+Z-up.");

        RigidBodyMechanismProof proof;
        require(proof.armMeshBinding().body.world == &proof.world()
            && proof.armMeshBinding().body.index == 1, "Arm binding must reference the driven body.");
        StaticMeshGpuHandleCache gpuCache;
        int uploads = 0;
        double maximumDrift = 0.0;
        for (int tick = 0; tick < 2400; ++tick)
        {
            proof.world().stepFixed();
            const auto instance = proof.armMeshInstance();
            const auto body = proof.world().bodyState(proof.armMeshBinding().body);
            maximumDrift = std::max(maximumDrift,
                glm::length(transformPoint(instance.transform, {0, 0, 0}) - glm::dvec3{0, -12, 6}));
            require(maximumDrift < 0.002, "Mesh origin must remain at the actual hinge within 2 mm.");
            for (const auto& vertex : asset->vertices)
            {
                const auto local = glm::dvec3{vertex.position} - glm::dvec3{2, 0, 0};
                require(glm::length(transformPoint(instance.transform, vertex.position)
                    - (body.positionMeters + body.orientation * local)) < 3e-6,
                    "Every imported mesh vertex must follow bodyState with only float rounding.");
            }
            (void)gpuCache.getOrUpload(*cache.load(instance.assetIdentifier),
                [&uploads](const StaticMeshAsset&) { ++uploads; return StaticMeshGpuHandle{0}; });
        }
        require(uploads == 1, "Motion must retain the existing GPU cache handle.");
        std::cout << "GLB: " << asset->vertices.size() << " vertices; 2400 poses; maximum pivot drift "
            << maximumDrift << " m; one cached upload.\n";
    }

    void resetAndTeardown()
    {
        auto proof = std::make_unique<RigidBodyMechanismProof>();
        SimulationPreview preview;
        auto track = quantum::coaster::createNewDocument();
        quantum::coaster::setSectionLength(track.section(0), 1000.0);
        require(preview.rebuild(track), "Train preview must remain available.");
        preview.setRigidBodyWorld(&proof->world());
        const auto initial = proof->armMeshInstance();
        preview.play();
        preview.update(0.25);
        preview.pause();
        const auto paused = proof->armMeshInstance();
        preview.update(0.25);
        require(proof->armMeshInstance().transform == paused.transform, "Pause must freeze mesh pose.");
        preview.play();
        preview.update(0.25);
        require(proof->armMeshInstance().transform != paused.transform, "Resume must change the pose.");
        for (int reset = 0; reset < 140; ++reset)
        {
            preview.setRigidBodyWorld(nullptr);
            proof = std::make_unique<RigidBodyMechanismProof>();
            preview.setRigidBodyWorld(&proof->world());
            const auto instance = proof->armMeshInstance();
            require(instance.transform == initial.transform && instance.assetIdentifier == initial.assetIdentifier,
                "Fresh proof must rebuild its binding at the initial pose.");
            preview.update(1.0 / 240.0);
        }
        // Renderer-facing values contain no borrowed world/body pointers.
        const auto copiedInstance = proof->armMeshInstance();
        preview.setRigidBodyWorld(nullptr);
        proof.reset();
        require(copiedInstance.assetIdentifier == mechanicalArmAssetId, "Copied visual survives owner teardown.");
        preview.update(0.25);
        require(preview.isAvailable(), "Train remains independent after proof teardown.");
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
        localTransformComposition();
        assetAndMotion(argv[1]);
        resetAndTeardown();
        std::cout << "Rigid-body mesh binding: 3 scenarios passed.\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
