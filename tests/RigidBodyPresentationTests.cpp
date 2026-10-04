#include <quantum/editor/RigidBodyMechanismProof.hpp>
#include <quantum/editor/SimulationPreview.hpp>

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>

#if defined(_MSC_VER) && defined(_DEBUG)
#include <crtdbg.h>
#endif

namespace
{
    using namespace quantum::editor;

    void require(const bool condition, const std::string& message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

    glm::dvec3 point(const quantum::renderer::LineVertex& vertex)
    {
        return {vertex.x, vertex.y, vertex.z};
    }

    void boxConversion()
    {
        RigidBodyProofBox box;
        box.positionMeters = {10, 20, 30};
        box.halfExtentsMeters = {2, 3, 4};
        box.orientation = glm::angleAxis(std::numbers::pi / 2.0, glm::dvec3{0, 0, 1});
        box.role = RigidBodyProofRole::DrivenArm;
        std::vector<quantum::renderer::LineVertex> vertices;
        appendRigidBodyProofVertices(vertices, std::span{&box, 1});
        require(vertices.size() == 24, "Box needs twelve complete edges.");
        require(glm::length(point(vertices[0]) - glm::dvec3{13, 18, 26}) < 1e-5,
            "Quaternion must rotate scaled local corner before world translation.");
        require(glm::length(point(vertices[1]) - glm::dvec3{13, 22, 26}) < 1e-5,
            "Full X dimension must be twice half extent, rotated to world Y.");
        glm::dvec3 minimum{1e9}, maximum{-1e9};
        for (const auto& vertex : vertices)
        {
            minimum = glm::min(minimum, point(vertex));
            maximum = glm::max(maximum, point(vertex));
        }
        require(glm::length(maximum - minimum - glm::dvec3{6, 4, 8}) < 1e-5,
            "Rotated box must have expected full dimensions with no axis swap.");
        const auto count = vertices.size();
        appendRigidBodyProofVertices(vertices, {});
        require(vertices.size() == count, "Empty proof must preserve existing train entries.");
        appendRigidBodyProofVertices(vertices, std::span{&box, 1});
        require(vertices.size() == count * 2, "Proof entries must append instead of replacing train entries.");
    }

    void mechanismStateBridge()
    {
        RigidBodyMechanismProof proof;
        const auto initial = proof.snapshot();
        require(initial[0].role == RigidBodyProofRole::Support
            && initial[1].role == RigidBodyProofRole::DrivenArm, "Expected support and arm.");
        double maximumPivotDrift = 0.0;
        for (int tick = 0; tick < 2400; ++tick)
        {
            proof.world().stepFixed();
            const auto boxes = proof.snapshot();
            // The dedicated setup creates exactly these two bodies in order.
            for (std::size_t index = 0; index < boxes.size(); ++index)
            {
                const auto physics = proof.world().bodyState({&proof.world(), index});
                require(boxes[index].positionMeters == physics.positionMeters
                    && boxes[index].orientation == physics.orientation,
                    "Snapshot must copy actual RigidBodyWorld pose without animation/interpolation.");
            }
            require(boxes[0].positionMeters == initial[0].positionMeters
                && boxes[0].orientation == initial[0].orientation, "Static support must stay fixed.");
            const auto pivot = boxes[1].positionMeters
                + boxes[1].orientation * glm::dvec3{-2, 0, 0};
            maximumPivotDrift = std::max(maximumPivotDrift,
                glm::length(pivot - glm::dvec3{0, -12, 6}));
            require(maximumPivotDrift < 0.002, "Rendered end must stay at world hinge within 2 mm.");
            require(glm::length(boxes[1].orientation * glm::dvec3{0, 1, 0}
                - glm::dvec3{0, 1, 0}) < 1e-5, "Hinge must rotate around world Y.");
            std::vector<quantum::renderer::LineVertex> vertices;
            appendRigidBodyProofVertices(vertices, boxes);
            const auto expectedCorner = boxes[1].positionMeters
                + boxes[1].orientation * -boxes[1].halfExtentsMeters;
            require(glm::length(point(vertices[24]) - expectedCorner) < 2e-6,
                "Moving wire vertex must follow physics pose with only float rounding.");
        }
        require(std::abs(proof.armAngularSpeedRadiansPerSecond() - 1.5) < 0.05,
            "Arm must reach the hinge motor's target speed.");
        require(glm::length(proof.snapshot()[1].positionMeters - initial[1].positionMeters) > 1.0,
            "Snapshot must visibly change after stepping.");
        bool rejectedThirdBody = false;
        try { (void)proof.world().bodyState({&proof.world(), 2}); }
        catch (const std::invalid_argument&) { rejectedThirdBody = true; }
        require(rejectedThirdBody, "Proof should contain only two bodies.");
        bool rejectedSecondConstraint = false;
        try { proof.world().removeConstraint({&proof.world(), 1}); }
        catch (const std::invalid_argument&) { rejectedSecondConstraint = true; }
        require(rejectedSecondConstraint, "Proof should contain only one constraint.");
        std::cout << "10 s / 2400 ticks: maximum rendered pivot drift "
            << maximumPivotDrift << " m; arm "
            << proof.armAngularSpeedRadiansPerSecond() << " rad/s\n";
    }

    void previewCadenceAndLifecycle()
    {
        auto proof = std::make_unique<RigidBodyMechanismProof>();
        SimulationPreview preview;
        auto track = quantum::coaster::createNewDocument();
        quantum::coaster::setSectionLength(track.section(0), 1000.0);
        preview.setRigidBodyWorld(&proof->world());
        require(preview.rebuild(track), "Train preview must remain available.");
        const auto initial = proof->snapshot();
        preview.play();
        preview.update(0.25);
        require(proof->world().tick() == 60, "Proof must use accepted preview ticks.");
        preview.pause();
        const auto paused = proof->snapshot();
        preview.update(0.25);
        require(proof->world().tick() == 60 && proof->snapshot()[1].orientation == paused[1].orientation,
            "Pause must stop both physics and presented pose.");
        preview.play();
        preview.update(0.25);
        require(proof->world().tick() == 120, "Resume must continue the same world.");
        preview.update(0.25, true);
        require(proof->world().tick() == 120, "Host interruption must not advance proof.");
        preview.reset();
        require(proof->world().tick() == 120, "Train reset must preserve independently owned proof state.");
        for (int reset = 0; reset < 140; ++reset)
        {
            preview.setRigidBodyWorld(nullptr);
            proof = std::make_unique<RigidBodyMechanismProof>();
            preview.setRigidBodyWorld(&proof->world());
            require(proof->world().tick() == 0
                && proof->snapshot()[1].positionMeters == initial[1].positionMeters,
                "Fresh-world reset must restore pose without exhausting stale constraint slots.");
        }
        preview.setRigidBodyWorld(nullptr);
        proof.reset();
        preview.play();
        preview.update(0.25);
        require(preview.isAvailable() && !preview.vertices().empty(),
            "Train must keep running after safe proof removal.");
    }
}

int main()
{
#if defined(_MSC_VER) && defined(_DEBUG)
    _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ASSERT, _CRTDBG_FILE_STDERR);
#endif
    try
    {
        boxConversion();
        mechanismStateBridge();
        previewCadenceAndLifecycle();
        std::cout << "Rigid-body presentation: 3 scenarios passed.\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
