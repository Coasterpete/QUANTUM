#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/SupportSolidGeometry.hpp>
#include <quantum/coaster/Supports.hpp>
#include <quantum/coaster/WoodenSupportGenerator.hpp>

#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <unordered_map>
#include <vector>

// Consuming SupportMemberOrientation in solid rendering: authored Hybrid
// planes select the cross-section reference explicitly instead of the
// Generic angle-based fallback. Generic behavior stays bit-identical.

namespace
{
    using namespace quantum::coaster;

    void require(const bool condition, const std::string_view message)
    {
        if (!condition)
        {
            throw std::runtime_error(std::string(message));
        }
    }

    constexpr double coherence = 0.999;

    [[nodiscard]] bool near(const double a, const double b)
    {
        return std::abs(a - b) <= 1e-9;
    }

    [[nodiscard]] AuthoredTrack elevatedTrack()
    {
        AuthoredTrack track = createNewDocument();
        AuthoredStartPose pose = track.startPose();
        pose.position.z = 20.0;
        track.setStartPose(pose);
        return track;
    }

    [[nodiscard]] SupportStructure hybridOneStory()
    {
        AuthoredTrack track = elevatedTrack();
        WoodenSupportRunRecipe recipe{5.0, 25.0, 5.0, 4.0, 0.0, -0.5, 0.2,
            true};
        recipe.family = TimberSupportFamily::HybridTimberLattice;
        recipe.storyHeight = 24.0;
        static_cast<void>(generateWoodenSupportRun(track, recipe));
        return track.supports().structures.front();
    }

    [[nodiscard]] glm::dvec3 nodePosition(
        const SupportStructure& structure, const SupportElementId id)
    {
        const auto found = std::find_if(
            structure.nodes.begin(), structure.nodes.end(),
            [id](const SupportNode& node)
            {
                return node.id == id;
            });
        if (found == structure.nodes.end())
        {
            throw std::runtime_error("test fixture references unknown node");
        }
        return found->position;
    }

    [[nodiscard]] SupportMemberFrame frameOf(
        const SupportStructure& structure, const SupportMember& member)
    {
        return resolveSupportMemberFrame(
            nodePosition(structure, member.startNodeId),
            nodePosition(structure, member.endNodeId), member.orientation,
            member.orientationReference);
    }

    [[nodiscard]] SupportMemberFrame frameBetween(const SupportStructure& structure,
        const SupportElementId start, const SupportElementId end)
    {
        const auto member = std::find_if(structure.members.begin(), structure.members.end(),
            [start, end](const SupportMember& value)
            {
                return value.startNodeId == start && value.endNodeId == end;
            });
        require(member != structure.members.end(), "fixture must contain the requested member");
        return frameOf(structure, *member);
    }

    void requireRightHanded(
        const SupportMemberFrame& frame, const std::string_view message)
    {
        require(
            near(
                glm::dot(
                    glm::cross(frame.axisY, frame.axisZ), frame.axisX),
                1.0),
            message);
    }

    void postsInOneBentStayCoherent()
    {
        const SupportStructure structure = hybridOneStory();
        // Bent-0 post members resolve through their authored transverse
        // references now, not the vertical branch: same coherence, exact
        // bent frame.
        std::vector<SupportMemberFrame> frames;
        for (const SupportMember& member : structure.members)
        {
            if (member.role != SupportMemberRole::PrimaryPost)
            {
                continue;
            }
            const auto isBentPost = [&](const SupportElementId id)
            {
                return std::any_of(
                    structure.nodes.begin(), structure.nodes.begin() + 8,
                    [id](const SupportNode& node)
                    {
                        return node.id == id;
                    });
            };
            if (!isBentPost(member.startNodeId)
                || !isBentPost(member.endNodeId))
            {
                continue;
            }
            frames.push_back(frameOf(structure, member));
        }
        require(frames.size() == 6, "bent 0 needs four lower and two interface post segments");
        for (const SupportMemberFrame& frame : frames)
        {
            require(
                !frame.usedVerticalFallback,
                "referenced posts resolve from authored transverse data");
            requireRightHanded(frame, "post frames must stay right-handed");
        }
        // Lower same-lane segments are collinear; the upper interface is
        // separate. On this unbanked fixture their section axes stay coherent.
        // Upright inner posts share these axes across both lanes; the upper
        // interface must not introduce an arbitrary 90-degree roll flip.
        constexpr double noFlip = 0.99;
        for (std::size_t index = 1; index < frames.size(); ++index)
        {
            require(
                glm::dot(frames.front().axisY, frames[index].axisY) > noFlip,
                "neighboring posts must not arbitrarily rotate");
            require(
                glm::dot(frames.front().axisZ, frames[index].axisZ) > noFlip,
                "neighboring posts must share their depth axis");
        }
        for (const std::size_t lane : {0u, 2u})
        {
            require(
                glm::dot(frames[lane].axisY, frames[lane + 1].axisY)
                    > coherence,
                "same-lane post segments must continue without rolling");
        }
    }

    void batterStraddlingToleranceStaysCoherent()
    {
        // Lean A sits just outside the vertical tolerance, lean B just
        // inside: the Generic fallback straddles its angle branch here.
        const glm::dvec3 foot{0.0, 0.0, 0.0};
        const glm::dvec3 leanA{0.5, 0.0, 10.0};
        const glm::dvec3 leanB{0.02, 0.0, 10.0};
        const SupportMemberFrame genericA =
            resolveSupportMemberFrame(foot, leanA);
        const SupportMemberFrame genericB =
            resolveSupportMemberFrame(foot, leanB);
        require(
            genericA.usedVerticalFallback != genericB.usedVerticalFallback,
            "the generic fallback straddles its angle branch here");
        const SupportMemberFrame postA = resolveSupportMemberFrame(
            foot, leanA, SupportMemberOrientation::BentPost);
        const SupportMemberFrame postB = resolveSupportMemberFrame(
            foot, leanB, SupportMemberOrientation::BentPost);
        require(
            postA.usedVerticalFallback && postB.usedVerticalFallback,
            "authored posts ignore the angle threshold");
        // The leans differ by ~3 degrees, so authored frames differ by that
        // correct geometric amount -- while the straddled Generic pair rolls
        // ~180 degrees apart. No-flip means a near-one dot, not exact
        // equality: these are differently leaning posts, not parallel ones.
        require(
            glm::dot(postA.axisY, postB.axisY) > 0.99
                && glm::dot(postA.axisZ, postB.axisZ) > 0.99,
            "authored posts stay coherent across the tolerance boundary");
        require(
            glm::dot(genericA.axisY, genericB.axisY) < 0.99,
            "the straddled generic pair must show the branch flip");
    }

    void transverseCapOrientation()
    {
        const SupportStructure structure = hybridOneStory();
        const auto& nodes = structure.nodes;
        // Bent-0 top cap with its authored orientation.
        const SupportMemberFrame cap = frameBetween(structure, nodes[4].id, nodes[5].id);
        require(
            !cap.usedVerticalFallback,
            "transverse caps use the upright reference");
        require(
            glm::dot(cap.axisY, glm::dvec3{0.0, 0.0, 1.0}) > 0.99,
            "cap width stays upright");
        require(
            std::abs(glm::dot(cap.axisZ, glm::dvec3{0.0, 0.0, 1.0})) < 0.05,
            "cap depth stays horizontal");
        requireRightHanded(cap, "cap frames must stay right-handed");

        // Caps across bents agree: no 90-degree roll jumps along the run.
        for (std::size_t bent = 1; bent < 5; ++bent)
        {
            const SupportMemberFrame other = frameBetween(structure,
                structure.nodes[bent * 8 + 4].id, structure.nodes[bent * 8 + 5].id);
            require(
                glm::dot(cap.axisY, other.axisY) > coherence
                    && glm::dot(cap.axisZ, other.axisZ) > coherence,
                "transverse caps must agree along the run");
        }
    }

    void longitudinalTieOrientation()
    {
        const SupportStructure structure = hybridOneStory();
        const auto& nodes = structure.nodes;
        const SupportMemberFrame tie = frameBetween(structure, nodes[4].id, nodes[8 + 4].id);
        require(
            glm::dot(tie.axisY, glm::dvec3{0.0, 0.0, 1.0}) > 0.99,
            "longitudinal tie width stays upright");
        require(
            std::abs(glm::dot(tie.axisZ, glm::dvec3{0.0, 0.0, 1.0})) < 0.05,
            "longitudinal tie depth stays horizontal");
        requireRightHanded(tie, "tie frames must stay right-handed");

        // Upper ties in every bay agree with each other.
        for (std::size_t bent = 1; bent < 4; ++bent)
        {
            const SupportMemberFrame other = frameBetween(structure,
                structure.nodes[bent * 8 + 4].id, structure.nodes[(bent + 1) * 8 + 4].id);
            require(
                glm::dot(tie.axisY, other.axisY) > coherence
                    && glm::dot(tie.axisZ, other.axisZ) > coherence,
                "longitudinal ties must agree along the run");
        }
    }

    [[nodiscard]] const SupportMember* findBentZeroDiagonal(
        const SupportStructure& structure)
    {
        // Bent-0 lower diagonal: raised ledger to horizontal shoulder.
        const SupportElementId start = structure.nodes[6].id;
        const SupportElementId end = structure.nodes[3].id;
        const auto found = std::find_if(
            structure.members.begin(), structure.members.end(),
            [start, end](const SupportMember& member)
            {
                return (member.startNodeId == start
                        && member.endNodeId == end)
                    || (member.startNodeId == end
                        && member.endNodeId == start);
            });
        if (found == structure.members.end())
        {
            throw std::runtime_error("test fixture references unknown member");
        }
        return &*found;
    }

    void transverseBraceOrientation()
    {
        const SupportStructure structure = hybridOneStory();
        // Authored bent-normal reference: brace width reads longitudinal,
        // keeping the roll inside the bent plane by construction.
        const SupportMemberFrame brace =
            frameOf(structure, *findBentZeroDiagonal(structure));
        const auto& nodes = structure.nodes;
        const SupportMemberFrame tie = frameBetween(structure, nodes[2].id, nodes[8 + 2].id);
        require(
            std::abs(glm::dot(brace.axisY, tie.axisX)) > 0.9,
            "transverse-brace width must read as the run direction");
        requireRightHanded(brace, "transverse braces stay right-handed");

        // Same-direction diagonals in neighboring bents agree.
        const SupportMember* neighbor = nullptr;
        for (const SupportMember& member : structure.members)
        {
            if (member.role == SupportMemberRole::Brace
                && member.orientation
                    == SupportMemberOrientation::BentDiagonal
                && member.startNodeId != findBentZeroDiagonal(structure)->startNodeId)
            {
                neighbor = &member;
                break;
            }
        }
        require(neighbor != nullptr, "the fixture needs repeated diagonals");
        const SupportMemberFrame neighborFrame = frameOf(structure, *neighbor);
        require(
            glm::dot(brace.axisY, neighborFrame.axisY) > coherence
                && glm::dot(brace.axisZ, neighborFrame.axisZ) > coherence,
            "repeated transverse diagonals must agree bent to bent");
    }

    void longitudinalBraceOrientation()
    {
        const SupportStructure structure = hybridOneStory();
        const SupportMember* braced = nullptr;
        for (const SupportMember& member : structure.members)
        {
            if (member.role == SupportMemberRole::Brace
                && member.orientation
                    == SupportMemberOrientation::RunDiagonal)
            {
                braced = &member;
                break;
            }
        }
        require(braced != nullptr, "the fixture needs one braced bay");
        const SupportMemberFrame brace = frameOf(structure, *braced);
        // Authored transverse reference: brace width reads transverse,
        // keeping the roll inside the run plane by construction.
        const auto& nodes = structure.nodes;
        const SupportMemberFrame cap = frameBetween(structure, nodes[2].id, nodes[3].id);
        require(
            std::abs(glm::dot(brace.axisY, cap.axisX)) > 0.9,
            "longitudinal-brace width must read as the transverse direction");
        requireRightHanded(brace, "longitudinal braces stay right-handed");
    }

    void genericFallbackRemainsUnchanged()
    {
        const std::vector<std::pair<glm::dvec3, glm::dvec3>> cases{
            {{0.0, 0.0, 5.0}, {4.0, 0.0, 5.0}},
            {{0.0, 0.0, 5.0}, {4.0, 0.0, 6.0}},
            {{0.0, 0.0, 5.0}, {4.0, 0.0, 40.0}},
            {{1.0, 2.0, 0.0}, {1.0, 2.0, 9.0}},
            {{0.0, 0.0, 5.0}, {-3.0, 2.0, 5.0}},
            {{0.0, 0.0, 0.0}, {0.5, 0.0, 10.0}},
        };
        for (const auto& [start, end] : cases)
        {
            const SupportMemberFrame legacy =
                resolveSupportMemberFrame(start, end);
            const SupportMemberFrame generic = resolveSupportMemberFrame(
                start, end, SupportMemberOrientation::Generic);
            require(
                near(legacy.length, generic.length)
                    && near(
                        glm::length(legacy.axisX - generic.axisX), 0.0)
                    && near(
                        glm::length(legacy.axisY - generic.axisY), 0.0)
                    && near(
                        glm::length(legacy.axisZ - generic.axisZ), 0.0)
                    && near(
                        glm::length(legacy.origin - generic.origin), 0.0)
                    && legacy.usedVerticalFallback
                        == generic.usedVerticalFallback,
                "Generic must resolve exactly like the legacy call");
        }
    }

    void presentationStaysDeterministicWithHints()
    {
        const SupportStructure structure = hybridOneStory();
        const SupportSolidPresentation first =
            buildSupportSolidPresentation(structure);
        const SupportSolidPresentation second =
            buildSupportSolidPresentation(structure);
        require(
            first.memberCount() == structure.members.size(),
            "every hinted member must still present exactly once");
        require(
            first.batches.size() == second.batches.size(),
            "hinted presentation builds must agree on batches");
        for (std::size_t batch = 0; batch < first.batches.size(); ++batch)
        {
            require(
                first.batches[batch].instances.size()
                    == second.batches[batch].instances.size(),
                "hinted presentation builds must agree on instances");
            for (std::size_t index = 0;
                index < first.batches[batch].instances.size(); ++index)
            {
                require(
                    first.batches[batch].instances[index]
                        == second.batches[batch].instances[index],
                    "hinted instance transforms must be deterministic");
            }
        }
        bool rejected = false;
        try
        {
            static_cast<void>(resolveSupportMemberFrame(
                {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0},
                static_cast<SupportMemberOrientation>(255)));
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        require(rejected, "unknown orientations must fail frame resolution");
    }
}

int main()
{
    try
    {
        postsInOneBentStayCoherent();
        batterStraddlingToleranceStaysCoherent();
        transverseCapOrientation();
        longitudinalTieOrientation();
        transverseBraceOrientation();
        longitudinalBraceOrientation();
        genericFallbackRemainsUnchanged();
        presentationStaysDeterministicWithHints();
    }
    catch (const std::exception& error)
    {
        std::cerr << "Support orientation frame test failure: "
                  << error.what() << '\n';
        return 1;
    }

    std::cout << "Support orientation frame tests passed.\n";
    return 0;
}
