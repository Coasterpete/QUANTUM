#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/WoodenSupportGenerator.hpp>

#include <glm/geometric.hpp>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string_view>

namespace
{
    using namespace quantum::coaster;

    void require(const bool condition, const std::string_view message)
    {
        if (!condition)
        {
            throw std::runtime_error(message.data());
        }
    }

    AuthoredTrack elevatedTrack()
    {
        AuthoredTrack track = createNewDocument();
        AuthoredStartPose pose = track.startPose();
        pose.position.z = 20.0;
        track.setStartPose(pose);
        return track;
    }

    bool hasMember(const SupportStructure& structure,
        const SupportElementId a, const SupportElementId b)
    {
        return std::any_of(structure.members.begin(), structure.members.end(),
            [a, b](const SupportMember& member)
            {
                return (member.startNodeId == a && member.endNodeId == b)
                    || (member.startNodeId == b && member.endNodeId == a);
            });
    }

    void deterministicConnectedRun()
    {
        const WoodenSupportRunRecipe recipe{5.0, 25.0, 5.0, 4.0,
            0.0, -0.5, 0.2, true};
        AuthoredTrack first = elevatedTrack();
        AuthoredTrack second = elevatedTrack();
        static_cast<void>(generateWoodenSupportRun(first, recipe));
        static_cast<void>(generateWoodenSupportRun(second, recipe));
        require(first.supports() == second.supports(),
            "identical inputs must produce identical supports");
        const SupportStructure& structure = first.supports().structures.front();
        require(structure.generatedWoodenRun == recipe,
            "generated provenance and recipe must be stored");
        require(structure.nodes.size() == 20,
            "20-unit range with 5-unit spacing needs five bents");
        require(structure.members.size() == 49,
            "five bents need connected longitudinal members and braces");
        AuthoredTrack repeated = first;
        static_cast<void>(generateWoodenSupportRun(
            repeated, recipe, structure.id));
        require(repeated.supports() == first.supports(),
            "regeneration with the same recipe must reproduce the topology");
        for (std::size_t i = 0; i < 5; ++i)
        {
            const auto& baseLeft = structure.nodes[i * 4];
            const auto& baseRight = structure.nodes[i * 4 + 1];
            const auto& topLeft = structure.nodes[i * 4 + 2];
            const auto& topRight = structure.nodes[i * 4 + 3];
            require(baseLeft.foundation && baseRight.foundation,
                "each bent needs two foundations");
            require(topLeft.trackAttachment && topRight.trackAttachment,
                "each bent needs two track attachments");
            require(topLeft.trackAttachment->station >= recipe.startStation
                && topLeft.trackAttachment->station <= recipe.endStation,
                "generated bents must stay inside the range");
            if (i > 0)
            {
                require(hasMember(structure,
                    structure.nodes[(i - 1) * 4 + 2].id, topLeft.id),
                    "left upper longitudinal must connect consecutive bents");
                require(hasMember(structure,
                    structure.nodes[(i - 1) * 4 + 3].id, topRight.id),
                    "right upper longitudinal must connect consecutive bents");
            }
        }
        for (const SupportMember& member : structure.members)
        {
            const auto start = std::find_if(structure.nodes.begin(),
                structure.nodes.end(), [&](const SupportNode& node)
                { return node.id == member.startNodeId; });
            const auto end = std::find_if(structure.nodes.begin(),
                structure.nodes.end(), [&](const SupportNode& node)
                { return node.id == member.endNodeId; });
            require(glm::length(start->position - end->position) > 1e-6,
                "every member must be nondegenerate");
        }
        const auto restored = deserializeCoasterDocument(
            serializeCoasterDocument(first));
        require(restored.has_value()
            && restored->supports() == first.supports(),
            "generated structure must survive document serialization");
    }

    void curvedBankedAndPreserved()
    {
        AuthoredTrack track = createDefaultAuthoredTrack();
        AuthoredStartPose pose = track.startPose();
        pose.position.z = 50.0;
        track.setStartPose(pose);
        const SupportStructureId manualId = track.createSupportStructure("Manual");
        static_cast<void>(track.createSupportNode(
            manualId, {1.0, 2.0, 3.0}));
        const SupportStructure manual = track.supports().structures.front();
        WoodenSupportRunRecipe recipe{10.0, 30.0, 5.0,
            4.0, -50.0, -0.5, 0.2, false};
        const SupportStructureId generatedId =
            generateWoodenSupportRun(track, recipe);
        const auto& structure = track.supports().structures.back();
        require(structure.nodes.size() == 20 && structure.members.size() == 41,
            "unbraced curved run must retain longitudinal connections");
        const auto left = structure.nodes[4 + 2].position;
        const auto right = structure.nodes[4 + 3].position;
        require(std::abs(left.z - right.z) > 1e-4,
            "banked frame must tilt the upper bent beam");
        const auto firstCenter = (structure.nodes[2].position
            + structure.nodes[3].position) * 0.5;
        const auto lastCenter = (structure.nodes[18].position
            + structure.nodes[19].position) * 0.5;
        require(std::abs(firstCenter.y - lastCenter.y) > 1e-3,
            "curved centerline must move bent positions laterally");
        require(track.supports().structures.front() == manual,
            "new generation must preserve manual structures");
        recipe.bentSpacing = 10.0;
        static_cast<void>(generateWoodenSupportRun(
            track, recipe, generatedId));
        require(track.supports().structures.size() == 2
            && track.supports().structures.front() == manual
            && track.supports().structures.back().nodes.size() == 12,
            "regeneration must replace only the selected generated structure");
    }
}

int main(const int argc, char* argv[])
{
    try
    {
        deterministicConnectedRun();
        curvedBankedAndPreserved();
        if (argc >= 2)
        {
            AuthoredTrack preview = createDefaultAuthoredTrack();
            AuthoredStartPose pose = preview.startPose();
            pose.position.z = 50.0;
            preview.setStartPose(pose);
            static_cast<void>(generateWoodenSupportRun(preview,
                {5.0, 85.0, 5.0, 4.0, -10.0, -0.5, 0.2, true}));
            std::ofstream output(argv[1]);
            output << serializeCoasterDocument(preview);
            require(output.good(), "could not write preview document");
            if (argc >= 3)
            {
                AuthoredTrack detail = createDefaultAuthoredTrack();
                setSectionLength(detail.section(0), 50.0);
                detail.setStartPose(pose);
                static_cast<void>(generateWoodenSupportRun(detail,
                    {5.0, 40.0, 5.0, 4.0, -10.0, -0.5, 0.2, true}));
                std::ofstream detailOutput(argv[2]);
                detailOutput << serializeCoasterDocument(detail);
                require(detailOutput.good(), "could not write detail preview");
            }
        }
        std::cout << "Wooden support generator tests passed.\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
