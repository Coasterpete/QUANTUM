#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/WoodenSupportGenerator.hpp>

#include <glm/geometric.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <set>
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
        recipe.storyHeight = 200.0;
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

    void familyTopologiesAndTallStories()
    {
        const TimberSupportFamily families[] = {
            TimberSupportFamily::TraditionalTimberBent,
            TimberSupportFamily::ModernTwisterTimber,
            TimberSupportFamily::PrefabricatedTimberLattice,
            TimberSupportFamily::HybridTimberLattice};
        std::set<std::vector<std::pair<std::size_t, std::size_t>>> topologies;
        for (const TimberSupportFamily family : families)
        {
            WoodenSupportRunRecipe recipe{10.0, 30.0, 5.0,
                4.0, -50.0, -0.5, 0.2, true};
            recipe.family = family;
            recipe.storyHeight = 18.0;
            AuthoredTrack first = createDefaultAuthoredTrack();
            AuthoredStartPose pose = first.startPose();
            pose.position.z = 50.0;
            first.setStartPose(pose);
            AuthoredTrack second = first;
            const auto id = generateWoodenSupportRun(first, recipe);
            static_cast<void>(generateWoodenSupportRun(second, recipe));
            require(first.supports() == second.supports(),
                "each family must generate deterministically");
            const auto& structure = first.supports().structures.front();
            require(structure.id == id && structure.generatedWoodenRun == recipe,
                "each generated structure must retain its family recipe");
            std::vector<std::pair<std::size_t, std::size_t>> edges;
            for (const auto& member : structure.members)
            {
                const auto a = std::find_if(structure.nodes.begin(), structure.nodes.end(),
                    [&](const SupportNode& node) { return node.id == member.startNodeId; });
                const auto b = std::find_if(structure.nodes.begin(), structure.nodes.end(),
                    [&](const SupportNode& node) { return node.id == member.endNodeId; });
                edges.push_back(std::minmax(
                    static_cast<std::size_t>(a - structure.nodes.begin()),
                    static_cast<std::size_t>(b - structure.nodes.begin())));
            }
            std::sort(edges.begin(), edges.end());
            topologies.insert(std::move(edges));
            require(std::any_of(structure.nodes.begin(), structure.nodes.end(),
                [](const SupportNode& node)
                {
                    return !node.foundation && !node.trackAttachment;
                }), "a tall run needs intermediate framing nodes");
            for (const auto& member : structure.members)
            {
                const auto start = std::find_if(structure.nodes.begin(),
                    structure.nodes.end(), [&](const SupportNode& node)
                    { return node.id == member.startNodeId; });
                const auto end = std::find_if(structure.nodes.begin(),
                    structure.nodes.end(), [&](const SupportNode& node)
                    { return node.id == member.endNodeId; });
                require(glm::length(start->position - end->position) > 1e-6,
                    "family geometry must contain no degenerate members");
            }
            const auto restored = deserializeCoasterDocument(
                serializeCoasterDocument(first));
            require(restored && restored->supports() == first.supports(),
                "each family recipe must round-trip through the document");
        }
        require(topologies.size() == 4,
            "four families need distinct node/member connectivity");
    }

    void legacyAndMalformedFamilyFields()
    {
        AuthoredTrack track = elevatedTrack();
        static_cast<void>(generateWoodenSupportRun(track,
            {5.0, 25.0, 5.0, 4.0, 0.0, -0.5, 0.2, true}));
        using json = nlohmann::json;
        const json valid = json::parse(serializeCoasterDocument(track));
        json legacy = valid;
        auto& legacyRecipe = legacy["supports"]["structures"][0]
            ["generatedWoodenRun"];
        legacyRecipe.erase("family");
        legacyRecipe.erase("storyHeight");
        const auto restored = deserializeCoasterDocument(legacy.dump());
        require(restored && restored->supports() == track.supports(),
            "M0 recipe without M1 fields must preserve its saved structure");

        json unknown = valid;
        unknown["supports"]["structures"][0]["generatedWoodenRun"]
            ["family"] = "InventedFamily";
        require(!deserializeCoasterDocument(unknown.dump()),
            "unknown family identifier must be rejected");
        json numeric = valid;
        numeric["supports"]["structures"][0]["generatedWoodenRun"]
            ["family"] = 2;
        require(!deserializeCoasterDocument(numeric.dump()),
            "numeric family identifier must be rejected");
        json invalidStory = valid;
        invalidStory["supports"]["structures"][0]["generatedWoodenRun"]
            ["storyHeight"] = 0.0;
        require(!deserializeCoasterDocument(invalidStory.dump()),
            "nonpositive story height must be rejected");
        WoodenSupportRunRecipe invalid = *track.supports().structures.front()
            .generatedWoodenRun;
        invalid.storyHeight = std::numeric_limits<double>::infinity();
        try
        {
            validateWoodenSupportRunRecipe(invalid);
            throw std::runtime_error("non-finite story height was accepted");
        }
        catch (const std::invalid_argument&) {}
    }

    void mixedFamiliesPreserveOtherStructures()
    {
        AuthoredTrack track = createDefaultAuthoredTrack();
        AuthoredStartPose pose = track.startPose();
        pose.position.z = 50.0;
        track.setStartPose(pose);
        const auto manualId = track.createSupportStructure("Manual");
        static_cast<void>(track.createSupportNode(manualId, {1.0, 2.0, 3.0}));
        WoodenSupportRunRecipe twister{10.0, 30.0, 5.0,
            4.0, -10.0, -0.5, 0.2, true};
        twister.family = TimberSupportFamily::ModernTwisterTimber;
        WoodenSupportRunRecipe hybrid = twister;
        hybrid.startStation = 35.0;
        hybrid.endStation = 55.0;
        hybrid.family = TimberSupportFamily::HybridTimberLattice;
        const auto twisterId = generateWoodenSupportRun(track, twister);
        static_cast<void>(generateWoodenSupportRun(track, hybrid));
        const auto manual = track.supports().structures[0];
        const auto other = track.supports().structures[2];
        twister.bentSpacing = 4.0;
        static_cast<void>(generateWoodenSupportRun(track, twister, twisterId));
        require(track.supports().structures.size() == 3
            && track.supports().structures[0] == manual
            && track.supports().structures[2] == other
            && track.supports().structures[1].generatedWoodenRun == twister,
            "regeneration must replace only its selected family structure");
        const auto restored = deserializeCoasterDocument(
            serializeCoasterDocument(track));
        require(restored && restored->supports() == track.supports(),
            "mixed family structures must survive save/load");
    }

    void rapidGeometryChangesTightenBents()
    {
        for (const TimberSupportFamily family : {
            TimberSupportFamily::ModernTwisterTimber,
            TimberSupportFamily::HybridTimberLattice})
        {
            AuthoredTrack track = createDefaultAuthoredTrack();
            setSectionLength(track.section(0), 60.0);
            AuthoredStartPose pose = track.startPose();
            pose.position.z = 45.0;
            track.setStartPose(pose);
            WoodenSupportRunRecipe recipe{5.0, 55.0, 5.0, 4.0,
                -10.0, -0.5, 0.2, true};
            recipe.family = family;
            static_cast<void>(generateWoodenSupportRun(track, recipe));
            std::vector<double> stations;
            for (const auto& node : track.supports().structures.front().nodes)
            {
                if (node.trackAttachment
                    && node.trackAttachment->lateralOffset < 0.0)
                {
                    stations.push_back(node.trackAttachment->station);
                }
            }
            bool shortened = false;
            for (std::size_t index = 1; index < stations.size(); ++index)
            {
                shortened |= stations[index] - stations[index - 1]
                    < recipe.bentSpacing - 1e-6;
            }
            require(shortened,
                "adaptive family needs denser stations on changing geometry");
        }
    }

    void hybridOneStoryUsesOpenFramedBays()
    {
        AuthoredTrack track = elevatedTrack();
        WoodenSupportRunRecipe recipe{5.0, 25.0, 5.0, 4.0,
            0.0, -0.5, 0.2, true};
        recipe.family = TimberSupportFamily::HybridTimberLattice;
        recipe.storyHeight = 24.0;
        static_cast<void>(generateWoodenSupportRun(track, recipe));
        const auto& structure = track.supports().structures.front();
        require(structure.nodes.size() == 20,
            "one-story hybrid needs two foundations and two upper posts per bent");
        for (std::size_t bent = 0; bent < 5; ++bent)
        {
            const auto& nodes = structure.nodes;
            const auto baseLeft = nodes[bent * 4].id;
            const auto baseRight = nodes[bent * 4 + 1].id;
            const auto topLeft = nodes[bent * 4 + 2].id;
            const auto topRight = nodes[bent * 4 + 3].id;
            require(hasMember(structure, baseLeft, topLeft)
                && hasMember(structure, baseRight, topRight)
                && hasMember(structure, topLeft, topRight),
                "hybrid primary posts and cap must be continuous");
            const bool risingRight = hasMember(structure, baseLeft, topRight);
            const bool risingLeft = hasMember(structure, baseRight, topLeft);
            require(risingRight != risingLeft,
                "each hybrid bent needs exactly one transverse story diagonal");
            require(risingRight == (bent % 2 == 0),
                "hybrid diagonal direction must alternate between bents");
            if (bent > 0)
            {
                require(hasMember(structure, nodes[(bent - 1) * 4 + 2].id,
                        topLeft)
                    && hasMember(structure, nodes[(bent - 1) * 4 + 3].id,
                        topRight),
                    "hybrid upper longitudinal ties must connect adjacent bents");
                const bool leftBayBrace = hasMember(structure,
                    nodes[(bent - 1) * 4].id, topLeft);
                const bool rightBayBrace = hasMember(structure,
                    nodes[(bent - 1) * 4 + 1].id, topRight);
                require((leftBayBrace ? 1 : 0) + (rightBayBrace ? 1 : 0)
                        == (bent == 2 ? 1 : 0),
                    "only selected hybrid longitudinal bays should be braced");
            }
        }
    }
}

int main(const int argc, char* argv[])
{
    try
    {
        deterministicConnectedRun();
        curvedBankedAndPreserved();
        familyTopologiesAndTallStories();
        legacyAndMalformedFamilyFields();
        mixedFamiliesPreserveOtherStructures();
        rapidGeometryChangesTightenBents();
        hybridOneStoryUsesOpenFramedBays();
        if (argc == 6 || argc == 7)
        {
            const TimberSupportFamily families[] = {
                TimberSupportFamily::TraditionalTimberBent,
                TimberSupportFamily::ModernTwisterTimber,
                TimberSupportFamily::PrefabricatedTimberLattice,
                TimberSupportFamily::HybridTimberLattice};
            for (std::size_t index = 0; index < static_cast<std::size_t>(argc - 1);
                ++index)
            {
                AuthoredTrack comparison = createDefaultAuthoredTrack();
                setSectionLength(comparison.section(0),
                    index == 4 ? 40.0 : 60.0);
                AuthoredStartPose pose = comparison.startPose();
                pose.position.z = index == 4 ? 65.0 : index == 5 ? 25.0 : 45.0;
                comparison.setStartPose(pose);
                WoodenSupportRunRecipe recipe{5.0, index == 4 ? 35.0 : 55.0,
                    5.0, 4.0,
                    -10.0, -0.5, 0.2, true};
                recipe.family = families[std::min(index, std::size_t{3})];
                recipe.storyHeight = 16.0;
                if (index == 4)
                {
                    recipe.family = TimberSupportFamily::TraditionalTimberBent;
                    recipe.foundationElevation = 0.0;
                    recipe.storyHeight = 12.0;
                }
                if (index == 5)
                {
                    recipe.family = TimberSupportFamily::ModernTwisterTimber;
                    recipe.foundationElevation = 0.0;
                    recipe.storyHeight = 16.0;
                }
                static_cast<void>(generateWoodenSupportRun(comparison, recipe));
                std::ofstream output(argv[index + 1]);
                output << serializeCoasterDocument(comparison);
                require(output.good(), "could not write family comparison document");
            }
        }
        else if (argc >= 2)
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
