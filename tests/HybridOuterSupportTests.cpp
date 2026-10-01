#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/SupportSolidGeometry.hpp>
#include <quantum/coaster/WoodenSupportGenerator.hpp>
#include <quantum/editor/AuthoredTrackEditTransaction.hpp>
#include <quantum/editor/DocumentHistory.hpp>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <set>
#include <stdexcept>
#include <string_view>

namespace
{
    using namespace quantum;
    using namespace quantum::coaster;
    using json = nlohmann::json;

    void require(const bool condition, const std::string_view message)
    {
        if (!condition) throw std::runtime_error(std::string(message));
    }

    bool near(const glm::dvec3& a, const glm::dvec3& b)
    {
        return glm::length(a - b) < 1e-9;
    }

    AuthoredTrack sourceTrack(const double height = 38.5)
    {
        auto track = createNewDocument();
        auto pose = track.startPose();
        pose.position.z = height;
        track.setStartPose(pose);
        setSectionLength(track.section(0), 31.0);
        return track;
    }

    WoodenSupportRunRecipe recipe()
    {
        WoodenSupportRunRecipe value{0, 30, 7.5, 5, 0, -.5, .5, true};
        value.family = TimberSupportFamily::HybridTimberLattice;
        value.storyHeight = 8;
        value.hybridArchetype = HybridFramingArchetype::ConnectedTowers;
        return value;
    }

    const SupportNode& node(const SupportStructure& structure, const SupportElementId id)
    {
        const auto found = std::find_if(structure.nodes.begin(), structure.nodes.end(),
            [id](const auto& value) { return value.id == id; });
        require(found != structure.nodes.end(), "missing support node");
        return *found;
    }

    const SupportMemberPlacement& placed(const std::vector<SupportMemberPlacement>& placements,
        const SupportElementId id)
    {
        const auto found = std::find_if(placements.begin(), placements.end(),
            [id](const auto& value) { return value.memberId == id; });
        require(found != placements.end(), "missing member placement");
        return *found;
    }

    std::size_t foundationCount(const SupportStructure& structure)
    {
        return std::count_if(structure.nodes.begin(), structure.nodes.end(),
            [](const auto& value) { return value.foundation.has_value(); });
    }

    void unchangedCore(const SupportStructure& before, const SupportStructure& after)
    {
        require(after.nodes.size() >= before.nodes.size() && after.members.size() >= before.members.size(),
            "outer lines cannot remove inner framing");
        require(std::equal(before.nodes.begin(), before.nodes.end(), after.nodes.begin())
            && std::equal(before.members.begin(), before.members.end(), after.members.begin()),
            "outer selection must preserve inner IDs, upright geometry, foundations and mounting intent exactly");
        const auto oldPlacements = resolveSupportMemberPlacements(before);
        const auto newPlacements = resolveSupportMemberPlacements(after);
        for (const auto& old : oldPlacements)
            require(near(old.start, placed(newPlacements, old.memberId).start)
                && near(old.end, placed(newPlacements, old.memberId).end),
                "additional ledger contacts cannot displace existing inner physical members");
        for (const auto& member : before.members)
            if (member.role == SupportMemberRole::PrimaryPost
                && !node(before, member.endNodeId).trackAttachment)
            {
                const auto delta = node(after, member.endNodeId).position - node(after, member.startNodeId).position;
                require(std::abs(delta.x) < 1e-9 && std::abs(delta.y) < 1e-9,
                    "the lower inner posts must remain upright under banking");
            }
    }

    // Trace each new primary chain from its own foundation and check its
    // connections against the baseline's actual ledgers, independently of
    // generator row indices or its interpolation implementation.
    void inspectOuterLines(const SupportStructure& core, const SupportStructure& structure,
        const std::size_t expectedLines)
    {
        std::set<SupportElementId> ledgerNodes;
        for (const auto& member : core.members)
            if (member.role == SupportMemberRole::LedgerCap)
                for (const auto id : {member.startNodeId, member.endNodeId})
                    if (!node(core, id).trackAttachment) ledgerNodes.insert(id);
        const auto placements = resolveSupportMemberPlacements(structure);
        std::size_t lines = 0;
        std::size_t primarySegments = 0;
        for (const auto& root : structure.nodes)
        {
            if (!root.foundation || root.id < core.nextElementId) continue;
            ++lines;
            auto current = root.id;
            std::optional<glm::dvec3> axis;
            std::size_t segments = 0;
            while (true)
            {
                const auto post = std::find_if(structure.members.begin(), structure.members.end(),
                    [current](const auto& value)
                    { return value.startNodeId == current && value.role == SupportMemberRole::PrimaryPost; });
                if (post == structure.members.end()) break;
                require(++segments <= 65, "outer primary chain must terminate without a cycle");
                ++primarySegments;
                const auto& end = node(structure, post->endNodeId);
                const auto delta = end.position - node(structure, current).position;
                const auto direction = glm::normalize(delta);
                if (!axis) axis = direction;
                require(near(direction, *axis) && std::abs(direction.z) < 1.0 - 1e-9,
                    "an outer line must be straight and inclined throughout its local chain");
                require(end.id >= core.nextElementId && !end.foundation && !end.trackAttachment,
                    "outer primary nodes must remain separate from inner chains and banked attachments");
                require(post->orientation == SupportMemberOrientation::BentPost
                    && post->profile.outerDimensions == glm::dvec2(.5)
                    && post->profile.shape == SupportMemberProfileShape::Rectangular
                    && post->profile.wallThickness == 0,
                    "inclined lines must use the primary role and full solid primary section");
                require(post->orientationReference
                    && std::abs(glm::dot(*post->orientationReference, direction)) < 1e-9,
                    "outer primary sections need their own perpendicular directed frame evidence");
                require(!post->startConnection && !post->endConnection,
                    "primary mounting hosts must stay on their logical foundation-rooted chains");
                require(near(placed(placements, post->id).start, node(structure, current).position)
                    && near(placed(placements, post->id).end, end.position),
                    "primary physical endpoints must remain on their own line");
                const auto ledger = std::find_if(structure.members.begin(), structure.members.end(),
                    [&](const auto& value)
                    { return value.role == SupportMemberRole::LedgerCap
                        && (value.startNodeId == end.id || value.endNodeId == end.id); });
                require(ledger != structure.members.end(), "every outer chain junction needs a deliberate local ledger connection");
                const auto inner = ledger->startNodeId == end.id ? ledger->endNodeId : ledger->startNodeId;
                require(ledgerNodes.contains(inner) && std::abs(node(core, inner).position.z - end.position.z) < 1e-9,
                    "outer connections must meet existing inner ledgers at their actual elevations");
                require(ledger->startConnection && ledger->startConnection->mounting
                    && ledger->endConnection && ledger->endConnection->mounting
                    && ledger->startConnection->mounting->coverage == SupportMemberEndCoverage::OutsideSupport,
                    "outer ledgers must use the existing physical face contact and end coverage system");
                current = end.id;
            }
            require(segments >= 1, "each outer foundation must carry its own primary chain");
        }
        require(lines == expectedLines && foundationCount(structure) == foundationCount(core) + expectedLines,
            "only the explicitly selected sides and towers may gain independent foundations");
        require(std::count_if(structure.members.begin(), structure.members.end(),
            [&](const auto& value) { return value.id >= core.nextElementId && value.role == SupportMemberRole::PrimaryPost; })
            == static_cast<std::ptrdiff_t>(primarySegments), "all added primary members must belong to founded outer chains");
        for (const auto& member : structure.members)
            if (member.id >= core.nextElementId)
                require(member.role == SupportMemberRole::PrimaryPost || member.role == SupportMemberRole::LedgerCap,
                    "outer topology must not add secondary braces or unrelated framing");
        const auto presentation = buildSupportSolidPresentation(structure);
        require(presentation.memberCount() == structure.members.size()
            && presentation.foundations.size() == foundationCount(structure),
            "existing solid placement must render every primary line and independent pad");
    }

    void localSelectionAndIndependentFoundations()
    {
        for (const auto archetype : {HybridFramingArchetype::SimpleBent, HybridFramingArchetype::ConnectedTowers})
            for (const double height : {8.5, 38.5})
            {
                auto value = recipe();
                value.hybridArchetype = archetype;
                auto coreTrack = sourceTrack(height);
                static_cast<void>(generateWoodenSupportRun(coreTrack, value));
                const auto core = coreTrack.supports().structures[0];
                for (const auto sides : {HybridOuterSupportSides::None, HybridOuterSupportSides::Left,
                    HybridOuterSupportSides::Right, HybridOuterSupportSides::Both})
                {
                    value.hybridOuterSupports = {{2, sides, 6.25, 1.25}};
                    auto track = sourceTrack(height);
                    static_cast<void>(generateWoodenSupportRun(track, value));
                    const auto& structure = track.supports().structures[0];
                    unchangedCore(core, structure);
                    const std::size_t expected = sides == HybridOuterSupportSides::None ? 0
                        : sides == HybridOuterSupportSides::Both ? 2 : 1;
                    inspectOuterLines(core, structure, expected);
                    for (const auto& foundation : structure.nodes)
                        if (foundation.foundation && foundation.id >= core.nextElementId)
                        {
                            require(near(foundation.position, {15, foundation.position.y, 0})
                                && std::abs(std::abs(foundation.position.y) - 8.75) < 1e-9,
                                "authored outer footings must use an additive local distance, not the old 1.5x base splay");
                            require((foundation.position.y < 0 && sides != HybridOuterSupportSides::Right)
                                || (foundation.position.y > 0 && sides != HybridOuterSupportSides::Left),
                                "left/right must retain the directed lane convention");
                        }
                }
            }
    }

    void deterministicAndDormantChoices()
    {
        auto value = recipe();
        value.hybridOuterSupports = {{3, HybridOuterSupportSides::Both, 9, 1.25},
            {0, HybridOuterSupportSides::Left, 4, 1}, {1, HybridOuterSupportSides::None, 5, 1}};
        auto first = sourceTrack();
        static_cast<void>(generateWoodenSupportRun(first, value));
        auto second = sourceTrack();
        static_cast<void>(generateWoodenSupportRun(second, value));
        require(first.supports() == second.supports(), "repeated generation must match every ID and descriptor exactly");
        std::reverse(value.hybridOuterSupports.begin(), value.hybridOuterSupports.end());
        auto reordered = sourceTrack();
        static_cast<void>(generateWoodenSupportRun(reordered, value));
        require(reordered.supports().structures[0].nodes == first.supports().structures[0].nodes
            && reordered.supports().structures[0].members == first.supports().structures[0].members,
            "selection vector order must not affect topology or directed frames");
        value.hybridOuterSupports.push_back({99, HybridOuterSupportSides::Both, 10, 2});
        static_cast<void>(generateWoodenSupportRun(reordered, value, reordered.supports().structures[0].id));
        require(reordered.supports().structures[0].nodes == first.supports().structures[0].nodes,
            "unavailable tower selections must stay dormant without remapping");
        const auto full = reordered.supports();
        value.endStation = 7.5;
        static_cast<void>(generateWoodenSupportRun(reordered, value, full.structures[0].id));
        require(foundationCount(reordered.supports().structures[0]) == 5,
            "shortening a run may generate only its still-present local selection");
        value.endStation = 30;
        static_cast<void>(generateWoodenSupportRun(reordered, value, full.structures[0].id));
        require(reordered.supports() == full, "restoring the run domain must restore dormant outer selections exactly");
    }

    void saveLoadAndHistory()
    {
        auto track = sourceTrack();
        auto value = recipe();
        const auto id = generateWoodenSupportRun(track, value);
        const auto oldText = serializeCoasterDocument(track);
        require(!json::parse(oldText)["supports"]["structures"][0]["generatedWoodenRun"].contains("hybridOuterSupports"),
            "legacy recipes must omit the additive field");
        const auto legacy = deserializeCoasterDocument(oldText);
        require(legacy && legacy->supports() == track.supports(), "old documents must retain their persisted geometry");
        editor::DocumentHistory history;
        history.reset(track);
        for (const auto sides : {HybridOuterSupportSides::Left, HybridOuterSupportSides::Right,
            HybridOuterSupportSides::Both, HybridOuterSupportSides::None})
        {
            const auto before = serializeCoasterDocument(track);
            value.hybridOuterSupports = {{2, sides, 7.25, 1.5}};
            editor::AuthoredTrackEditTransaction edit{track};
            static_cast<void>(generateWoodenSupportRun(edit.candidate(), value, id));
            edit.commit(track);
            history.record(track);
            const auto after = serializeCoasterDocument(track);
            auto loaded = deserializeCoasterDocument(after);
            require(loaded && serializeCoasterDocument(*loaded) == after,
                "all side choices, nodes, roles, frames and mounting intent must save/load exactly");
            static_cast<void>(generateWoodenSupportRun(*loaded, *loaded->supports().structures[0].generatedWoodenRun, id));
            require(serializeCoasterDocument(*loaded) == after, "loaded outer selections must regenerate exactly");
            const auto undo = history.undo();
            const auto redo = history.redo();
            require(undo && serializeCoasterDocument(*undo) == before, "Undo must restore the full previous topology and recipe");
            require(redo && serializeCoasterDocument(*redo) == after, "Redo must restore the full outer topology and recipe");
            const auto expected = resolveSupportMemberPlacements(track.supports().structures[0]);
            const auto restored = resolveSupportMemberPlacements(redo->supports().structures[0]);
            for (const auto& physical : expected)
                require(near(physical.start, placed(restored, physical.memberId).start)
                    && near(physical.end, placed(restored, physical.memberId).end), "Redo must restore resolved outer contacts");
        }
    }

    void curvedBankedOrientation()
    {
        for (const double rise : {-.3, .3})
        {
            auto base = createDefaultAuthoredTrack();
            auto pose = base.startPose();
            pose.position.z = 38.5;
            pose.orientation = glm::angleAxis(.7, glm::dvec3(0, 0, 1))
                * glm::angleAxis(-std::asin(rise), glm::dvec3(0, 1, 0))
                * glm::angleAxis(.4, glm::dvec3(1, 0, 0));
            base.setStartPose(pose);
            auto value = recipe();
            value.endStation = 20;
            value.bentSpacing = 5;
            value.hybridTransversePanels = {{0, 1, SupportMemberMountingFace::NegativeZ,
                HybridDiagonalDirection::LowerLastToUpperFirst}};
            value.hybridLongitudinalPanels = {{1, 0, 0, HybridLongitudinalBracing::SingleDiagonal}};
            auto coreTrack = base;
            static_cast<void>(generateWoodenSupportRun(coreTrack, value));
            const auto core = coreTrack.supports().structures[0];
            value.hybridOuterSupports = {{0, HybridOuterSupportSides::Left, 6.25, 1.25},
                {2, HybridOuterSupportSides::Both, 9, 1.5}};
            auto track = base;
            static_cast<void>(generateWoodenSupportRun(track, value));
            const auto& structure = track.supports().structures[0];
            unchangedCore(core, structure);
            inspectOuterLines(core, structure, 3);
            for (const auto& choice : value.hybridOuterSupports)
            {
                std::vector<const SupportNode*> feet;
                for (const auto& point : core.nodes) if (point.foundation) feet.push_back(&point);
                const auto& left = *feet[2 * choice.towerIndex];
                const auto& right = *feet[2 * choice.towerIndex + 1];
                const auto lateral = glm::normalize(right.position - left.position);
                for (const auto sign : {-1.0, 1.0})
                {
                    if (sign > 0 && choice.sides == HybridOuterSupportSides::Left) continue;
                    const auto expected = (sign < 0 ? left.position : right.position)
                        + sign * choice.foundationOutset * lateral;
                    require(std::any_of(structure.nodes.begin(), structure.nodes.end(),
                        [&](const auto& point) { return point.foundation && near(point.position, expected); }),
                        "curved/banked outer footings must use the local unbanked lateral, not world Y or rider banking");
                }
            }
            const auto placements = resolveSupportMemberPlacements(structure);
            auto reversed = structure;
            for (auto& member : reversed.members)
            {
                std::swap(member.startNodeId, member.endNodeId);
                std::swap(member.startConnection, member.endConnection);
            }
            std::reverse(reversed.members.begin(), reversed.members.end());
            std::reverse(reversed.nodes.begin(), reversed.nodes.end());
            const auto reversePlacements = resolveSupportMemberPlacements(reversed);
            for (const auto& physical : placements)
                require(near(physical.start, placed(reversePlacements, physical.memberId).end)
                    && near(physical.end, placed(reversePlacements, physical.memberId).start),
                    "independent outer host faces must survive endpoint/container reversal");
            const auto rotation = glm::angleAxis(.8, glm::normalize(glm::dvec3(1, 2, 3)));
            auto rotated = structure;
            for (auto& point : rotated.nodes) point.position = rotation * point.position;
            for (auto& member : rotated.members)
                if (member.orientationReference) member.orientationReference = rotation * *member.orientationReference;
            const auto rotatedPlacements = resolveSupportMemberPlacements(rotated);
            for (const auto& physical : placements)
                require(near(rotation * physical.start, placed(rotatedPlacements, physical.memberId).start)
                    && near(rotation * physical.end, placed(rotatedPlacements, physical.memberId).end),
                    "outer member frames and contacts must rotate with authored structural evidence");
        }
    }

    void invalidChoicesAreAtomic()
    {
        auto track = sourceTrack();
        auto value = recipe();
        value.hybridOuterSupports = {{2, HybridOuterSupportSides::Both, 8, 1}};
        const auto id = generateWoodenSupportRun(track, value);
        const auto before = serializeCoasterDocument(track);
        const auto rejectedRecipe = [&](const WoodenSupportRunRecipe& bad)
        {
            bool threw = false;
            try { static_cast<void>(generateWoodenSupportRun(track, bad, id)); }
            catch (const std::invalid_argument&) { threw = true; }
            require(threw && serializeCoasterDocument(track) == before, "invalid selections must reject atomically");
        };
        auto bad = value; bad.hybridOuterSupports.push_back(bad.hybridOuterSupports[0]); rejectedRecipe(bad);
        bad = value; bad.hybridOuterSupports[0].sides = static_cast<HybridOuterSupportSides>(255); rejectedRecipe(bad);
        bad = value; bad.family = TimberSupportFamily::TraditionalTimberBent;
        bad.hybridArchetype = HybridFramingArchetype::Automatic; rejectedRecipe(bad);
        for (const double distance : {0.0, -1.0, 1.0, std::numeric_limits<double>::infinity()})
        { bad = value; bad.hybridOuterSupports[0].foundationOutset = distance; rejectedRecipe(bad); }
        bad = value; bad.hybridOuterSupports[0].topOutset = 0; rejectedRecipe(bad);
        const auto valid = json::parse(before);
        const auto choice = valid["supports"]["structures"][0]["generatedWoodenRun"]["hybridOuterSupports"][0];
        const auto rejectedJson = [&](const json& choices)
        {
            auto text = valid;
            text["supports"]["structures"][0]["generatedWoodenRun"]["hybridOuterSupports"] = choices;
            require(!deserializeCoasterDocument(text.dump()), "malformed outer choice JSON must be rejected");
        };
        for (const json sides : {json("Automatic"), json(3), json("SixPosts")})
        { auto changed = choice; changed["sides"] = sides; rejectedJson(json::array({changed})); }
        for (const json index : {json(-1), json(1.5), json(4294967296ULL), json("2")})
        { auto changed = choice; changed["towerIndex"] = index; rejectedJson(json::array({changed})); }
        for (const char* field : {"foundationOutset", "topOutset"})
        { auto changed = choice; changed.erase(field); rejectedJson(json::array({changed})); }
        auto changed = choice; changed["baseMultiplier"] = 1.5; rejectedJson(json::array({changed}));
        changed = choice; changed["topOutset"] = 0; rejectedJson(json::array({changed}));
        rejectedJson(json::array({choice, choice}));
        rejectedJson(json::object());
    }

    void writeCaptureDocuments(const std::filesystem::path& directory)
    {
        std::filesystem::create_directories(directory);
        const auto write = [&](const char* name, const AuthoredTrack& track)
        {
            const auto text = serializeCoasterDocument(track);
            { std::ofstream output(directory / name); output << text; require(output.good(), "capture document write failed"); }
            std::ifstream input(directory / name);
            const std::string saved{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
            const auto restored = deserializeCoasterDocument(saved);
            require(restored && serializeCoasterDocument(*restored) == text, "capture file must round-trip through disk");
        };
        auto value = recipe();
        value.hybridArchetype = HybridFramingArchetype::SimpleBent;
        auto low = sourceTrack(8.5);
        static_cast<void>(generateWoodenSupportRun(low, value));
        write("low-none.quantum", low);
        value.hybridOuterSupports = {{2, HybridOuterSupportSides::Left, 4, 1}};
        auto one = sourceTrack(8.5);
        static_cast<void>(generateWoodenSupportRun(one, value));
        write("low-left.quantum", one);
        value = recipe();
        auto tallBefore = sourceTrack();
        static_cast<void>(generateWoodenSupportRun(tallBefore, value));
        write("tall-none.quantum", tallBefore);
        value.hybridOuterSupports = {{2, HybridOuterSupportSides::Both, 10, 1}};
        auto tall = sourceTrack();
        static_cast<void>(generateWoodenSupportRun(tall, value));
        write("tall-both.quantum", tall);
    }
}

int main(const int argc, char* argv[])
{
    try
    {
        localSelectionAndIndependentFoundations();
        deterministicAndDormantChoices();
        saveLoadAndHistory();
        curvedBankedOrientation();
        invalidChoicesAreAtomic();
        if (argc == 2) writeCaptureDocuments(argv[1]);
    }
    catch (const std::exception& error)
    {
        std::cerr << "Hybrid outer support test failure: " << error.what() << '\n';
        return 1;
    }
    std::cout << "Hybrid outer support tests passed (5 groups).\n";
}
