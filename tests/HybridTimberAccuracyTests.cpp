#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/SupportSolidGeometry.hpp>
#include <quantum/coaster/Supports.hpp>
#include <quantum/coaster/WoodenSupportGenerator.hpp>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <fstream>
#include <map>
#include <set>
#include <stdexcept>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Hybrid Timber Lattice accuracy suite.
//
// QUANTUM procedural archetypes based on observable reference geometry
// (broad two-post bents on discrete footings, a raised lower ledger, one
// repeated transverse diagonal per bent/story, upper caps, longitudinal ties
// at deliberate levels, mostly open bays) -- NOT official RMC engineering
// categories. Simple bents stay simple: single diagonals, never automatic X
// bracing, no grade-level footing ties.

namespace
{
    using namespace quantum::coaster;
    using json = nlohmann::json;

    void require(const bool condition, const std::string_view message)
    {
        if (!condition)
        {
            throw std::runtime_error(std::string(message));
        }
    }

    [[nodiscard]] AuthoredTrack elevatedTrack(const double elevation = 20.0)
    {
        AuthoredTrack track = createNewDocument();
        AuthoredStartPose pose = track.startPose();
        pose.position.z = elevation;
        track.setStartPose(pose);
        return track;
    }

    [[nodiscard]] WoodenSupportRunRecipe hybridOneStoryRecipe()
    {
        WoodenSupportRunRecipe recipe{5.0, 25.0, 5.0, 4.0, 0.0, -0.5, 0.2,
            true};
        recipe.family = TimberSupportFamily::HybridTimberLattice;
        recipe.storyHeight = 24.0;
        return recipe;
    }

    [[nodiscard]] bool hasMember(
        const SupportStructure& structure,
        const SupportElementId a,
        const SupportElementId b)
    {
        return std::any_of(
            structure.members.begin(), structure.members.end(),
            [a, b](const SupportMember& member)
            {
                return (member.startNodeId == a && member.endNodeId == b)
                    || (member.startNodeId == b && member.endNodeId == a);
            });
    }

    [[nodiscard]] const SupportMember* findMember(
        const SupportStructure& structure,
        const SupportElementId a,
        const SupportElementId b)
    {
        const auto found = std::find_if(
            structure.members.begin(), structure.members.end(),
            [a, b](const SupportMember& member)
            {
                return (member.startNodeId == a && member.endNodeId == b)
                    || (member.startNodeId == b && member.endNodeId == a);
            });
        return found == structure.members.end() ? nullptr : &*found;
    }

    // One-story node order: footings, horizontal shoulders, banked track
    // attachments, then the raised-ledger pair.
    struct OneStoryBent
    {
        SupportElementId baseLeft;
        SupportElementId baseRight;
        SupportElementId shoulderLeft;
        SupportElementId shoulderRight;
        SupportElementId topLeft;
        SupportElementId topRight;
        SupportElementId ledgerLeft;
        SupportElementId ledgerRight;
    };

    [[nodiscard]] OneStoryBent bentAt(
        const SupportStructure& structure, const std::size_t bent)
    {
        const auto& nodes = structure.nodes;
        const std::size_t offset = bent * 8;
        return {nodes[offset].id, nodes[offset + 1].id, nodes[offset + 2].id,
            nodes[offset + 3].id, nodes[offset + 4].id, nodes[offset + 5].id,
            nodes[offset + 6].id, nodes[offset + 7].id};
    }

    void simpleBentTopology()
    {
        AuthoredTrack track = elevatedTrack();
        static_cast<void>(
            generateWoodenSupportRun(track, hybridOneStoryRecipe()));
        const SupportStructure& structure =
            track.supports().structures.front();
        require(
            structure.nodes.size() == 40,
            "one-story hybrid keeps five eight-node bents");
        for (std::size_t bent = 0; bent < 5; ++bent)
        {
            const auto& nodes = structure.nodes;
            const std::size_t offset = bent * 8;
            require(
                nodes[offset].foundation.has_value()
                    && nodes[offset + 1].foundation.has_value(),
                "hybrid footings sit under the primary posts only");
            require(
                !nodes[offset + 2].foundation.has_value()
                    && !nodes[offset + 3].foundation.has_value()
                    && !nodes[offset + 4].foundation.has_value()
                    && !nodes[offset + 5].foundation.has_value()
                    && !nodes[offset + 6].foundation.has_value()
                    && !nodes[offset + 7].foundation.has_value(),
                "ledger, intermediate, and track nodes never carry footings");
            require(
                nodes[offset + 4].trackAttachment.has_value()
                    && nodes[offset + 5].trackAttachment.has_value(),
                "hybrid upper nodes stay track-attached");
            require(
                !nodes[offset + 2].trackAttachment.has_value()
                    && !nodes[offset + 3].trackAttachment.has_value()
                    && !nodes[offset + 6].trackAttachment.has_value()
                    && !nodes[offset + 7].trackAttachment.has_value(),
                "the raised ledger stays unattached");

            const OneStoryBent ids = bentAt(structure, bent);
            // Posts split at the raised ledger; ledger and cap span the bent.
            require(
                hasMember(structure, ids.baseLeft, ids.ledgerLeft)
                    && hasMember(structure, ids.ledgerLeft, ids.shoulderLeft)
                    && hasMember(structure, ids.shoulderLeft, ids.topLeft)
                    && hasMember(structure, ids.baseRight, ids.ledgerRight)
                    && hasMember(structure, ids.ledgerRight, ids.shoulderRight)
                    && hasMember(structure, ids.shoulderRight, ids.topRight)
                    && hasMember(structure, ids.ledgerLeft, ids.ledgerRight)
                    && hasMember(structure, ids.topLeft, ids.topRight),
                "hybrid posts run footing-ledger-cap with a raised ledger "
                "and a top cap");
            // Exactly one repeated transverse diagonal per bent, same
            // direction in neighboring bents: never an automatic X.
            require(
                hasMember(structure, ids.ledgerLeft, ids.shoulderRight),
                "the simple bent carries its single repeated diagonal");
            require(
                !hasMember(structure, ids.ledgerRight, ids.shoulderLeft)
                    && !hasMember(structure, ids.baseLeft, ids.topRight)
                    && !hasMember(structure, ids.baseRight, ids.topLeft)
                    && !hasMember(structure, ids.baseLeft, ids.topLeft)
                    && !hasMember(structure, ids.baseRight, ids.topRight),
                "the simple archetype must not X-brace or bypass the ledger");
        }
    }

    void memberRolesAndOrientations()
    {
        AuthoredTrack track = elevatedTrack();
        static_cast<void>(
            generateWoodenSupportRun(track, hybridOneStoryRecipe()));
        const SupportStructure& structure =
            track.supports().structures.front();

        auto count = [&](const SupportMemberRole role)
        {
            return std::count_if(
                structure.members.begin(), structure.members.end(),
                [role](const SupportMember& member)
                {
                    return member.role == role;
                });
        };
        require(
            count(SupportMemberRole::Unspecified) == 0,
            "every hybrid member names its role");
        for (const SupportMemberRole role : {SupportMemberRole::PrimaryPost,
                 SupportMemberRole::LedgerCap,
                 SupportMemberRole::LongitudinalTie, SupportMemberRole::Brace})
        {
            require(count(role) > 0, "every generated hybrid structural role must occur");
        }
        require(count(SupportMemberRole::TrackSupport) == 0,
            "row height alone must never classify a Hybrid tie as a track-support stringer");

        // Bent-local ID sets separate transverse (in-bent) diagonals from
        // longitudinal (between-bent) diagonals without splitting the Brace
        // role: the plane lives in the orientation hint instead.
        std::unordered_map<SupportElementId, std::size_t> bentOf;
        for (std::size_t bent = 0; bent < 5; ++bent)
        {
            const OneStoryBent ids = bentAt(structure, bent);
            for (const SupportElementId id : {ids.baseLeft, ids.baseRight,
                     ids.shoulderLeft, ids.shoulderRight, ids.topLeft, ids.topRight, ids.ledgerLeft,
                     ids.ledgerRight})
            {
                bentOf.emplace(id, bent);
            }
        }
        for (const SupportMember& member : structure.members)
        {
            validateSupportMemberRole(member.role);
            validateSupportMemberOrientation(member.orientation);
            switch (member.role)
            {
            case SupportMemberRole::PrimaryPost:
                require(
                    member.orientation == SupportMemberOrientation::BentPost,
                    "hybrid posts align to the bent frame");
                break;
            case SupportMemberRole::LedgerCap:
                require(
                    member.orientation
                        == SupportMemberOrientation::BentTransverse,
                    "hybrid ledgers/caps lie in the transverse bent plane");
                break;
            case SupportMemberRole::LongitudinalTie:
            case SupportMemberRole::TrackSupport:
                require(
                    member.orientation
                        == SupportMemberOrientation::RunLongitudinal,
                    "hybrid longitudinal ties/stringers run with the track");
                break;
            case SupportMemberRole::Brace:
            {
                const bool sameBent = bentOf.at(member.startNodeId)
                    == bentOf.at(member.endNodeId);
                require(
                    member.orientation
                        == (sameBent
                                ? SupportMemberOrientation::BentDiagonal
                                : SupportMemberOrientation::RunDiagonal),
                    "transverse braces name the bent plane and longitudinal "
                    "braces name the run plane");
                break;
            }
            case SupportMemberRole::Unspecified:
                throw std::runtime_error(
                    "hybrid members must never stay Unspecified");
            }
        }
    }

    void noGradeTiesAndOpenBays()
    {
        AuthoredTrack track = elevatedTrack();
        static_cast<void>(
            generateWoodenSupportRun(track, hybridOneStoryRecipe()));
        const SupportStructure& structure =
            track.supports().structures.front();

        std::unordered_set<SupportElementId> foundations;
        for (const SupportNode& node : structure.nodes)
        {
            if (node.foundation.has_value())
            {
                foundations.insert(node.id);
            }
        }
        require(foundations.size() == 10, "five bents need ten footings");
        for (const SupportMember& member : structure.members)
        {
            require(
                !foundations.contains(member.startNodeId)
                    || !foundations.contains(member.endNodeId)
                    || member.role == SupportMemberRole::PrimaryPost,
                "footings carry primary posts only: no grade-level ties");
            require(
                !(foundations.contains(member.startNodeId)
                    && foundations.contains(member.endNodeId)),
                "discrete hybrid footings must not be tied footing to "
                "footing");
        }

        for (std::size_t bent = 1; bent < 5; ++bent)
        {
            const OneStoryBent before = bentAt(structure, bent - 1);
            const OneStoryBent after = bentAt(structure, bent);
            // Raised ledger, shoulder and interface each have their own ties.
            require(
                hasMember(structure, before.ledgerLeft, after.ledgerLeft)
                    && hasMember(
                        structure, before.ledgerRight, after.ledgerRight),
                "lower-ledger longitudinal ties must connect adjacent bents");
            const SupportMember* upperTie =
                findMember(structure, before.topLeft, after.topLeft);
            require(
                upperTie != nullptr
                    && upperTie->role == SupportMemberRole::LongitudinalTie
                    && upperTie->orientation
                        == SupportMemberOrientation::RunLongitudinal,
                "upper connections must remain longitudinal ties without a demonstrated stringer function");
            const SupportMember* shoulderTie = findMember(structure, before.shoulderLeft, after.shoulderLeft);
            require(shoulderTie && shoulderTie->role == SupportMemberRole::LongitudinalTie,
                "the highest lower-frame row must also retain tie semantics");
            require(
                hasMember(structure, before.topRight, after.topRight),
                "both upper tie lines must connect the interface rows");
            // Selected single-side bay bracing only: bays stay open except
            // the periodic braced bay the references show.
            const bool leftBrace = hasMember(
                  structure, before.ledgerLeft, after.shoulderLeft);
            const bool rightBrace = hasMember(
                  structure, before.ledgerRight, after.shoulderRight);
            const std::size_t braced =
                (leftBrace ? 1 : 0) + (rightBrace ? 1 : 0);
            require(
                braced == (bent == 2 ? 1 : 0),
                "only the selected longitudinal bay carries a brace");
        }
    }

    void tallTowerStacksCoherently()
    {
        AuthoredTrack track = elevatedTrack(20.0);
        WoodenSupportRunRecipe recipe{5.0, 25.0, 5.0, 4.0, 0.0, -0.5, 0.2,
            true};
        recipe.family = TimberSupportFamily::HybridTimberLattice;
        recipe.storyHeight = 8.0;
        recipe.hybridArchetype = HybridFramingArchetype::SimpleBent;
        static_cast<void>(generateWoodenSupportRun(track, recipe));
        const SupportStructure& first =
            track.supports().structures.front();

        // Node rows per bent: foundation row, story rows, raised ledger pair.
        const std::size_t perBent =
            first.nodes.size() / 5;
        require(
            first.nodes.size() % 5 == 0 && perBent > 6
                && (perBent - 4) % 2 == 0,
            "a tall hybrid run stacks multi-story bents");
        const std::size_t stories = (perBent - 4) / 2 - 1;
        require(stories >= 2, "the tall fixture must frame real stories");

        for (std::size_t bent = 0; bent < 5; ++bent)
        {
            const std::size_t offset = bent * perBent;
            // Row r holds the lane pair; the ledger pair trails the rows.
            const auto row = [&](const std::size_t level,
                                 const std::size_t lane)
            {
                return first.nodes[offset + level * 2 + lane].id;
            };
            const SupportElementId ledgerLeft =
                first.nodes[offset + (stories + 2) * 2].id;
            const SupportElementId ledgerRight =
                first.nodes[offset + (stories + 2) * 2 + 1].id;
            require(
                first.nodes[offset].foundation.has_value()
                    && first.nodes[offset + 1].foundation.has_value(),
                "tall towers keep discrete footings under the posts");
            for (std::size_t level = 1; level <= stories; ++level)
            {
                // One transverse ledger per story: coherent stacked levels.
                require(
                    hasMember(first, row(level, 0), row(level, 1)),
                    "every tall story needs its transverse ledger");
                // Exactly one diagonal per story: coherent, never an X.
                const SupportElementId lowerLeft =
                    level == 1 ? ledgerLeft : row(level - 1, 0);
                const SupportElementId lowerRight =
                    level == 1 ? ledgerRight : row(level - 1, 1);
                const bool forward =
                    hasMember(first, lowerLeft, row(level, 1));
                const bool backward =
                    hasMember(first, lowerRight, row(level, 0));
                require(
                    forward != backward,
                    "each tall story carries exactly one diagonal");
            }
        }

        AuthoredTrack second = elevatedTrack(20.0);
        static_cast<void>(generateWoodenSupportRun(second, recipe));
        require(
            second.supports() == track.supports(),
            "tall hybrid towers must generate deterministically");
    }

    void lowerFrameAndBankedInterfaceStaySeparate()
    {
        for (const double bank : {-0.5, 0.5})
        {
            AuthoredTrack track = elevatedTrack(40.0);
            AuthoredStartPose pose = track.startPose();
            pose.orientation = glm::angleAxis(0.7, glm::dvec3{0.0, 0.0, 1.0})
                * glm::angleAxis(-0.35, glm::dvec3{0.0, 1.0, 0.0})
                * glm::angleAxis(bank, glm::dvec3{1.0, 0.0, 0.0});
            track.setStartPose(pose);
            const auto states = integrateAuthoredTrackKinematics(track, 0.25);
            WoodenSupportRunRecipe recipe{0.0, 5.0, 5.0, 4.0,
                0.0, -0.5, 0.2, true};
            recipe.family = TimberSupportFamily::HybridTimberLattice;
            recipe.storyHeight = 8.0;
            recipe.hybridArchetype = HybridFramingArchetype::SimpleBent;
            static_cast<void>(generateWoodenSupportRun(track, recipe));
            const auto& structure = track.supports().structures.front();
            std::vector<std::size_t> offsets;
            for (std::size_t i = 0; i < structure.nodes.size(); ++i)
            {
                if (structure.nodes[i].foundation
                    && (i == 0 || !structure.nodes[i - 1].foundation))
                {
                    offsets.push_back(i);
                }
            }
            offsets.push_back(structure.nodes.size());
            require(offsets.size() == 3, "pitched fixture needs two bents");
            for (std::size_t bent = 0; bent + 1 < offsets.size(); ++bent)
            {
                const auto offset = offsets[bent];
                const auto count = offsets[bent + 1] - offset;
                const auto stories = (count - 4) / 2 - 1;
                const auto& nodes = structure.nodes;
                const auto shoulder = offset + stories * 2;
                const auto interface = shoulder + 2;
                const glm::dvec3 baseCenter = (nodes[offset].position + nodes[offset + 1].position) * 0.5;
                const glm::dvec3 baseAcross = glm::normalize(nodes[offset + 1].position - nodes[offset].position);
                for (std::size_t row = 0; row <= stories; ++row)
                {
                    const auto& left = nodes[offset + row * 2];
                    const auto& right = nodes[offset + row * 2 + 1];
                    require(!left.trackAttachment && !right.trackAttachment
                        && std::abs(left.position.z - right.position.z) < 1e-9,
                        "all lower timber rows must stay horizontal under combined pitch and bank");
                    require(glm::dot(glm::normalize(right.position - left.position), baseAcross) > 1.0 - 1e-12,
                        "lower framing must retain its unbanked transverse direction");
                    const auto midpoint = (left.position + right.position) * 0.5;
                    require(std::abs(midpoint.x - baseCenter.x) < 1e-9
                        && std::abs(midpoint.y - baseCenter.y) < 1e-9,
                        "lower framing must not shear toward pitched/banked attachments");
                }
                const auto ledger = interface + 2;
                require(std::abs(nodes[ledger].position.z - nodes[ledger + 1].position.z) < 1e-9,
                    "the raised ledger must also stay horizontal");
                require(nodes[shoulder].position.z < std::min(nodes[interface].position.z, nodes[interface + 1].position.z),
                    "the horizontal shoulder must sit below both track attachments");
                require(std::abs(nodes[interface].position.z - nodes[interface + 1].position.z) > 0.1,
                    "the upper interface must retain track banking");
                for (std::size_t lane = 0; lane < 2; ++lane)
                {
                    const auto& top = nodes[interface + lane];
                    require(top.trackAttachment && !top.foundation,
                        "only interface endpoints attach to the track");
                    const auto resolved = resolveSupportTrackAttachment(states, *top.trackAttachment);
                    require(glm::length(top.position - resolved.position) < 1e-9,
                        "interface endpoints must match the actual rider-frame geometry");
                    require(hasMember(structure, nodes[shoulder + lane].id, top.id),
                        "the upper interface must connect to its own post lane");
                }
            }
        }
    }

    void hybridProfilesDifferAsIntended()
    {
        AuthoredTrack track = elevatedTrack();
        const WoodenSupportRunRecipe recipe = hybridOneStoryRecipe();
        static_cast<void>(generateWoodenSupportRun(track, recipe));
        const SupportStructure& structure =
            track.supports().structures.front();

        // Pinned Hybrid hierarchy ratios (QUANTUM approximations of the
        // visible reference order: posts heaviest, single diagonals
        // lightest). s = 0.2 here.
        const double s = recipe.memberSize;
        const std::unordered_map<SupportMemberRole, glm::dvec2> expected{
            {SupportMemberRole::PrimaryPost, {s, s}},
            {SupportMemberRole::LedgerCap, {s * 0.95, s * 0.75}},
            {SupportMemberRole::TrackSupport, {s * 0.80, s * 0.62}},
            {SupportMemberRole::LongitudinalTie, {s * 0.70, s * 0.55}},
            {SupportMemberRole::Brace, {s * 0.55, s * 0.45}},
        };
        for (const SupportMember& member : structure.members)
        {
            const glm::dvec2 want = expected.at(member.role);
            require(
                member.profile.shape
                        == SupportMemberProfileShape::Rectangular
                    && member.profile.wallThickness == 0.0
                    && std::abs(member.profile.outerDimensions.x - want.x)
                        < 1e-12
                    && std::abs(member.profile.outerDimensions.y - want.y)
                        < 1e-12,
                "hybrid sections must follow the documented hierarchy");
        }
        const double postArea = s * s;
        const double ledgerArea = s * 0.95 * s * 0.75;
        const double trackArea = s * 0.80 * s * 0.62;
        const double tieArea = s * 0.70 * s * 0.55;
        const double braceArea = s * 0.55 * s * 0.45;
        require(
            postArea > ledgerArea && ledgerArea > trackArea
                && trackArea > tieArea && tieArea > braceArea,
            "posts must visibly dominate secondary braces");
        require(
            postArea / braceArea > 3.0,
            "the post-to-brace area ratio must read as primary vs "
            "secondary");
    }

    [[nodiscard]] const SupportNode& nodeById(const SupportStructure& structure,
        const SupportElementId id)
    {
        const auto found = std::find_if(structure.nodes.begin(), structure.nodes.end(),
            [id](const SupportNode& node) { return node.id == id; });
        require(found != structure.nodes.end(), "member endpoint must exist");
        return *found;
    }

    void connectedTowerTopology()
    {
        AuthoredTrack track = elevatedTrack(35.0);
        auto recipe = hybridOneStoryRecipe();
        recipe.storyHeight = 8.0;
        const AuthoredTrack source = track;
        static_cast<void>(generateWoodenSupportRun(track, recipe));
        const auto& structure = track.supports().structures.front();
        require(std::count_if(structure.nodes.begin(), structure.nodes.end(),
            [](const SupportNode& node) { return node.foundation.has_value(); }) == 10,
            "the connected assembly retains five two-post towers on ten footings");

        std::set<std::pair<SupportElementId, SupportElementId>> endpoints;
        std::vector<double> firstBay;
        std::vector<double> secondBay;
        std::size_t intermediateConnections = 0;
        for (const auto& member : structure.members)
        {
            require(endpoints.emplace(std::min(member.startNodeId, member.endNodeId),
                std::max(member.startNodeId, member.endNodeId)).second,
                "the assembly must not duplicate member endpoints");
            const auto& start = nodeById(structure, member.startNodeId);
            const auto& end = nodeById(structure, member.endNodeId);
            const auto axis = glm::normalize(end.position - start.position);
            require(member.orientationReference
                && std::abs(glm::length(*member.orientationReference) - 1.0) < 1e-9
                && std::abs(glm::dot(axis, *member.orientationReference)) < 1e-9,
                "every assembly member needs a unit perpendicular geometry reference");
            if (member.role == SupportMemberRole::PrimaryPost
                && !end.trackAttachment)
                require(std::abs(start.position.x - end.position.x) < 1e-9
                    && std::abs(start.position.y - end.position.y) < 1e-9,
                    "lower primary posts must be upright without mandatory base splay");
            if (member.role == SupportMemberRole::Brace)
                require(member.orientation == SupportMemberOrientation::BentDiagonal
                    && end.position.y > start.position.y,
                    "selected tower faces repeat one direction while connecting bays remain open");
            if (member.role != SupportMemberRole::LongitudinalTie)
                continue;
            require(member.orientation == SupportMemberOrientation::RunLongitudinal
                && member.profile.outerDimensions == glm::dvec2{recipe.memberSize * 0.70,
                    recipe.memberSize * 0.55},
                "new ties retain the Hybrid role, section and longitudinal orientation");
            if (start.trackAttachment || end.trackAttachment)
                continue;
            require(!start.foundation && !end.foundation
                && std::abs(start.position.z - end.position.z) < 1e-9
                && std::abs(end.position.x - start.position.x - recipe.bentSpacing) < 1e-9,
                "local ties must be horizontal and connect only neighboring towers");
            if (std::abs(start.position.y + recipe.bentWidth * 0.5) < 1e-9)
            {
                if (std::abs(start.position.x - 5.0) < 1e-9)
                    firstBay.push_back(start.position.z);
                if (std::abs(start.position.x - 10.0) < 1e-9)
                    secondBay.push_back(start.position.z);
            }
            // Test each endpoint independently: intermediate connection points
            // must belong to a split post, without an invented transverse row.
            for (const auto id : {member.startNodeId, member.endNodeId})
            {
                const auto incoming = std::find_if(structure.members.begin(), structure.members.end(),
                    [id](const SupportMember& value)
                    { return value.role == SupportMemberRole::PrimaryPost && value.endNodeId == id; });
                const auto outgoing = std::find_if(structure.members.begin(), structure.members.end(),
                    [id](const SupportMember& value)
                    { return value.role == SupportMemberRole::PrimaryPost && value.startNodeId == id; });
                require(incoming != structure.members.end() && outgoing != structure.members.end(),
                    "a lower tie endpoint must be a proper junction on the primary post");
                const bool ledger = std::any_of(structure.members.begin(), structure.members.end(),
                    [id](const SupportMember& value)
                    { return value.role == SupportMemberRole::LedgerCap
                        && (value.startNodeId == id || value.endNodeId == id); });
                if (!ledger)
                    ++intermediateConnections;
                if (id == member.startNodeId)
                {
                    auto expected = nodeById(structure, id).position
                        - nodeById(structure, incoming->startNodeId).position;
                    expected = glm::normalize(expected - axis * glm::dot(expected, axis));
                    require(glm::dot(expected, *member.orientationReference) > 1.0 - 1e-12,
                        "new tie roll must come from the incident primary post geometry");
                }
                require(std::count_if(structure.members.begin(), structure.members.end(),
                    [&, id](const SupportMember& value)
                    {
                        if (value.role != SupportMemberRole::LongitudinalTie
                            || (value.startNodeId != id && value.endNodeId != id))
                            return false;
                        const auto other = value.startNodeId == id ? value.endNodeId : value.startNodeId;
                        return std::abs(nodeById(structure, other).position.x
                            - (id == member.startNodeId ? end.position.x : start.position.x)) < 1e-9;
                    }) == 1,
                    "a connection point must not fan to multiple rows on the same neighboring tower");
            }
        }
        require(intermediateConnections > 0 && !firstBay.empty() && !secondBay.empty()
            && firstBay != secondBay,
            "neighboring bays need distinct elevations and intermediate post connections");
        auto simpleRecipe = recipe;
        simpleRecipe.hybridArchetype = HybridFramingArchetype::SimpleBent;
        AuthoredTrack simple = source;
        static_cast<void>(generateWoodenSupportRun(simple, simpleRecipe));
        const auto ledgers = [](const SupportStructure& value)
        {
            std::vector<std::array<double, 6>> result;
            for (const auto& member : value.members)
                if (member.role == SupportMemberRole::LedgerCap)
                {
                    const auto a = nodeById(value, member.startNodeId).position;
                    const auto b = nodeById(value, member.endNodeId).position;
                    result.push_back({a.x, a.y, a.z, b.x, b.y, b.z});
                }
            std::sort(result.begin(), result.end());
            return result;
        };
        require(ledgers(structure) == ledgers(simple.supports().structures.front()),
            "staggered ties must not force extra transverse ledgers or caps");
        const auto simpleRestored = deserializeCoasterDocument(serializeCoasterDocument(simple));
        require(simpleRestored && simpleRestored->supports() == simple.supports(),
            "explicit simple framing must remain selectable and persist exactly");
        AuthoredTrack explicitAssembly = source;
        recipe.hybridArchetype = HybridFramingArchetype::ConnectedTowers;
        static_cast<void>(generateWoodenSupportRun(explicitAssembly, recipe));
        require(explicitAssembly.supports().structures.front().nodes == structure.nodes
            && explicitAssembly.supports().structures.front().members == structure.members,
            "automatic stacked selection must agree with the explicit connected archetype");
        const auto expected = explicitAssembly.supports();
        const auto serialized = serializeCoasterDocument(explicitAssembly);
        const auto restored = deserializeCoasterDocument(serialized);
        require(restored && restored->supports() == expected,
            "assembly nodes, split posts and explicit recipe must round-trip exactly");
        static_cast<void>(generateWoodenSupportRun(explicitAssembly, recipe, expected.structures.front().id));
        require(explicitAssembly.supports() == expected,
            "regeneration must reproduce all local junctions and framing exactly");
        json malformed = json::parse(serialized);
        malformed["supports"]["structures"][0]["generatedWoodenRun"]["hybridArchetype"] = "UnknownTower";
        require(!deserializeCoasterDocument(malformed.dump()), "unknown archetypes must be rejected");
    }

    void independentTowerStories()
    {
        for (const double rise : {-0.3, 0.3})
        {
            AuthoredTrack track = elevatedTrack(26.5);
            auto pose = track.startPose();
            pose.orientation = glm::angleAxis(-std::asin(rise), glm::dvec3{0.0, 1.0, 0.0});
            track.setStartPose(pose);
            auto recipe = hybridOneStoryRecipe();
            recipe.storyHeight = 10.0;
            recipe.hybridArchetype = HybridFramingArchetype::ConnectedTowers;
            const AuthoredTrack source = track;
            static_cast<void>(generateWoodenSupportRun(track, recipe));
            const auto expected = track.supports();
            const auto& structure = expected.structures.front();
            std::map<double, std::size_t> storyCounts;
            bool receivingPostWithoutLedger = false;
            for (const auto& member : structure.members)
            {
                const auto& a = nodeById(structure, member.startNodeId);
                const auto& b = nodeById(structure, member.endNodeId);
                if (member.role == SupportMemberRole::LedgerCap && !a.trackAttachment)
                    ++storyCounts[a.position.x];
                if (member.role == SupportMemberRole::LongitudinalTie && !a.trackAttachment)
                {
                    require(std::abs(a.position.z - b.position.z) < 1e-9,
                        "changing story counts must not incline or round local horizontal ties");
                    receivingPostWithoutLedger |= std::none_of(structure.members.begin(), structure.members.end(),
                        [&](const SupportMember& other)
                        { return other.role == SupportMemberRole::LedgerCap
                            && (other.startNodeId == b.id || other.endNodeId == b.id); });
                }
            }
            require(storyCounts.size() == 5
                && storyCounts.begin()->second != storyCounts.rbegin()->second
                && receivingPostWithoutLedger,
                "rising and falling assemblies must retain independently terminating tower stories");
            AuthoredTrack repeat = source;
            static_cast<void>(generateWoodenSupportRun(repeat, recipe));
            require(repeat.supports() == expected,
                "independent local story geometry must generate deterministically");
        }
    }

    void writeHybridCaptureDocuments(char* const paths[])
    {
        for (std::size_t index = 0; index < 3; ++index)
        {
            AuthoredTrack track = elevatedTrack(index == 0 ? 7.0 : index == 2 ? 23.0 : 38.5);
            // Leave a short track margin beyond the final support station.
            setSectionLength(track.section(0), index == 2 ? 21.0 : 31.0);
            auto recipe = hybridOneStoryRecipe();
            recipe.startStation = 0.0;
            recipe.endStation = index == 2 ? 20.0 : 30.0;
            recipe.bentSpacing = index == 0 ? 6.0 : 7.5;
            recipe.bentWidth = 5.0;
            recipe.memberSize = 0.50;
            recipe.storyHeight = 8.0;
            recipe.longitudinalBracing = index != 0;
            recipe.hybridArchetype = index == 0 ? HybridFramingArchetype::SimpleBent
                : HybridFramingArchetype::ConnectedTowers;
            static_cast<void>(generateWoodenSupportRun(track, recipe));
            std::ofstream output(paths[index]);
            output << serializeCoasterDocument(track);
            require(output.good(), "could not write Hybrid visual acceptance document");
        }
    }

    void otherFamiliesUntouched()
    {
        // Hybrid-specific framing leaves this Traditional fixture's counts
        // intact. Shared role sections apply, with Generic orientation.
        AuthoredTrack track = elevatedTrack();
        const WoodenSupportRunRecipe recipe{5.0, 25.0, 5.0, 4.0, 0.0, -0.5,
            0.2, true};
        static_cast<void>(generateWoodenSupportRun(track, recipe));
        const SupportStructure& structure =
            track.supports().structures.front();
        require(
            structure.nodes.size() == 20 && structure.members.size() == 49,
            "traditional fixture topology counts must stay unchanged");
        for (const SupportMember& member : structure.members)
        {
            require(
                member.orientation == SupportMemberOrientation::Generic,
                "non-Hybrid members must keep the generic fallback");
            require(
                member.profile
                    == timberProfileForRole(member.role, recipe.memberSize),
                "non-Hybrid sections must keep the shared table");
        }
    }

    void serializationRemainsCompatible()
    {
        AuthoredTrack track = elevatedTrack();
        static_cast<void>(
            generateWoodenSupportRun(track, hybridOneStoryRecipe()));
        const std::string serialized = serializeCoasterDocument(track);
        const json document = json::parse(serialized);
        const json& firstMember =
            document["supports"]["structures"][0]["members"][0];
        require(
            firstMember.contains("role") && firstMember["role"].is_string()
                && firstMember.contains("orientation")
                && firstMember["orientation"].is_string()
                && firstMember.contains("orientationReference")
                && firstMember["orientationReference"].is_object(),
            "hybrid roles, orientations, and references persist as data");

        const auto restored = deserializeCoasterDocument(serialized);
        require(restored.has_value(), "hybrid documents must reload");
        require(
            restored->supports() == track.supports(),
            "roles, orientations, and sections must round-trip exactly");
        require(
            serializeCoasterDocument(*restored) == serialized,
            "hybrid serialization must stay deterministic");

        // Pre-reference documents (orientation without references, or
        // neither) load through the enum fallbacks.
        json legacy = document;
        for (auto& member : legacy["supports"]["structures"][0]["members"])
        {
            member.erase("orientationReference");
        }
        const auto oriented = deserializeCoasterDocument(legacy.dump());
        require(oriented.has_value(), "reference-less documents must load");
        for (const SupportMember& member :
            oriented->supports().structures.front().members)
        {
            require(
                !member.orientationReference.has_value(),
                "absent references must fall back to the enum");
        }
        for (auto& member : legacy["supports"]["structures"][0]["members"])
        {
            member.erase("orientation");
        }
        const auto roleOnly = deserializeCoasterDocument(legacy.dump());
        require(roleOnly.has_value(), "role-only documents must load");
        for (auto& member : legacy["supports"]["structures"][0]["members"])
        {
            member.erase("role");
        }
        const auto bare = deserializeCoasterDocument(legacy.dump());
        require(bare.has_value(), "bare M1/M2A documents must load");
        for (const SupportMember& member :
            bare->supports().structures.front().members)
        {
            require(
                member.role == SupportMemberRole::Unspecified
                    && member.orientation
                        == SupportMemberOrientation::Generic
                    && !member.orientationReference.has_value(),
                "absent fields must fall back to Unspecified/Generic/absent");
        }

        json unknown = document;
        unknown["supports"]["structures"][0]["members"][0]["orientation"] =
            "InventedPlane";
        require(
            !deserializeCoasterDocument(unknown.dump()).has_value(),
            "unknown orientations must be rejected");
        json numeric = document;
        numeric["supports"]["structures"][0]["members"][0]["orientation"] = 2;
        require(
            !deserializeCoasterDocument(numeric.dump()).has_value(),
            "numeric orientations must be rejected");

        bool rejected = false;
        try
        {
            validateSupportMemberOrientation(
                static_cast<SupportMemberOrientation>(255));
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        require(rejected, "unknown orientations must fail validation");

        // The renderer contract is untouched: one rectangular batch family.
        const SupportSolidPresentation presentation =
            buildSupportSolidPresentation(
                track.supports().structures.front());
        require(
            presentation.memberCount()
                == track.supports().structures.front().members.size(),
            "every hybrid member must still present exactly once");
        require(
            presentation.drawCallCount() <= 2,
            "hybrid accuracy must not add renderer draw calls");
    }
}

int main(const int argc, char* argv[])
{
    try
    {
        simpleBentTopology();
        memberRolesAndOrientations();
        noGradeTiesAndOpenBays();
        tallTowerStacksCoherently();
        lowerFrameAndBankedInterfaceStaySeparate();
        hybridProfilesDifferAsIntended();
        connectedTowerTopology();
        independentTowerStories();
        otherFamiliesUntouched();
        serializationRemainsCompatible();
        if (argc == 4)
            writeHybridCaptureDocuments(argv + 1);
    }
    catch (const std::exception& error)
    {
        std::cerr << "Hybrid timber accuracy test failure: " << error.what()
                  << '\n';
        return 1;
    }

    std::cout << "Hybrid timber accuracy tests passed.\n";
    return 0;
}
