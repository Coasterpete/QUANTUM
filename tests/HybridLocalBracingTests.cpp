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

    AuthoredTrack sourceTrack()
    {
        auto track = createNewDocument();
        auto pose = track.startPose();
        pose.position.z = 38.5;
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

    WoodenSupportRunRecipe mixedRecipe()
    {
        auto value = recipe();
        value.hybridLongitudinalPanels = {{0, 1, 0, HybridLongitudinalBracing::Open},
            {1, 1, 0, HybridLongitudinalBracing::Open},
            {2, 0, 0, HybridLongitudinalBracing::Open},
            {2, 1, 0, HybridLongitudinalBracing::SingleDiagonal},
            {3, 1, 0, HybridLongitudinalBracing::Open}};
        value.hybridTransversePanels = {
            {0, 0, SupportMemberMountingFace::PositiveZ, HybridDiagonalDirection::LowerFirstToUpperLast},
            {0, 1, SupportMemberMountingFace::PositiveZ, HybridDiagonalDirection::LowerLastToUpperFirst},
            {0, 2, SupportMemberMountingFace::NegativeZ, HybridDiagonalDirection::LowerLastToUpperFirst},
            {0, 3, SupportMemberMountingFace::NegativeZ, HybridDiagonalDirection::LowerFirstToUpperLast}};
        return value;
    }

    AuthoredTrack generated(const WoodenSupportRunRecipe& value)
    {
        auto track = sourceTrack();
        static_cast<void>(generateWoodenSupportRun(track, value));
        return track;
    }

    const SupportNode& node(const SupportStructure& structure, const SupportElementId id)
    {
        const auto found = std::find_if(structure.nodes.begin(), structure.nodes.end(),
            [id](const auto& value) { return value.id == id; });
        require(found != structure.nodes.end(), "missing structural node");
        return *found;
    }

    const SupportMemberPlacement& placed(const std::vector<SupportMemberPlacement>& placements,
        const SupportElementId id)
    {
        const auto found = std::find_if(placements.begin(), placements.end(),
            [id](const auto& value) { return value.memberId == id; });
        require(found != placements.end(), "missing physical placement");
        return *found;
    }

    std::size_t braceCount(const SupportStructure& structure, const SupportMemberOrientation orientation)
    {
        return std::count_if(structure.members.begin(), structure.members.end(),
            [orientation](const auto& member)
            { return member.role == SupportMemberRole::Brace && member.orientation == orientation; });
    }

    double halfExtent(const SupportMember& member, const SupportMemberFrame& frame, const glm::dvec3& normal)
    {
        return .5 * (member.profile.outerDimensions.x * std::abs(glm::dot(frame.axisY, normal))
            + member.profile.outerDimensions.y * std::abs(glm::dot(frame.axisZ, normal)));
    }

    // Generated lower posts are already foundation-directed. Choose the lower
    // incident segment, just as the physical resolver does at a split junction.
    glm::dvec3 faceNormal(const SupportStructure& structure, const SupportElementId id,
        const SupportMemberMountingFace face)
    {
        const auto post = std::find_if(structure.members.begin(), structure.members.end(),
            [id](const auto& value)
            { return value.role == SupportMemberRole::PrimaryPost && value.endNodeId == id; });
        require(post != structure.members.end(), "brace endpoint must have a lower incident post");
        const auto frame = resolveSupportMemberFrame(node(structure, post->startNodeId).position,
            node(structure, id).position, post->orientation, post->orientationReference);
        switch (face)
        {
        case SupportMemberMountingFace::PositiveY: return frame.axisY;
        case SupportMemberMountingFace::NegativeY: return -frame.axisY;
        case SupportMemberMountingFace::PositiveZ: return frame.axisZ;
        case SupportMemberMountingFace::NegativeZ: return -frame.axisZ;
        default: throw std::runtime_error("expected a structural side face");
        }
    }

    void openAndLocalLongitudinalPanels()
    {
        const auto open = generated(recipe());
        require(braceCount(open.supports().structures[0], SupportMemberOrientation::RunDiagonal) == 0,
            "ConnectedTowers defaults must retain open longitudinal bays");
        const auto value = mixedRecipe();
        const auto track = generated(value);
        const auto& structure = track.supports().structures[0];
        require(braceCount(structure, SupportMemberOrientation::RunDiagonal) == 1,
            "one local selection must create exactly one run brace, not a periodic or paired pattern");
        std::set<std::pair<SupportElementId, SupportElementId>> endpoints;
        for (const auto& member : structure.members)
        {
            require(endpoints.emplace(std::min(member.startNodeId, member.endNodeId),
                std::max(member.startNodeId, member.endNodeId)).second,
                "local variants cannot duplicate/fan member connections");
            if (member.orientation != SupportMemberOrientation::RunDiagonal) continue;
            const auto& a = node(structure, member.startNodeId);
            const auto& b = node(structure, member.endNodeId);
            require(near(a.position, {15, -2.5, 8}) && near(b.position, {22.5, -2.5, 16}),
                "the selected bay/panel/lane must use its actual tie boundary corners");
            std::vector<double> rows;
            for (const auto& tie : structure.members)
            {
                if (tie.role != SupportMemberRole::LongitudinalTie) continue;
                const auto& start = node(structure, tie.startNodeId);
                const auto& end = node(structure, tie.endNodeId);
                if (!start.trackAttachment && start.position.x == a.position.x
                    && end.position.x == b.position.x && start.position.y == a.position.y)
                    rows.push_back(start.position.z);
            }
            std::sort(rows.begin(), rows.end());
            const auto lower = std::find(rows.begin(), rows.end(), a.position.z);
            require(lower != rows.end() && lower + 1 != rows.end() && *(lower + 1) == b.position.z,
                "a brace must stay between consecutive matched local tie rows");
            require(!a.foundation && !b.foundation && !a.trackAttachment && !b.trackAttachment,
                "no fake ground, cap, or interface panel may be filled");
            require(member.role == SupportMemberRole::Brace
                && member.profile.shape == SupportMemberProfileShape::Rectangular
                && member.profile.outerDimensions == glm::dvec2(.275, .225),
                "run braces must keep the Hybrid brace role and solid section");
            const auto expected = glm::normalize(glm::cross(glm::dvec3(0, 0, 8), b.position - a.position));
            require(member.orientationReference && near(*member.orientationReference, expected),
                "RunDiagonal roll must derive from the actual panel triangle");
        }
        // Explicit local choices are authoritative; the old whole-run boolean
        // is retained for older SimpleBent/other-family recipes only.
        auto disabled = value;
        disabled.longitudinalBracing = false;
        require(braceCount(generated(disabled).supports().structures[0], SupportMemberOrientation::RunDiagonal) == 1,
            "a legacy global toggle cannot override an explicit local ConnectedTowers panel");
        auto otherLane = value;
        otherLane.hybridLongitudinalPanels[3].laneIndex = 1;
        const auto right = generated(otherLane);
        for (const auto& member : right.supports().structures[0].members)
            if (member.orientation == SupportMemberOrientation::RunDiagonal)
                require(member.startConnection->mounting->face == SupportMemberMountingFace::PositiveY
                    && node(right.supports().structures[0], member.startNodeId).position.y == 2.5,
                    "the opposite run-side lane must use its own directed outside face");
        // A staggered bay has different real panel boundaries from its neighbor.
        auto staggered = recipe();
        staggered.hybridLongitudinalPanels = {{1, 0, 0, HybridLongitudinalBracing::SingleDiagonal}};
        const auto shifted = generated(staggered);
        for (const auto& member : shifted.supports().structures[0].members)
            if (member.orientation == SupportMemberOrientation::RunDiagonal)
            {
                require(near(node(shifted.supports().structures[0], member.startNodeId).position, {7.5, -2.5, 5.2})
                    && near(node(shifted.supports().structures[0], member.endNodeId).position, {15, -2.5, 13.2}),
                    "staggered panel choices must use actual local tie elevations, not story ordinals");
            }
    }

    void transverseFaceAndDirection()
    {
        const auto track = generated(mixedRecipe());
        const auto& structure = track.supports().structures[0];
        require(braceCount(structure, SupportMemberOrientation::BentDiagonal) == 25,
            "five towers must retain exactly one brace per five selected panels, never automatic X/front-back pairs");
        const std::array faces{SupportMemberMountingFace::PositiveZ, SupportMemberMountingFace::PositiveZ,
            SupportMemberMountingFace::NegativeZ, SupportMemberMountingFace::NegativeZ, SupportMemberMountingFace::PositiveZ};
        const std::array forward{true, false, false, true, true};
        std::size_t panel = 0;
        const auto placements = resolveSupportMemberPlacements(structure);
        for (const auto& member : structure.members)
        {
            const auto& a = node(structure, member.startNodeId);
            const auto& b = node(structure, member.endNodeId);
            if (member.orientation != SupportMemberOrientation::BentDiagonal || a.position.x != 0) continue;
            require(panel < faces.size() && (b.position.y > a.position.y) == forward[panel],
                "authored diagonal direction must be independent of story parity");
            require(member.startConnection->mounting->face == faces[panel]
                && member.endConnection->mounting->face == faces[panel],
                "each transverse panel must carry its selected bent face to both endpoints");
            const auto normal = faceNormal(structure, a.id, faces[panel]);
            const auto& physical = placed(placements, member.id);
            require(glm::dot(physical.start - a.position, normal) > 0
                && glm::dot(physical.end - b.position, normal) > 0,
                "the brace must resolve physically on its selected structural face");
            const double expectedOffset = .25 + .1375 + .01
                + (faces[panel] == SupportMemberMountingFace::PositiveZ ? .375 : 0.0);
            require(std::abs(glm::dot(physical.start - a.position, normal) - expectedOffset) < 1e-9,
                "opposite-face braces cannot inherit a phantom ledger layer from the positive face");
            require(member.startConnection->mounting->layer == SupportMemberMountingLayer::OutsideLedger,
                "local face selection must preserve outside-ledger mounting intent");
            ++panel;
        }
        require(panel == faces.size(), "all four local face/direction combinations must be exercised");
        // The same selections work on SimpleBent without changing its archetype.
        auto simple = mixedRecipe();
        simple.hybridArchetype = HybridFramingArchetype::SimpleBent;
        require(braceCount(generated(simple).supports().structures[0], SupportMemberOrientation::BentDiagonal) == 25,
            "SimpleBent must retain local transverse variants and its baseline single-brace topology");
    }

    void physicalLayersAndStableFaces()
    {
        const auto track = generated(mixedRecipe());
        const auto& structure = track.supports().structures[0];
        const auto placements = resolveSupportMemberPlacements(structure);
        for (const auto& member : structure.members)
        {
            if (member.role != SupportMemberRole::Brace) continue;
            const auto& physical = placed(placements, member.id);
            for (const auto end : {SupportMemberEnd::Start, SupportMemberEnd::End})
            {
                const bool first = end == SupportMemberEnd::Start;
                const auto id = first ? member.startNodeId : member.endNodeId;
                const auto& intent = *(first ? member.startConnection : member.endConnection)->mounting;
                const auto logical = node(structure, id).position;
                const auto normal = faceNormal(structure, id, intent.face);
                const auto position = first ? physical.start : physical.end;
                const auto braceInner = glm::dot(position - logical, normal) - halfExtent(member, physical.frame, normal);
                for (const auto& horizontal : structure.members)
                {
                    if (horizontal.role != SupportMemberRole::LedgerCap
                        && horizontal.role != SupportMemberRole::LongitudinalTie) continue;
                    if (horizontal.startNodeId != id && horizontal.endNodeId != id) continue;
                    const bool horizontalFirst = horizontal.startNodeId == id;
                    const auto& contact = horizontalFirst ? horizontal.startConnection : horizontal.endConnection;
                    if (!contact || !contact->mounting || contact->mounting->face != intent.face) continue;
                    const auto& horizontalPlacement = placed(placements, horizontal.id);
                    const auto horizontalPosition = horizontalFirst ? horizontalPlacement.start : horizontalPlacement.end;
                    const auto outer = glm::dot(horizontalPosition - logical, normal)
                        + halfExtent(horizontal, horizontalPlacement.frame, normal);
                    require(braceInner >= outer + intent.separation - 1e-9,
                        "each new brace end must clear the actual same-face ledger/tie surface");
                }
            }
        }
        auto reversed = structure;
        for (auto& member : reversed.members)
        {
            std::swap(member.startNodeId, member.endNodeId);
            std::swap(member.startConnection, member.endConnection);
        }
        std::reverse(reversed.nodes.begin(), reversed.nodes.end());
        std::reverse(reversed.members.begin(), reversed.members.end());
        const auto reversedPlacements = resolveSupportMemberPlacements(reversed);
        for (const auto& physical : placements)
            require(near(placed(reversedPlacements, physical.memberId).start, physical.end)
                && near(placed(reversedPlacements, physical.memberId).end, physical.start),
                "reversing post/member endpoints and containers cannot swap selected structural faces");
        const auto rotation = glm::angleAxis(.71, glm::normalize(glm::dvec3(.3, .5, .7)));
        auto rotated = structure;
        for (auto& value : rotated.nodes) value.position = rotation * value.position;
        for (auto& member : rotated.members)
            if (member.orientationReference) member.orientationReference = rotation * *member.orientationReference;
        const auto rotatedPlacements = resolveSupportMemberPlacements(rotated);
        for (const auto& physical : placements)
            require(near(placed(rotatedPlacements, physical.memberId).start, rotation * physical.start)
                && near(placed(rotatedPlacements, physical.memberId).end, rotation * physical.end),
                "mounting must follow the authored structural frame, never a world axis");
    }

    void curvedBankedAndChangingStories()
    {
        for (const bool curved : {false, true})
            for (const double rise : {-.3, .3})
            {
                auto track = curved ? createDefaultAuthoredTrack() : sourceTrack();
                auto pose = track.startPose();
                pose.position.z = 38.5;
                pose.orientation = glm::angleAxis(.7, glm::dvec3(0, 0, 1))
                    * glm::angleAxis(-std::asin(rise), glm::dvec3(0, 1, 0))
                    * glm::angleAxis(.4, glm::dvec3(1, 0, 0));
                track.setStartPose(pose);
                auto value = mixedRecipe();
                value.endStation = 20;
                value.bentSpacing = 5;
                value.memberSize = .2;
                value.hybridLongitudinalPanels = {{1, 0, 0, HybridLongitudinalBracing::SingleDiagonal}};
                static_cast<void>(generateWoodenSupportRun(track, value));
                const auto& structure = track.supports().structures[0];
                require(braceCount(structure, SupportMemberOrientation::RunDiagonal) == 1,
                    "curved/banked and rising/falling towers must keep the single local run choice");
                const auto placements = resolveSupportMemberPlacements(structure);
                require(buildSupportSolidPresentation(structure).memberCount() == structure.members.size(),
                    "all local variants must resolve and render on curved/banked geometry");
                for (const auto& member : structure.members)
                {
                    if (member.role != SupportMemberRole::Brace) continue;
                    const auto& physical = placed(placements, member.id);
                    const auto a = node(structure, member.startNodeId).position;
                    const auto b = node(structure, member.endNodeId).position;
                    require(member.orientationReference
                        && std::abs(glm::dot(glm::normalize(b - a), *member.orientationReference)) < 1e-9,
                        "local brace roll must remain perpendicular to its actual geometry");
                    const auto normalA = faceNormal(structure, member.startNodeId, member.startConnection->mounting->face);
                    const auto normalB = faceNormal(structure, member.endNodeId, member.endConnection->mounting->face);
                    require(glm::length(glm::cross(physical.start - a, normalA)) < 1e-9
                        && glm::length(glm::cross(physical.end - b, normalB)) < 1e-9,
                        "brace contacts must resolve independently against each endpoint's post frame");
                    if (member.orientation == SupportMemberOrientation::RunDiagonal)
                    {
                        std::array<std::vector<double>, 2> rows;
                        for (const auto& tie : structure.members)
                        {
                            if (tie.role != SupportMemberRole::LongitudinalTie) continue;
                            const auto& first = node(structure, tie.startNodeId);
                            const auto& last = node(structure, tie.endNodeId);
                            if (first.trackAttachment || last.trackAttachment) continue;
                            if (glm::length(glm::dvec2(first.position - a)) >= 1e-9
                                || glm::length(glm::dvec2(last.position - b)) >= 1e-9) continue;
                            for (const auto end : {0, 1})
                            {
                                const auto endpoint = end == 0 ? a : b;
                                const auto candidate = end == 0 ? first.position : last.position;
                                if (glm::length(glm::dvec2(candidate - endpoint)) < 1e-9)
                                    rows[end].push_back(candidate.z);
                            }
                        }
                        for (auto& elevations : rows)
                        {
                            std::sort(elevations.begin(), elevations.end());
                            require(elevations.size() >= 2 && std::abs(elevations[0] - a.z) < 1e-9
                                && std::abs(elevations[1] - b.z) < 1e-9,
                                "unequal tower stories cannot make a brace bridge unmatched local rows");
                        }
                    }
                }
            }
    }

    void persistenceRegenerationAndHistory()
    {
        auto track = generated(mixedRecipe());
        const auto before = serializeCoasterDocument(track);
        auto restored = deserializeCoasterDocument(before);
        require(restored && serializeCoasterDocument(*restored) == before,
            "local choices and physical intent must save/load exactly");
        const auto original = track.supports().structures[0];
        const auto id = original.id;
        auto value = *original.generatedWoodenRun;
        static_cast<void>(generateWoodenSupportRun(*restored, value, id));
        require(serializeCoasterDocument(*restored) == before,
            "regeneration after loading must restore authored local choices exactly");
        std::reverse(value.hybridTransversePanels.begin(), value.hybridTransversePanels.end());
        std::reverse(value.hybridLongitudinalPanels.begin(), value.hybridLongitudinalPanels.end());
        const auto reordered = generated(value);
        require(reordered.supports().structures[0].members == original.members
            && reordered.supports().structures[0].nodes == original.nodes,
            "choice iteration order cannot change generated geometry or face semantics");
        value = *original.generatedWoodenRun;
        value.hybridTransversePanels.push_back({0, 63, SupportMemberMountingFace::NegativeZ,
            HybridDiagonalDirection::LowerLastToUpperFirst});
        value.hybridLongitudinalPanels.push_back({2, 63, 0, HybridLongitudinalBracing::SingleDiagonal});
        const auto dormant = generated(value);
        require(dormant.supports().structures[0].members == original.members,
            "absent panels must remain dormant, never bridge unrelated stories or invent rows");
        auto shorter = value;
        shorter.endStation = 7.5;
        static_cast<void>(generateWoodenSupportRun(track, shorter, id));
        require(track.supports().structures[0].generatedWoodenRun == shorter
            && braceCount(track.supports().structures[0], SupportMemberOrientation::RunDiagonal) == 0,
            "shortening a run must preserve dormant selections without remapping them");
        static_cast<void>(generateWoodenSupportRun(track, value, id));
        require(track.supports() == dormant.supports(), "restoring the domain must restore its local panels");

        editor::DocumentHistory history;
        history.reset(*restored);
        editor::AuthoredTrackEditTransaction edit{*restored};
        value = *original.generatedWoodenRun;
        value.hybridLongitudinalPanels[3].bracing = HybridLongitudinalBracing::Open;
        value.hybridTransversePanels[0].face = SupportMemberMountingFace::NegativeZ;
        value.hybridTransversePanels[0].direction = HybridDiagonalDirection::LowerLastToUpperFirst;
        static_cast<void>(generateWoodenSupportRun(edit.candidate(), value, id));
        edit.commit(*restored);
        history.record(*restored);
        const auto after = serializeCoasterDocument(*restored);
        const auto undo = history.undo();
        const auto redo = history.redo();
        require(undo && serializeCoasterDocument(*undo) == before, "local choice Undo must be byte-exact");
        require(redo && serializeCoasterDocument(*redo) == after, "local choice Redo must be byte-exact");
        const auto expectedPlacement = resolveSupportMemberPlacements(restored->supports().structures[0]);
        const auto redoPlacement = resolveSupportMemberPlacements(redo->supports().structures[0]);
        for (const auto& physical : expectedPlacement)
            require(near(placed(redoPlacement, physical.memberId).start, physical.start),
                "Redo must also restore the exact resolved physical placement");

        // Legacy documents have neither additive recipe field. Loading cannot
        // regenerate or retroactively move their already persisted members.
        const auto legacy = generated(recipe());
        auto oldJson = json::parse(serializeCoasterDocument(legacy));
        const auto& oldRecipe = oldJson["supports"]["structures"][0]["generatedWoodenRun"];
        require(!oldRecipe.contains("hybridTransversePanels") && !oldRecipe.contains("hybridLongitudinalPanels"),
            "empty local choice lists must retain the old document shape");
        auto oldLoaded = deserializeCoasterDocument(oldJson.dump());
        require(oldLoaded && oldLoaded->supports() == legacy.supports(), "old generated documents must load unchanged");
        static_cast<void>(generateWoodenSupportRun(*oldLoaded,
            *oldLoaded->supports().structures[0].generatedWoodenRun, oldLoaded->supports().structures[0].id));
        require(oldLoaded->supports() == legacy.supports(), "legacy defaults must regenerate deterministically");
    }

    void malformedChoicesAreAtomic()
    {
        const auto source = generated(mixedRecipe());
        const auto original = serializeCoasterDocument(source);
        auto valid = json::parse(original);
        const auto rejected = [&](const char* field, const json& panels)
        {
            auto malformed = valid;
            malformed["supports"]["structures"][0]["generatedWoodenRun"][field] = panels;
            require(!deserializeCoasterDocument(malformed.dump()), "malformed local panel choices must be rejected");
        };
        auto panel = valid["supports"]["structures"][0]["generatedWoodenRun"]["hybridTransversePanels"][0];
        for (const json invalid : {json("WorldX"), json("PositiveY"), json(3)})
        {
            auto changed = panel; changed["face"] = invalid;
            rejected("hybridTransversePanels", json::array({changed}));
        }
        for (const json invalid : {json(-1), json(1.5), json(4294967296ULL), json("0")})
        {
            auto changed = panel; changed["towerIndex"] = invalid;
            rejected("hybridTransversePanels", json::array({changed}));
        }
        auto changed = panel; changed["direction"] = "Alternating";
        rejected("hybridTransversePanels", json::array({changed}));
        rejected("hybridTransversePanels", json::array({panel, panel}));
        rejected("hybridTransversePanels", json::object());
        changed = panel; changed["paired"] = true;
        rejected("hybridTransversePanels", json::array({changed}));
        panel = valid["supports"]["structures"][0]["generatedWoodenRun"]["hybridLongitudinalPanels"][3];
        changed = panel; changed["laneIndex"] = 2;
        rejected("hybridLongitudinalPanels", json::array({changed}));
        changed = panel; changed["bracing"] = "EveryThird";
        rejected("hybridLongitudinalPanels", json::array({changed}));
        rejected("hybridLongitudinalPanels", json::array({panel, panel}));
        auto track = source;
        auto value = mixedRecipe();
        value.hybridTransversePanels.push_back(value.hybridTransversePanels[0]);
        bool threw = false;
        try { static_cast<void>(generateWoodenSupportRun(track, value, track.supports().structures[0].id)); }
        catch (const std::invalid_argument&) { threw = true; }
        require(threw && serializeCoasterDocument(track) == original, "rejected regeneration must leave all current work intact");
    }

    void writeCaptureDocuments(const std::filesystem::path& directory)
    {
        std::filesystem::create_directories(directory);
        const auto write = [&](const char* name, const AuthoredTrack& track)
        {
            std::ofstream output(directory / name);
            output << serializeCoasterDocument(track);
            require(output.good(), "could not write local bracing capture document");
        };
        write("open.quantum", generated(recipe()));
        auto value = mixedRecipe();
        // Two explicitly authored adjacent panels in one bay, not an automatic
        // rule. Neighboring bays and the opposite run side stay open.
        value.hybridLongitudinalPanels.push_back({2, 2, 0, HybridLongitudinalBracing::SingleDiagonal});
        const auto mixed = generated(value);
        write("mixed.quantum", mixed);
        auto isolated = mixed;
        auto collection = mixed.supports();
        auto& tower = collection.structures[0];
        std::erase_if(tower.members, [&](const auto& member)
        { return node(tower, member.startNodeId).position.x != 0 || node(tower, member.endNodeId).position.x != 0; });
        std::erase_if(tower.nodes, [](const auto& value) { return value.position.x != 0; });
        // Keep source recipe metadata for the capture UI, as the existing
        // mounting tower export does. This is a crop, not a different recipe.
        tower.name = "Hybrid face variants (capture crop)";
        isolated.setSupports(collection);
        write("face-variants.quantum", isolated);
    }
}

int main(const int argc, char* argv[])
{
    try
    {
        openAndLocalLongitudinalPanels();
        transverseFaceAndDirection();
        physicalLayersAndStableFaces();
        curvedBankedAndChangingStories();
        persistenceRegenerationAndHistory();
        malformedChoicesAreAtomic();
        if (argc == 2) writeCaptureDocuments(argv[1]);
    }
    catch (const std::exception& error)
    {
        std::cerr << "Hybrid local bracing test failure: " << error.what() << '\n';
        return 1;
    }
    std::cout << "Hybrid local bracing tests passed (6 groups).\n";
}
