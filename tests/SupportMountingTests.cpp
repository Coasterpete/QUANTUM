#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/WoodenSupportGenerator.hpp>
#include <quantum/editor/AuthoredTrackEditTransaction.hpp>
#include <quantum/editor/DocumentHistory.hpp>
#include <quantum/editor/SupportPicking.hpp>
#include <quantum/editor/SupportVisualization.hpp>
#include <quantum/editor/ViewportTrackAnchors.hpp>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string_view>

namespace
{
    using namespace quantum;
    using namespace quantum::coaster;

    void require(const bool condition, const std::string_view message)
    {
        if (!condition) throw std::runtime_error(std::string(message));
    }

    bool near(const glm::dvec3& a, const glm::dvec3& b)
    {
        return glm::length(a - b) < 1e-9;
    }

    SupportMemberProfile section(const double width, const double depth)
    {
        return {SupportMemberProfileShape::Rectangular, {width, depth}, 0.0};
    }

    struct BentFixture
    {
        AuthoredTrack track = createNewDocument();
        SupportStructureId id = track.createSupportStructure("Mounting fixture");
        std::array<SupportElementId, 6> nodes{};
        SupportElementId lowerLedger;
        SupportElementId upperLedger;
        SupportElementId brace;

        BentFixture()
        {
            const std::array<glm::dvec3, 6> positions{{
                {0, 0, 0}, {4, 0, 0}, {0, 0, 2}, {4, 0, 2}, {0, 0, 5}, {4, 0, 5}}};
            for (std::size_t i = 0; i < nodes.size(); ++i)
                nodes[i] = track.createSupportNode(id, positions[i]);
            track.setSupportFoundation(id, nodes[0]);
            track.setSupportFoundation(id, nodes[1]);
            for (std::size_t lane = 0; lane < 2; ++lane)
                for (std::size_t row = 0; row < 2; ++row)
                    static_cast<void>(track.createSupportMember(id, nodes[row * 2 + lane],
                        nodes[row * 2 + lane + 2], section(.4, .6), SupportMemberRole::PrimaryPost,
                        SupportMemberOrientation::BentPost, glm::dvec3(1, 0, 0)));
            lowerLedger = track.createSupportMember(id, nodes[2], nodes[3], section(.8, .2),
                SupportMemberRole::LedgerCap, SupportMemberOrientation::BentTransverse, glm::dvec3(0, 0, 1));
            upperLedger = track.createSupportMember(id, nodes[4], nodes[5], section(.8, .2),
                SupportMemberRole::LedgerCap, SupportMemberOrientation::BentTransverse, glm::dvec3(0, 0, 1));
            brace = track.createSupportMember(id, nodes[2], nodes[5], section(.16, .3),
                SupportMemberRole::Brace, SupportMemberOrientation::BentDiagonal, glm::dvec3(0, 1, 0));
        }

        const SupportStructure& structure() const { return track.supports().structures.front(); }

        void mount(const SupportElementId member, const SupportMemberMounting& mounting)
        {
            SupportMemberEndConnection connection;
            connection.mounting = mounting;
            for (const auto end : {SupportMemberEnd::Start, SupportMemberEnd::End})
                track.setSupportMemberEndConnection(id, member, end, connection);
        }

        void mountAll()
        {
            SupportMemberMounting ledger;
            ledger.coverage = SupportMemberEndCoverage::OutsideSupport;
            ledger.overhang = .05;
            mount(lowerLedger, ledger);
            mount(upperLedger, ledger);
            SupportMemberMounting diagonal;
            diagonal.layer = SupportMemberMountingLayer::OutsideLedger;
            diagonal.separation = .01;
            mount(brace, diagonal);
        }
    };

    const SupportMemberPlacement& placement(const std::vector<SupportMemberPlacement>& placements,
        const SupportElementId id)
    {
        const auto found = std::find_if(placements.begin(), placements.end(),
            [id](const auto& value) { return value.memberId == id; });
        if (found == placements.end()) throw std::runtime_error("Missing placement");
        return *found;
    }

    void legacyAndConnectorPlacementRemainCentered()
    {
        BentFixture fixture;
        SupportMemberEndConnection connector;
        connector.localPlacement = SupportMemberEndPlacement{{9, 8, 7}};
        fixture.track.setSupportMemberEndConnection(fixture.id, fixture.lowerLedger,
            SupportMemberEnd::Start, connector);
        const auto before = fixture.track.supports();
        const auto resolved = resolveSupportMemberPlacements(fixture.structure());
        const auto solid = buildSupportSolidPresentation(fixture.structure());
        for (std::size_t i = 0; i < resolved.size(); ++i)
        {
            const auto& member = fixture.structure().members[i];
            const auto start = std::find_if(fixture.structure().nodes.begin(), fixture.structure().nodes.end(),
                [&](const auto& node) { return node.id == member.startNodeId; })->position;
            const auto end = std::find_if(fixture.structure().nodes.begin(), fixture.structure().nodes.end(),
                [&](const auto& node) { return node.id == member.endNodeId; })->position;
            const auto frame = resolveSupportMemberFrame(start, end, member.orientation, member.orientationReference);
            require(resolved[i].start == start && resolved[i].end == end,
                "absent mounting must preserve logical endpoints exactly");
            glm::mat4 expected{1.0F};
            expected[0] = glm::vec4(glm::vec3(frame.axisX) * static_cast<float>(frame.length), 0.0F);
            expected[1] = glm::vec4(glm::vec3(frame.axisY) * static_cast<float>(member.profile.outerDimensions.x), 0.0F);
            expected[2] = glm::vec4(glm::vec3(frame.axisZ) * static_cast<float>(member.profile.outerDimensions.y), 0.0F);
            expected[3] = glm::vec4(glm::vec3(frame.origin), 1.0F);
            require(solid.batches[0].instances[i].transform == expected,
                "legacy instance matrix must remain bit-identical, including connector localPlacement");
        }
        require(fixture.track.supports() == before, "presentation must never mutate graph data");
    }

    void ledgerAndBraceContactAndSectionEdits()
    {
        BentFixture fixture;
        const auto logical = fixture.structure().nodes;
        fixture.mountAll();
        auto resolved = resolveSupportMemberPlacements(fixture.structure());
        const auto& ledger = placement(resolved, fixture.lowerLedger);
        require(near(ledger.start, {-.25, .4, 2}) && near(ledger.end, {4.25, .4, 2}),
            "ledger must clear post face and cover outside edges plus axial overhang");
        const auto& diagonal = placement(resolved, fixture.brace);
        require(near(diagonal.start, {0, .59, 2}) && near(diagonal.end, {4, .59, 5}),
            "brace must clear post + full ledger thickness + its own half-thickness + gap");
        require(std::abs((diagonal.start.y - .08) - (ledger.start.y + .1) - .01) < 1e-12,
            "brace/ledger surfaces must be separated by the explicit gap");
        auto edited = fixture.track.supports();
        for (auto& member : edited.structures[0].members)
        {
            if (member.role == SupportMemberRole::PrimaryPost) member.profile = section(.8, .8);
            if (member.role == SupportMemberRole::LedgerCap) member.profile = section(.8, .3);
        }
        fixture.track.setSupports(edited);
        resolved = resolveSupportMemberPlacements(fixture.structure());
        require(near(placement(resolved, fixture.lowerLedger).start, {-.45, .55, 2}),
            "post/ledger section edits must recompute contact and end coverage");
        require(near(placement(resolved, fixture.brace).start, {0, .79, 2}),
            "brace layer must use edited ledger thickness rather than a baked offset");
        require(fixture.structure().nodes == logical, "all mounting and size edits must preserve logical nodes");

        // Direct braces need only the face contact, independently of ledger axes.
        fixture.mount(fixture.brace, SupportMemberMounting{});
        resolved = resolveSupportMemberPlacements(fixture.structure());
        require(near(placement(resolved, fixture.brace).start, {0, .48, 2}),
            "direct brace thickness must come from its broad-face axis Y, not local Z");
    }

    double halfExtent(const SupportMemberProfile& profile, const SupportMemberFrame& frame,
        const glm::dvec3& normal)
    {
        return .5 * (profile.outerDimensions.x * std::abs(glm::dot(frame.axisY, normal))
            + profile.outerDimensions.y * std::abs(glm::dot(frame.axisZ, normal)));
    }

    void twoEndedTieAndDirectedFrames()
    {
        AuthoredTrack track = createNewDocument();
        const auto id = track.createSupportStructure("Curved two-ended tie");
        const glm::dvec3 a{0, 0, 2}, b{0, 5, 2};
        const auto footA = track.createSupportNode(id, a - glm::dvec3(0, 0, 2));
        const auto footB = track.createSupportNode(id, b - glm::dvec3(0, 0, 2));
        const auto nodeA = track.createSupportNode(id, a);
        const auto nodeB = track.createSupportNode(id, b);
        track.setSupportFoundation(id, footA);
        track.setSupportFoundation(id, footB);
        const glm::dvec3 acrossB{std::cos(.55), std::sin(.55), 0};
        static_cast<void>(track.createSupportMember(id, footA, nodeA, section(.4, .6),
            SupportMemberRole::PrimaryPost, SupportMemberOrientation::BentPost, glm::dvec3(1, 0, 0)));
        static_cast<void>(track.createSupportMember(id, footB, nodeB, section(.8, .5),
            SupportMemberRole::PrimaryPost, SupportMemberOrientation::BentPost, acrossB));
        const auto tieId = track.createSupportMember(id, nodeA, nodeB, section(.5, .2),
            SupportMemberRole::LongitudinalTie, SupportMemberOrientation::RunLongitudinal, glm::dvec3(0, 0, 1));
        SupportMemberEndConnection connection;
        connection.mounting = SupportMemberMounting{};
        connection.mounting->face = SupportMemberMountingFace::NegativeY;
        track.setSupportMemberEndConnection(id, tieId, SupportMemberEnd::Start, connection);
        connection.mounting->face = SupportMemberMountingFace::PositiveY;
        track.setSupportMemberEndConnection(id, tieId, SupportMemberEnd::End, connection);
        const auto logical = track.supports();
        const auto first = resolveSupportMemberPlacements(logical.structures[0]);
        const auto tie = placement(first, tieId);
        const glm::dvec3 normalA{-1, 0, 0};
        require(near(tie.start, a + normalA * (.2 + halfExtent(section(.5, .2), tie.frame, normalA))),
            "tie start contact must use its own post section and signed face");
        require(near(tie.end, b + acrossB * (.4 + halfExtent(section(.5, .2), tie.frame, acrossB))),
            "tie end contact must use the other rotated post frame and section");
        require(tie.start.z == a.z && tie.end.z == b.z, "lower tie elevation must remain unchanged");
        auto reversed = logical;
        for (auto& member : reversed.structures[0].members)
        {
            std::swap(member.startNodeId, member.endNodeId);
            std::swap(member.startConnection, member.endConnection);
        }
        const auto reversePlacements = resolveSupportMemberPlacements(reversed.structures[0]);
        require(near(placement(reversePlacements, tieId).start, tie.end)
            && near(placement(reversePlacements, tieId).end, tie.start),
            "reversing both mounted and supporting endpoints must preserve chosen structural faces");
        std::reverse(reversed.structures[0].nodes.begin(), reversed.structures[0].nodes.end());
        std::reverse(reversed.structures[0].members.begin(), reversed.structures[0].members.end());
        const auto reordered = resolveSupportMemberPlacements(reversed.structures[0]);
        require(near(placement(reordered, tieId).start, tie.end), "container order cannot affect mounting face signs");

        // Rotate the entire authored structure, including its references. No
        // global-axis displacement may leak into physical mounting.
        auto rotated = logical;
        const auto rotation = glm::angleAxis(.7, glm::normalize(glm::dvec3(1, 2, 3)));
        const glm::dvec3 translation{11, -7, 4};
        for (auto& node : rotated.structures[0].nodes) node.position = rotation * node.position + translation;
        for (auto& member : rotated.structures[0].members)
            member.orientationReference = rotation * *member.orientationReference;
        const auto rotatedPlacement = resolveSupportMemberPlacements(rotated.structures[0]);
        require(near(placement(rotatedPlacement, tieId).start, rotation * tie.start + translation)
            && near(placement(rotatedPlacement, tieId).end, rotation * tie.end + translation),
            "two-ended placement must follow actual curved/banked authored frames under rigid rotation");
        require(track.supports() == logical, "two-ended resolution cannot modify the logical graph");
    }

    void terminalSeatIsExplicit()
    {
        BentFixture fixture;
        SupportMemberMounting seat;
        seat.mode = SupportMemberMountingMode::TerminalSeat;
        seat.face = SupportMemberMountingFace::PositiveX;
        fixture.mount(fixture.upperLedger, seat);
        const auto resolved = resolveSupportMemberPlacements(fixture.structure());
        require(near(placement(resolved, fixture.upperLedger).start, {0, 0, 5.4}),
            "explicit terminal seat places the cap bottom on the post end");
        fixture.mount(fixture.lowerLedger, seat);
        bool rejected = false;
        try { static_cast<void>(resolveSupportMemberPlacements(fixture.structure())); }
        catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "a story junction must not be treated as a terminal seat");
    }

    void generatedCurvedAndBankedFrames()
    {
        for (const bool curved : {false, true})
            for (const double bank : {-.4, 0.0, .45})
                for (const auto archetype : {HybridFramingArchetype::SimpleBent, HybridFramingArchetype::ConnectedTowers})
                {
                    auto track = curved ? createDefaultAuthoredTrack() : createNewDocument();
                    auto pose = track.startPose();
                    pose.position.z = 40;
                    pose.orientation = glm::angleAxis(.7, glm::dvec3(0, 0, 1))
                        * glm::angleAxis(-.35, glm::dvec3(0, 1, 0))
                        * glm::angleAxis(bank, glm::dvec3(1, 0, 0));
                    track.setStartPose(pose);
                    WoodenSupportRunRecipe recipe{0, 15, 5, 4, 0, -.5, .2, true};
                    recipe.family = TimberSupportFamily::HybridTimberLattice;
                    recipe.storyHeight = 10;
                    recipe.hybridArchetype = archetype;
                    static_cast<void>(generateWoodenSupportRun(track, recipe));
                    const auto logical = track.supports();
                    const auto& structure = logical.structures[0];
                    const auto resolved = resolveSupportMemberPlacements(structure);
                    const auto solid = buildSupportSolidPresentation(structure);
                    require(solid.memberCount() == structure.members.size(),
                        "every generated curved/banked Hybrid member must render");
                    bool sawBankedHost = false;
                    for (std::size_t i = 0; i < structure.members.size(); ++i)
                    {
                        const auto& member = structure.members[i];
                        const auto node = [&](const SupportElementId id) -> const SupportNode&
                        {
                            return *std::find_if(structure.nodes.begin(), structure.nodes.end(),
                                [id](const auto& value) { return value.id == id; });
                        };
                        if (member.role == SupportMemberRole::PrimaryPost)
                        {
                            require(resolved[i].start == node(member.startNodeId).position
                                && resolved[i].end == node(member.endNodeId).position,
                                "curved/banked primary posts must remain on their logical graph");
                            continue;
                        }
                        for (const bool atStart : {true, false})
                        {
                            const auto endId = atStart ? member.startNodeId : member.endNodeId;
                            const auto& mounting = *(atStart ? member.startConnection : member.endConnection)->mounting;
                            const auto incoming = std::find_if(structure.members.begin(), structure.members.end(),
                                [&](const auto& post) { return post.role == SupportMemberRole::PrimaryPost && post.endNodeId == endId; });
                            const auto host = incoming != structure.members.end() ? incoming
                                : std::find_if(structure.members.begin(), structure.members.end(),
                                    [&](const auto& post) { return post.role == SupportMemberRole::PrimaryPost && post.startNodeId == endId; });
                            require(host != structure.members.end(), "generated mounting must have an incident host");
                            const auto hostFrame = resolveSupportMemberFrame(node(host->startNodeId).position,
                                node(host->endNodeId).position, host->orientation, host->orientationReference);
                            auto normal = mounting.face == SupportMemberMountingFace::NegativeY ? -hostFrame.axisY
                                : mounting.face == SupportMemberMountingFace::PositiveY ? hostFrame.axisY : hostFrame.axisZ;
                            auto physical = atStart ? resolved[i].start : resolved[i].end;
                            double extension = mounting.overhang;
                            if (mounting.coverage == SupportMemberEndCoverage::OutsideSupport)
                            {
                                const double y = std::abs(glm::dot(resolved[i].frame.axisX, hostFrame.axisY));
                                const double z = std::abs(glm::dot(resolved[i].frame.axisX, hostFrame.axisZ));
                                extension += y >= z ? .5 * host->profile.outerDimensions.x / y
                                    : .5 * host->profile.outerDimensions.y / z;
                            }
                            physical -= (atStart ? -resolved[i].frame.axisX : resolved[i].frame.axisX) * extension;
                            const auto displacement = physical - node(endId).position;
                            require(glm::length(glm::cross(displacement, normal)) < 1e-9,
                                "generated endpoint displacement must follow its incident authored host normal");
                            if (mounting.layer == SupportMemberMountingLayer::Direct)
                                require(std::abs(glm::dot(displacement, normal) - halfExtent(host->profile, hostFrame, normal)
                                    - halfExtent(member.profile, resolved[i].frame, normal) - mounting.separation) < 1e-9,
                                    "generated contacts must use final section projection at each host");
                            if (!node(endId).trackAttachment && member.role == SupportMemberRole::LongitudinalTie)
                                require(std::abs(displacement.z) < 1e-9,
                                    "lower Hybrid mounting must stay upright/unbanked despite banked track");
                            sawBankedHost |= node(endId).trackAttachment.has_value() && std::abs(normal.z) > 1e-4;
                        }
                    }
                    require(bank == 0 || sawBankedHost, "banked attachment hosts must exercise non-world mounting normals");
                    require(track.supports() == logical, "curved/banked resolution must preserve all authored nodes and references");
                }
    }

    void persistenceDefaultsAndValidation()
    {
        BentFixture fixture;
        fixture.mountAll();
        const auto text = serializeCoasterDocument(fixture.track);
        auto loaded = deserializeCoasterDocument(text);
        require(loaded.has_value() && loaded->supports() == fixture.track.supports(),
            "mounting metadata must survive exact save/load");
        const auto expected = resolveSupportMemberPlacements(fixture.structure());
        const auto actual = resolveSupportMemberPlacements(loaded->supports().structures[0]);
        for (std::size_t i = 0; i < actual.size(); ++i)
            require(near(actual[i].start, expected[i].start) && near(actual[i].end, expected[i].end),
                "physical placement must survive save/load");
        auto json = nlohmann::json::parse(text);
        auto& members = json["supports"]["structures"][0]["members"];
        const auto& mounting = members[4]["startConnection"]["mounting"];
        require(mounting["mode"] == "Face" && mounting["face"] == "PositiveZ"
            && mounting["layer"] == "Direct" && mounting["coverage"] == "OutsideSupport",
            "mounting schema must use textual names");
        for (auto& member : members)
            for (const char* field : {"startConnection", "endConnection"})
                if (member.contains(field)) member[field].erase("mounting");
        loaded = deserializeCoasterDocument(json.dump());
        require(loaded.has_value(), "legacy connections without mounting must load");
        const auto legacy = resolveSupportMemberPlacements(loaded->supports().structures[0]);
        require(placement(legacy, fixture.lowerLedger).start == glm::dvec3(0, 0, 2),
            "old saved members cannot be retroactively repositioned");
        members[4]["startConnection"]["mounting"] = nlohmann::json::object();
        loaded = deserializeCoasterDocument(json.dump());
        require(loaded.has_value() && loaded->supports().structures[0].members[4]
            .startConnection->mounting == SupportMemberMounting{}, "partial mounting blocks must default deterministically");
        for (const auto& invalid : {nlohmann::json{{"face", "WorldX"}}, nlohmann::json{{"face", 3}},
            nlohmann::json{{"layer", "Layer2"}}, nlohmann::json{{"mode", "Unknown"}},
            nlohmann::json{{"separation", -1}}, nlohmann::json{{"overhang", -1}},
            nlohmann::json{{"coverage", "Invented"}}, nlohmann::json{{"mode", "TerminalSeat"}}})
        {
            members[4]["startConnection"]["mounting"] = invalid;
            require(!deserializeCoasterDocument(json.dump()).has_value(), "malformed mounting intent must be rejected");
        }
    }

    void exactHistoryAndRegeneration()
    {
        BentFixture fixture;
        editor::DocumentHistory history;
        history.reset(fixture.track);
        const auto before = serializeCoasterDocument(fixture.track);
        editor::AuthoredTrackEditTransaction edit{fixture.track};
        SupportMemberEndConnection connection;
        connection.mounting = SupportMemberMounting{};
        connection.mounting->separation = .017;
        edit.candidate().setSupportMemberEndConnection(fixture.id, fixture.lowerLedger, SupportMemberEnd::Start, connection);
        edit.commit(fixture.track);
        history.record(fixture.track);
        const auto after = serializeCoasterDocument(fixture.track);
        require(before != after && history.canUndo(), "mounting must participate in equality and history");
        auto undone = history.undo();
        require(undone && serializeCoasterDocument(*undone) == before, "mounting Undo must be byte-exact");
        auto redone = history.redo();
        require(redone && serializeCoasterDocument(*redone) == after, "mounting Redo must be byte-exact");
        require(near(resolveSupportMemberPlacements(redone->supports().structures[0])[4].start,
            resolveSupportMemberPlacements(fixture.structure())[4].start), "Redo must restore derived placement too");

        AuthoredTrack track = createNewDocument();
        auto pose = track.startPose();
        pose.position.z = 23;
        track.setStartPose(pose);
        WoodenSupportRunRecipe recipe{0, 20, 5, 5, 0, -.5, .3, true};
        recipe.family = TimberSupportFamily::HybridTimberLattice;
        recipe.storyHeight = 8;
        recipe.hybridArchetype = HybridFramingArchetype::ConnectedTowers;
        const auto id = generateWoodenSupportRun(track, recipe);
        const auto generated = track.supports();
        static_cast<void>(generateWoodenSupportRun(track, recipe, id));
        require(track.supports() == generated, "unchanged regeneration must preserve exact mounting intent and graph");
        for (const auto& member : generated.structures[0].members)
        {
            if (member.role == SupportMemberRole::PrimaryPost)
                require(!member.startConnection && !member.endConnection, "primary post centerlines must have no mounting");
            else
                require(member.startConnection && member.endConnection && member.startConnection->mounting
                    && member.endConnection->mounting && member.startConnection->mounting->mode == SupportMemberMountingMode::Face,
                    "all new Hybrid secondary members must author face mounting at both ends");
        }
        const auto logical = generated.structures[0].nodes;
        const auto placements = resolveSupportMemberPlacements(generated.structures[0]);
        for (std::size_t i = 0; i < placements.size(); ++i)
            if (generated.structures[0].members[i].role == SupportMemberRole::LongitudinalTie)
                require(std::abs(placements[i].start.z - placements[i].end.z) < 1e-9,
                    "staggered lower longitudinal tie elevations must remain horizontal");
        require(track.supports().structures[0].nodes == logical, "generation mounting cannot move procedural nodes");
        history.reset(track);
        const auto generationBefore = serializeCoasterDocument(track);
        editor::AuthoredTrackEditTransaction regeneration{track};
        recipe.memberSize = .45;
        static_cast<void>(generateWoodenSupportRun(regeneration.candidate(), recipe, id));
        regeneration.commit(track);
        history.record(track);
        const auto generationAfter = serializeCoasterDocument(track);
        undone = history.undo();
        require(undone && serializeCoasterDocument(*undone) == generationBefore, "regeneration Undo must be exact");
        redone = history.redo();
        require(redone && serializeCoasterDocument(*redone) == generationAfter, "regeneration Redo must be exact");
    }

    void pickingBoundsAndLogicalOverlay()
    {
        BentFixture fixture;
        fixture.mountAll();
        const auto visual = editor::createSupportVisualization(fixture.track.supports());
        require(visual.memberVertices[8].x == 0 && visual.memberVertices[8].y == 0
            && visual.memberVertices[9].x == 4 && visual.memberVertices[9].y == 0,
            "technical graph overlay must stay on logical ledger nodes");
        for (std::size_t i = 0; i < visual.nodes.size(); ++i)
            require(visual.nodes[i].position == fixture.structure().nodes[i].position,
                "node handles must stay on the logical structural graph");
        const auto bounds = editor::supportVisualizationBounds(visual);
        require(bounds.has_value() && bounds->second.y >= .67 - 1e-12,
            "framing bounds must include the outside physical brace surface");
        auto memberOnly = visual;
        std::erase_if(memberOnly.members, [&](const auto& member) { return member.selection.elementId != fixture.lowerLedger; });
        memberOnly.nodes.clear();
        memberOnly.solidPresentations.clear();
        const auto ledgerBounds = editor::supportVisualizationBounds(memberOnly);
        require(ledgerBounds && near(ledgerBounds->first, {-.25, .3, 1.6})
            && near(ledgerBounds->second, {4.25, .5, 2.4}), "ledger bounds must include overhang and oriented section");
        editor::ViewportCamera camera;
        camera.setBounds({-1, -1, 0}, {5, 2, 6});
        camera.applyPreset(editor::ViewportCameraPreset::Top);
        camera.frame(1);
        for (const auto point : {glm::dvec3(2, .4, 2), glm::dvec3(-.23, .4, 2), glm::dvec3(2, .49, 2)})
        {
            const auto projected = editor::projectViewportPoint(camera, point, 1);
            require(projected.has_value(), "physical pick fixture must project");
            const auto hit = editor::pickSupport(memberOnly, camera, projected->normalizedPosition, 1600, 1600, .1, .1);
            require(hit && hit->selection.elementId == fixture.lowerLedger,
                "solid picking must hit face displacement, axial overhang and section surface");
        }
        const auto logical = editor::projectViewportPoint(camera, {2, 0, 2}, 1);
        require(logical && !editor::pickSupport(memberOnly, camera, logical->normalizedPosition, 1600, 1600, .1, .1),
            "physical member picking must not hit the empty logical center plane");
    }
}

int main()
{
    try
    {
        legacyAndConnectorPlacementRemainCentered();
        ledgerAndBraceContactAndSectionEdits();
        twoEndedTieAndDirectedFrames();
        terminalSeatIsExplicit();
        generatedCurvedAndBankedFrames();
        persistenceDefaultsAndValidation();
        exactHistoryAndRegeneration();
        pickingBoundsAndLogicalOverlay();
    }
    catch (const std::exception& error)
    {
        std::cerr << "Support mounting test failure: " << error.what() << '\n';
        return 1;
    }
    std::cout << "Support mounting tests passed.\n";
}
