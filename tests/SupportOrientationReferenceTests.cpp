#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/SupportSolidGeometry.hpp>
#include <quantum/coaster/Supports.hpp>
#include <quantum/coaster/WoodenSupportGenerator.hpp>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <vector>

// Authored orientation references: directed bent-frame evidence stored per
// member, never sign-flipped, never re-derived on endpoint moves. Hybrid-only
// population; every other path keeps the absent (enum-fallback) form.

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

    [[nodiscard]] WoodenSupportRunRecipe hybridOneStoryRecipe()
    {
        WoodenSupportRunRecipe recipe{5.0, 25.0, 5.0, 4.0, 0.0, -0.5, 0.2,
            true};
        recipe.family = TimberSupportFamily::HybridTimberLattice;
        recipe.storyHeight = 24.0;
        return recipe;
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

    void normalizePreservesDirection()
    {
        // A directed frame vector keeps its sign: normalization scales only.
        const glm::dvec3 negative{-0.0, -3.0, -4.0};
        const glm::dvec3 normalized =
            normalizeSupportMemberOrientationReference(negative);
        require(
            near(glm::length(normalized), 1.0)
                && normalized.x == 0.0 && normalized.y < 0.0
                && normalized.z < 0.0,
            "normalization must preserve direction, never sign-flip");
        require(
            near(normalized.y, -0.6) && near(normalized.z, -0.8),
            "normalization must scale to unit length exactly");
        const glm::dvec3 oblique = normalizeSupportMemberOrientationReference({0.7, -0.3, 0.2});
        require(normalizeSupportMemberOrientationReference(oblique) == oblique,
            "normalization must be bit stable for already-unit authored references");

        for (const glm::dvec3 bad :
            {glm::dvec3{0.0, 0.0, 0.0},
             glm::dvec3{
                 std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
             glm::dvec3{
                 std::numeric_limits<double>::infinity(), 0.0, 0.0}})
        {
            bool rejected = false;
            try
            {
                static_cast<void>(
                    normalizeSupportMemberOrientationReference(bad));
            }
            catch (const std::invalid_argument&)
            {
                rejected = true;
            }
            require(rejected, "zero/non-finite references must be rejected");
        }

        // Stored state must already be unit: writers normalize.
        validateSupportMemberOrientationReference(std::nullopt);
        validateSupportMemberOrientationReference(glm::dvec3{0.0, 0.0, 1.0});
        bool rejected = false;
        try
        {
            validateSupportMemberOrientationReference(
                glm::dvec3{0.0, 0.0, 2.0});
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        require(rejected, "non-unit stored references must be rejected");
    }

    void setAndClearReference()
    {
        SupportCollection collection;
        const SupportStructureId structureId =
            createSupportStructure(collection, "Manual");
        const SupportElementId base = createSupportNode(
            collection, structureId, {0.0, 0.0, 0.0});
        const SupportElementId top = createSupportNode(
            collection, structureId, {0.0, 0.0, 5.0});
        const SupportElementId memberId = createSupportMember(
            collection, structureId, base, top);
        require(
            !collection.structures.front().members.front().orientationReference
                 .has_value(),
            "manual members default to absent references");

        // A doubled vector stores unit with its direction intact.
        setSupportMemberOrientationReference(
            collection, structureId, memberId, {0.0, 0.0, -2.0});
        const auto stored =
            collection.structures.front().members.front().orientationReference;
        require(
            stored.has_value() && near(glm::length(*stored), 1.0)
                && stored->z < 0.0 && near(stored->z, -1.0),
            "setting must normalize without flipping direction");

        bool rejected = false;
        try
        {
            setSupportMemberOrientationReference(
                collection, structureId, memberId, {0.0, 0.0, 0.0});
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        require(rejected, "zero references must be rejected");
        require(
            collection.structures.front()
                    .members.front()
                    .orientationReference.has_value(),
            "a rejected set must leave the stored reference unchanged");

        clearSupportMemberOrientationReference(
            collection, structureId, memberId);
        require(
            !collection.structures.front().members.front().orientationReference
                 .has_value(),
            "clearing must restore the enum fallback");
    }

    void nodeMovesPreserveReference()
    {
        // Endpoint edits never re-derive or drop authored evidence.
        AuthoredTrack track = elevatedTrack();
        const SupportStructureId structureId =
            track.createSupportStructure("Manual");
        const SupportElementId base =
            track.createSupportNode(structureId, {0.0, 0.0, 0.0});
        const SupportElementId top =
            track.createSupportNode(structureId, {0.0, 0.0, 5.0});
        const SupportElementId memberId = track.createSupportMember(
            structureId, base, top);
        track.setSupportMemberOrientationReference(
            structureId, memberId, {0.0, 1.0, 0.0});

        track.setSupportNodePosition(structureId, top, {1.0, 0.0, 5.0});
        const auto& members = track.supports().structures.front().members;
        require(
            members.front().orientationReference.has_value()
                && near(members.front().orientationReference->x, 0.0)
                && near(members.front().orientationReference->y, 1.0)
                && near(members.front().orientationReference->z, 0.0),
            "moving endpoints must preserve the authored reference");
        // The stale-but-valid reference still resolves instead of failing.
        const SupportSolidPresentation presentation =
            buildSupportSolidPresentation(
                track.supports().structures.front());
        require(
            presentation.memberCount() == 1,
            "a preserved reference must still present exactly once");
    }

    void hybridPopulatesReferencesDeterministically()
    {
        AuthoredTrack track = elevatedTrack();
        static_cast<void>(
            generateWoodenSupportRun(track, hybridOneStoryRecipe()));
        const SupportStructure& structure =
            track.supports().structures.front();
        for (const SupportMember& member : structure.members)
        {
            require(
                member.orientationReference.has_value(),
                "every hybrid member authors bent-frame evidence");
            validateSupportMemberOrientationReference(
                member.orientationReference);
        }

        // Bent-derived data on banked track is not a world axis in disguise:
        // at least one reference must differ from every global fallback axis.
        const glm::dvec3 axes[] = {{1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}};
        bool bentDerived = false;
        for (const SupportMember& member : structure.members)
        {
            const bool matchesAxis = std::any_of(
                std::begin(axes), std::end(axes),
                [&](const glm::dvec3& axis)
                {
                    return glm::length(*member.orientationReference - axis)
                            <= 1e-9
                        || glm::length(*member.orientationReference + axis)
                            <= 1e-9;
                });
            bentDerived = bentDerived || !matchesAxis;
        }
        require(
            bentDerived,
            "hybrid references must carry bent-frame data, not world axes");

        AuthoredTrack second = elevatedTrack();
        static_cast<void>(
            generateWoodenSupportRun(second, hybridOneStoryRecipe()));
        require(
            second.supports() == track.supports(),
            "references must regenerate deterministically, direction intact");

        // Regeneration recomputes from the bent frame, overwriting nothing
        // by hand: identical inputs reproduce identical vectors.
        require(
            generateWoodenSupportRun(
                track, hybridOneStoryRecipe(),
                track.supports().structures.front().id)
            == track.supports().structures.front().id,
            "regeneration must keep the structure identity");
        require(
            second.supports() == track.supports(),
            "regenerated references must match fresh generation exactly");
    }

    void otherFamiliesKeepAbsentReferences()
    {
        const TimberSupportFamily families[] = {
            TimberSupportFamily::TraditionalTimberBent,
            TimberSupportFamily::ModernTwisterTimber,
            TimberSupportFamily::PrefabricatedTimberLattice};
        for (const TimberSupportFamily family : families)
        {
            AuthoredTrack track = elevatedTrack();
            WoodenSupportRunRecipe recipe{5.0, 25.0, 5.0, 4.0, 0.0, -0.5,
                0.2, true};
            recipe.family = family;
            recipe.storyHeight = 18.0;
            static_cast<void>(generateWoodenSupportRun(track, recipe));
            for (const SupportMember& member :
                track.supports().structures.front().members)
            {
                require(
                    !member.orientationReference.has_value(),
                    "non-Hybrid members must not author references");
            }
        }
    }

    void generatedReferencesFollowActualMemberGeometry()
    {
        for (const double bank : {-0.5, 0.5})
        {
            for (const bool curved : {false, true})
            {
                for (const auto archetype : {HybridFramingArchetype::SimpleBent,
                    HybridFramingArchetype::ConnectedTowers})
                {
                AuthoredTrack track = curved ? createDefaultAuthoredTrack() : createNewDocument();
                AuthoredStartPose pose = track.startPose();
                pose.position.z = curved ? 40.0 : 23.0;
                pose.orientation = glm::angleAxis(0.7, glm::dvec3{0.0, 0.0, 1.0})
                    * glm::angleAxis(-0.35, glm::dvec3{0.0, 1.0, 0.0})
                    * glm::angleAxis(bank, glm::dvec3{1.0, 0.0, 0.0});
                track.setStartPose(pose);
                WoodenSupportRunRecipe recipe{0.0, 15.0, 5.0, 4.0,
                    0.0, -0.5, 0.2, true};
                recipe.family = TimberSupportFamily::HybridTimberLattice;
                recipe.storyHeight = 10.0;
                recipe.hybridArchetype = archetype;
                const AuthoredTrack source = track;
                static_cast<void>(generateWoodenSupportRun(track, recipe));
                const auto& structure = track.supports().structures.front();
                std::size_t ledgers = 0;
                std::size_t caps = 0;
                std::size_t ties = 0;
                std::size_t transverseBraces = 0;
                std::size_t longitudinalBraces = 0;
                bool nonplanarInterface = false;
                for (const auto& member : structure.members)
                {
                    const auto start = nodePosition(structure, member.startNodeId);
                    const auto end = nodePosition(structure, member.endNodeId);
                    const auto axis = glm::normalize(end - start);
                    require(member.orientationReference
                        && std::abs(glm::dot(*member.orientationReference, axis)) < 1e-9,
                        "every generated reference must be perpendicular to its own actual member");
                    const auto frame = resolveSupportMemberFrame(start, end, member.orientation,
                        member.orientationReference);
                    require(glm::dot(frame.axisY, *member.orientationReference) > 1.0 - 1e-12
                        && glm::dot(glm::cross(frame.axisY, frame.axisZ), frame.axisX) > 1.0 - 1e-12,
                        "the solid frame must consume the generated reference and remain right handed");
                    glm::dvec3 expected{};
                    if (member.role == SupportMemberRole::LedgerCap
                        || member.role == SupportMemberRole::LongitudinalTie
                        || member.role == SupportMemberRole::TrackSupport)
                    {
                        const auto post = std::find_if(structure.members.begin(), structure.members.end(),
                            [&](const SupportMember& value)
                            {
                                return value.role == SupportMemberRole::PrimaryPost
                                    && value.endNodeId == member.startNodeId;
                            });
                        require(post != structure.members.end(),
                            "each cap/ledger/tie must have an actual supporting post at its start");
                        expected = start - nodePosition(structure, post->startNodeId);
                        if (member.role == SupportMemberRole::LedgerCap)
                        {
                            const auto endpoint = std::find_if(structure.nodes.begin(), structure.nodes.end(),
                                [&](const SupportNode& value) { return value.id == member.startNodeId; });
                            if (endpoint->trackAttachment)
                            {
                                ++caps;
                                const auto otherPost = std::find_if(structure.members.begin(), structure.members.end(),
                                    [&](const SupportMember& value)
                                    {
                                        return value.role == SupportMemberRole::PrimaryPost
                                            && value.endNodeId == member.endNodeId;
                                    });
                                require(otherPost != structure.members.end(), "the interface needs two supporting posts");
                                const auto normal = glm::normalize(glm::cross(end - start, expected));
                                nonplanarInterface |= std::abs(glm::dot(normal,
                                    nodePosition(structure, otherPost->startNodeId) - start)) > 1e-3;
                            }
                            else
                            {
                                ++ledgers;
                            }
                        }
                        else
                        {
                            ++ties;
                        }
                    }
                    else if (member.orientation == SupportMemberOrientation::BentDiagonal)
                    {
                        const auto row = std::find_if(structure.members.begin(), structure.members.end(),
                            [&](const SupportMember& value)
                            {
                                return value.role == SupportMemberRole::LedgerCap
                                    && (value.startNodeId == member.startNodeId
                                        || value.endNodeId == member.startNodeId);
                            });
                        require(row != structure.members.end(), "a transverse brace must start at its actual lower ledger");
                        expected = glm::cross(nodePosition(structure, row->endNodeId)
                            - nodePosition(structure, row->startNodeId), end - start);
                        ++transverseBraces;
                    }
                    else if (member.orientation == SupportMemberOrientation::RunDiagonal)
                    {
                        const auto tie = std::find_if(structure.members.begin(), structure.members.end(),
                            [&](const SupportMember& value)
                            {
                                return (value.role == SupportMemberRole::LongitudinalTie
                                    || value.role == SupportMemberRole::TrackSupport)
                                    && value.endNodeId == member.endNodeId;
                            });
                        require(tie != structure.members.end(), "a run brace needs an explicit matched upper tie");
                        expected = glm::cross(nodePosition(structure, tie->startNodeId) - start, end - start);
                        ++longitudinalBraces;
                    }
                    else
                    {
                        continue;
                    }
                    expected = glm::normalize(expected - axis * glm::dot(expected, axis));
                    require(glm::dot(expected, *member.orientationReference) > 1.0 - 1e-12,
                        "references must follow actual incident-post or member-triangle geometry");
                }
                require(caps > 0 && ledgers > 0 && ties > 0
                    && transverseBraces > 0
                    && (archetype == HybridFramingArchetype::SimpleBent
                        ? longitudinalBraces > 0 : longitudinalBraces == 0),
                    "compound geometry must exercise every corrected member category");
                require(nonplanarInterface,
                    "combined pitch and bank must exercise a nonplanar interface, not a planar surrogate");
                const auto saved = serializeCoasterDocument(track);
                const auto restored = deserializeCoasterDocument(saved);
                require(restored && restored->supports() == track.supports(),
                    "oblique generated references must survive save/load exactly");
                require(json::parse(serializeCoasterDocument(*restored))["supports"]
                    == json::parse(saved)["supports"],
                    "serialized support data must remain bit stable after load");
                AuthoredTrack second = source;
                static_cast<void>(generateWoodenSupportRun(second, recipe));
                require(second.supports() == track.supports(),
                    "geometry-derived references must remain deterministic on curved/transition runs");
                }
            }
        }
    }

    void referenceResolvesBankedFrames()
    {
        // A run-direction member (a tie) on banked track: the banked up has
        // a lateral component the world-Z fallback cannot see. Gram-Schmidt
        // projects the reference off the axis, so width follows the banked
        // up only when the authored vector wins.
        const double bank = 0.5;
        const glm::dvec3 up{0.0, -std::sin(bank), std::cos(bank)};
        const glm::dvec3 start{0.0, 0.0, 0.0};
        const glm::dvec3 end{20.0, 0.3, 1.0};

        const SupportMemberFrame authored = resolveSupportMemberFrame(
            start, end, SupportMemberOrientation::RunLongitudinal, up);
        require(
            near(
                glm::dot(
                    glm::cross(authored.axisY, authored.axisZ),
                    authored.axisX),
                1.0),
            "banked reference frames must stay right-handed");
        require(
            glm::dot(authored.axisY, up) > 0.99,
            "an authored reference must place width with the banked frame");
        const SupportMemberFrame generic = resolveSupportMemberFrame(
            start, end, SupportMemberOrientation::RunLongitudinal);
        require(
            glm::dot(generic.axisY, up)
                < glm::dot(authored.axisY, up) - 0.05,
            "the world-axis fallback must stay bank-blind");
        require(
            glm::dot(authored.axisY, generic.axisY) < 0.999,
            "the datum must change output where the fallback approximates");

        // A stale reference parallel to the axis falls back deterministically
        // instead of corrupting the presentation.
        const glm::dvec3 axis = glm::normalize(end - start);
        const SupportMemberFrame stale = resolveSupportMemberFrame(
            start, end, SupportMemberOrientation::RunLongitudinal, axis);
        const SupportMemberFrame fallback = resolveSupportMemberFrame(
            start, end, SupportMemberOrientation::RunLongitudinal);
        require(
            near(glm::length(stale.axisY - fallback.axisY), 0.0)
                && near(glm::length(stale.axisZ - fallback.axisZ), 0.0),
            "a parallel reference must fall back to the plane reference");

        bool rejected = false;
        try
        {
            static_cast<void>(resolveSupportMemberFrame(
                start, end, SupportMemberOrientation::BentTransverse,
                glm::dvec3{
                    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}));
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        require(rejected, "non-finite references must be rejected");
    }

    void referenceSerializationRoundTrips()
    {
        AuthoredTrack track = elevatedTrack();
        static_cast<void>(
            generateWoodenSupportRun(track, hybridOneStoryRecipe()));
        const std::string serialized = serializeCoasterDocument(track);
        const json document = json::parse(serialized);
        const json& firstMember =
            document["supports"]["structures"][0]["members"][0];
        require(
            firstMember.contains("orientationReference")
                && firstMember["orientationReference"].is_object(),
            "hybrid references must persist as authored data");

        const auto restored = deserializeCoasterDocument(serialized);
        require(restored.has_value(), "reference documents must reload");
        require(
            restored->supports() == track.supports(),
            "references must round-trip exactly, direction intact");
        require(
            serializeCoasterDocument(*restored) == serialized,
            "reference serialization must stay deterministic");

        // Legacy documents without the field load through the fallback.
        json legacy = document;
        for (auto& member : legacy["supports"]["structures"][0]["members"])
        {
            member.erase("orientationReference");
        }
        const auto bare = deserializeCoasterDocument(legacy.dump());
        require(bare.has_value(), "documents without references must load");
        for (const SupportMember& member :
            bare->supports().structures.front().members)
        {
            require(
                !member.orientationReference.has_value(),
                "absent references must fall back to the enum");
        }

        // Malformed references are rejected, never repaired.
        json zero = document;
        zero["supports"]["structures"][0]["members"][0]
            ["orientationReference"] = {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}};
        require(
            !deserializeCoasterDocument(zero.dump()).has_value(),
            "zero references must be rejected");
        json partial = document;
        partial["supports"]["structures"][0]["members"][0]
            ["orientationReference"] = {{"x", 0.0}, {"y", 1.0}};
        require(
            !deserializeCoasterDocument(partial.dump()).has_value(),
            "partial references must be rejected");
        json text = document;
        text["supports"]["structures"][0]["members"][0]
            ["orientationReference"] = "up";
        require(
            !deserializeCoasterDocument(text.dump()).has_value(),
            "non-object references must be rejected");

        // Manual compactness: absent references serialize to nothing.
        SupportCollection manual;
        const SupportStructureId structureId =
            createSupportStructure(manual, "Manual");
        const SupportElementId base = createSupportNode(
            manual, structureId, {0.0, 0.0, 0.0});
        const SupportElementId top = createSupportNode(
            manual, structureId, {0.0, 0.0, 5.0});
        static_cast<void>(
            createSupportMember(manual, structureId, base, top));
        AuthoredTrack manualTrack = createNewDocument();
        manualTrack.setSupports(manual);
        const json manualDocument =
            json::parse(serializeCoasterDocument(manualTrack));
        require(
            !manualDocument["supports"]["structures"][0]["members"][0]
                 .contains("orientationReference"),
            "absent references must serialize to nothing");
    }
}

int main()
{
    try
    {
        normalizePreservesDirection();
        setAndClearReference();
        nodeMovesPreserveReference();
        hybridPopulatesReferencesDeterministically();
        otherFamiliesKeepAbsentReferences();
        generatedReferencesFollowActualMemberGeometry();
        referenceResolvesBankedFrames();
        referenceSerializationRoundTrips();
    }
    catch (const std::exception& error)
    {
        std::cerr << "Support orientation reference test failure: "
                  << error.what() << '\n';
        return 1;
    }

    std::cout << "Support orientation reference tests passed.\n";
    return 0;
}
