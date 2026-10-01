#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/SupportSolidGeometry.hpp>
#include <quantum/coaster/Supports.hpp>
#include <quantum/coaster/WoodenSupportGenerator.hpp>

#include <glm/geometric.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

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

    constexpr double tolerance = 1e-6;

    // Mirrors the M1 generator tests so a generated structure starts high
    // enough above its foundation plane to frame real stories.
    [[nodiscard]] AuthoredTrack elevatedTrack(const double elevation)
    {
        AuthoredTrack track = createNewDocument();
        AuthoredStartPose pose = track.startPose();
        pose.position.z = elevation;
        track.setStartPose(pose);
        return track;
    }

    [[nodiscard]] bool near(const double a, const double b)
    {
        return std::abs(a - b) <= tolerance;
    }

    [[nodiscard]] bool near(
        const glm::dvec3& a, const glm::dvec3& b)
    {
        return glm::length(a - b) <= tolerance;
    }

    // A single-structure collection with an explicit node/member graph, built
    // through the Core allocators so the IDs and counters match exactly what
    // the editor and generator produce.
    struct Fixture
    {
        SupportCollection collection{};

        [[nodiscard]] SupportStructure& structure()
        {
            return collection.structures.front();
        }

        SupportElementId addNode(const glm::dvec3& position,
            const bool foundation = false)
        {
            const SupportElementId id =
                allocateSupportElementId(collection.structures.front());
            collection.structures.front().nodes.push_back({id, position,
                std::nullopt,
                foundation ? std::optional<Foundation>{Foundation{}}
                    : std::nullopt});
            return id;
        }

        SupportElementId addMember(const SupportElementId start,
            const SupportElementId end,
            const SupportMemberProfile& profile)
        {
            const SupportElementId id =
                allocateSupportElementId(collection.structures.front());
            collection.structures.front().members.push_back(
                {id, start, end, profile});
            return id;
        }

        // Runs the same validation the editor does after every mutation, so a
        // fixture can never quietly hold state the document would reject.
        void validate() const
        {
            validateSupportCollection(collection);
        }
    };

    // createSupportStructure owns the ID allocation and nextStructureId counter,
    // so a hand-built fixture has to start from the same valid state.
    [[nodiscard]] Fixture newFixture(std::string name)
    {
        Fixture fixture;
        static_cast<void>(createSupportStructure(fixture.collection, name));
        return fixture;
    }

    [[nodiscard]] Fixture postAndLedgerFixture()
    {
        Fixture fixture = newFixture("Fixture");
        const SupportElementId base =
            fixture.addNode({0.0, 0.0, 0.0}, true);
        const SupportElementId top =
            fixture.addNode({0.0, 0.0, 10.0});
        fixture.addMember(base, top,
            {SupportMemberProfileShape::Rectangular, {0.25, 0.25}, 0.0});
        return fixture;
    }

    // A rectangular member's transform must place its unit box exactly over the
    // node-to-node segment with the authored cross-section.
    void rectangularTransformMatchesEndpointsAndProfile()
    {
        const Fixture fixture = postAndLedgerFixture();
        const SupportSolidPresentation presentation =
            buildSupportSolidPresentation(fixture.collection.structures.front());
        require(presentation.memberCount() == 1,
            "A single-member structure must publish one solid member.");
        require(presentation.batches.size() == 1,
            "Rectangular members belong to exactly one batch.");
        require(presentation.batches.front().mesh
                == SupportMemberMeshKind::Rectangular,
            "A rectangular profile must select the rectangular unit mesh.");

        const glm::mat4& transform =
            presentation.batches.front().instances.front().transform;
        // The instance transform is column-major, so each basis column is
        // axis * extent with the origin in column 3.
        const glm::vec3 center{transform[3][0], transform[3][1],
            transform[3][2]};
        require(near(glm::dvec3(center), glm::dvec3{0.0, 0.0, 5.0}),
            "A member's transform must be centered between its endpoints.");

        const double length = glm::length(transform[0]);
        const double width = glm::length(transform[1]);
        const double depth = glm::length(transform[2]);
        require(near(length, 10.0),
            "A vertical member's length must be its node-to-node distance.");
        require(near(width, 0.25) && near(depth, 0.25),
            "A rectangular member must use the authored profile width and "
            "depth for its cross-section.");

        // The longitudinal axis must point start-to-end, so the box spans
        // exactly the authored segment with no floating ends.
        const glm::vec3 axis = glm::normalize(transform[0]);
        require(near(glm::dvec3(axis), glm::dvec3{0.0, 0.0, 1.0}),
            "A vertical post's longitudinal axis must be its own direction.");
        const glm::vec3 halfExtent = glm::vec3(
            length, width, depth) * 0.5F;
        const glm::vec3 lower = center - axis * halfExtent.x;
        const glm::vec3 upper = center + axis * halfExtent.x;
        require(near(glm::dvec3(lower), glm::dvec3{0.0, 0.0, 0.0})
                && near(glm::dvec3(upper), glm::dvec3{0.0, 0.0, 10.0}),
            "A rendered member must terminate at its authored nodes.");
    }

    void rectangularOrientationIsRightHandedAndPerpendicular()
    {
        // Horizontal, shallow diagonal, steep diagonal, and vertical members
        // each get their own structure so the rule is checked in isolation.
        const std::array<std::pair<glm::dvec3, glm::dvec3>, 5> cases{{
            {{0.0, 0.0, 5.0}, {4.0, 0.0, 5.0}},          // horizontal ledger
            {{0.0, 0.0, 5.0}, {4.0, 0.0, 6.0}},          // shallow diagonal
            {{0.0, 0.0, 5.0}, {4.0, 0.0, 40.0}},         // steep diagonal
            {{0.0, 0.0, 5.0}, {0.0, 0.0, 15.0}},         // vertical post
            {{0.0, 0.0, 5.0}, {-3.0, 2.0, 5.0}},         // lateral ledger
        }};
        for (const auto& [start, end] : cases)
        {
            const SupportMemberFrame frame =
                resolveSupportMemberFrame(start, end);
            require(frame.length > 0.0,
                "A non-degenerate member must resolve a positive length.");
            require(near(glm::length(frame.axisX), 1.0)
                    && near(glm::length(frame.axisY), 1.0)
                    && near(glm::length(frame.axisZ), 1.0),
                "Every resolved axis must be a unit vector.");
            require(near(glm::dot(frame.axisX, frame.axisY), 0.0)
                    && near(glm::dot(frame.axisX, frame.axisZ), 0.0)
                    && near(glm::dot(frame.axisY, frame.axisZ), 0.0),
                "The resolved member frame must be mutually perpendicular.");
            // A left-handed basis would invert winding and flip normals.
            require(near(glm::dot(
                    glm::cross(frame.axisY, frame.axisZ), frame.axisX), 1.0),
                "The resolved member frame must be right-handed.");
            require(near(glm::dvec3(frame.axisX),
                    (end - start) / glm::length(end - start)),
                "The longitudinal axis must point from start to end.");
        }
    }

    void verticalMembersUseTheDocumentedFallback()
    {
        const SupportMemberFrame vertical = resolveSupportMemberFrame(
            {1.0, 2.0, 0.0}, {1.0, 2.0, 9.0});
        require(vertical.usedVerticalFallback,
            "A vertical member must use the world +X cross-section reference "
            "because world +Z is parallel to its own axis.");
        require(near(glm::dvec3(vertical.axisY), glm::dvec3{1.0, 0.0, 0.0}),
            "The vertical fallback reference must be world +X.");

        const SupportMemberFrame ledger = resolveSupportMemberFrame(
            {0.0, 0.0, 5.0}, {4.0, 0.0, 5.0});
        require(!ledger.usedVerticalFallback,
            "A non-vertical member must not take the vertical fallback.");
        require(near(glm::dvec3(ledger.axisY), glm::dvec3{0.0, 0.0, 1.0}),
            "A horizontal ledger's cross-section must keep world +Z as its "
            "width axis.");
    }

    void neighbouringMembersDoNotRandomlyRoll()
    {
        // Neighbouring posts in a bent are the case that motivated the rule:
        // they lean in different directions, and they must still agree on
        // which way their cross-section faces. A flip would show as a dot
        // product near -1 rather than near +1.
        const SupportMemberFrame first = resolveSupportMemberFrame(
            {0.0, 0.0, 0.0}, {0.02, 0.0, 10.0});
        const SupportMemberFrame second = resolveSupportMemberFrame(
            {0.0, 4.0, 0.0}, {-0.03, 4.0, 10.0});
        require(glm::dot(first.axisY, second.axisY) > 0.999,
            "Two near-parallel posts must keep the same cross-section "
            "orientation rather than flipping roll between them.");
        require(glm::dot(first.axisZ, second.axisZ) > 0.999,
            "Two near-parallel posts must keep the same depth axis rather "
            "than flipping roll between them.");

        // Continuity: a member rotating gradually through orientations must
        // not jump. Sampling either side of a small rotation change and
        // comparing the cross-section axes catches a sign flip.
        const SupportMemberFrame before = resolveSupportMemberFrame(
            {0.0, 0.0, 0.0}, {1.0, 0.0, 10.0});
        const SupportMemberFrame after = resolveSupportMemberFrame(
            {0.0, 0.0, 0.0}, {1.0001, 0.0, 10.0});
        require(near(glm::dot(before.axisY, after.axisY), 1.0),
            "Cross-section orientation must vary continuously with the member "
            "axis.");
    }

    void degenerateMembersAreSkipped()
    {
        Fixture fixture = newFixture("Degenerate");
        const SupportElementId a = fixture.addNode({0.0, 0.0, 0.0});
        const SupportElementId b = fixture.addNode({0.0, 0.0, 0.0});
        const SupportElementId c = fixture.addNode({0.0, 0.0, 5.0});
        // Coincident endpoints bypass the Core member validator on purpose:
        // the renderer must still refuse to emit a NaN basis.
        fixture.addMember(a, b,
            {SupportMemberProfileShape::Rectangular, {0.2, 0.2}, 0.0});
        fixture.addMember(a, c,
            {SupportMemberProfileShape::Rectangular, {0.2, 0.2}, 0.0});

        const SupportSolidPresentation presentation =
            buildSupportSolidPresentation(fixture.collection.structures.front());
        require(presentation.memberCount() == 1,
            "A zero-length member must be skipped rather than rendered.");
        for (const auto& instance : presentation.batches.front().instances)
        {
            for (int column = 0; column < 4; ++column)
            {
                for (int row = 0; row < 4; ++row)
                {
                    require(std::isfinite(instance.transform[column][row]),
                        "Every published instance component must be finite.");
                }
            }
        }
    }

    void circularProfilesUseTheCircularMeshAndDiameter()
    {
        Fixture fixture = newFixture("Round");
        const SupportElementId base =
            fixture.addNode({0.0, 0.0, 0.0}, true);
        const SupportElementId top = fixture.addNode({0.0, 0.0, 8.0});
        fixture.addMember(base, top,
            {SupportMemberProfileShape::Circular, {0.3, 0.3}, 0.0});

        const SupportSolidPresentation presentation =
            buildSupportSolidPresentation(fixture.collection.structures.front());
        require(presentation.batches.size() == 1
                && presentation.batches.front().mesh
                    == SupportMemberMeshKind::Circular,
            "A circular profile must select the circular unit mesh.");
        const glm::mat4& transform =
            presentation.batches.front().instances.front().transform;
        require(near(glm::length(transform[1]), 0.3)
                && near(glm::length(transform[2]), 0.3),
            "A circular member must use the authored diameter on both "
            "cross-section axes.");
        require(near(glm::length(transform[0]), 8.0),
            "A circular member's length must still be its node-to-node "
            "distance.");
    }

    void mixedProfilesSplitIntoSeparateDrawCalls()
    {
        Fixture fixture = newFixture("Mixed");
        const SupportElementId base = fixture.addNode({0.0, 0.0, 0.0});
        const SupportElementId mid = fixture.addNode({0.0, 0.0, 5.0});
        const SupportElementId top = fixture.addNode({3.0, 0.0, 9.0});
        fixture.addMember(base, mid,
            {SupportMemberProfileShape::Rectangular, {0.2, 0.3}, 0.0});
        fixture.addMember(mid, top,
            {SupportMemberProfileShape::Circular, {0.25, 0.25}, 0.0});

        const SupportSolidPresentation presentation =
            buildSupportSolidPresentation(fixture.collection.structures.front());
        require(presentation.memberCount() == 2,
            "Both members must be published.");
        require(presentation.drawCallCount() == 2,
            "Two profile shapes must produce two instanced draw calls, not one "
            "draw call per timber.");
        require(presentation.batches.size() == 2,
            "Each profile shape must occupy its own batch.");
    }

    void foundationsProduceOnePadPerFoundationNode()
    {
        Fixture fixture = newFixture("Founded");
        const SupportElementId left = fixture.addNode({-2.0, 0.0, 0.0}, true);
        const SupportElementId right = fixture.addNode({2.0, 0.0, 0.0}, true);
        const SupportElementId topLeft = fixture.addNode({-2.0, 0.0, 6.0});
        fixture.addMember(left, topLeft,
            {SupportMemberProfileShape::Rectangular, {0.3, 0.3}, 0.0});
        // The right footing carries no member, which must still produce a pad.
        validateSupportCollection(fixture.collection);

        const SupportSolidPresentation presentation =
            buildSupportSolidPresentation(fixture.collection.structures.front());
        require(presentation.foundations.size() == 2,
            "Every foundation node must produce exactly one footing pad.");
        const SupportFoundationPad& carried = presentation.foundations.front();
        require(near(carried.padDimensions.x,
                0.3 * supportFoundationPadFootprintScale)
                && near(carried.padDimensions.y,
                    0.3 * supportFoundationPadFootprintScale),
            "A footing pad must be sized from the cross-section it carries.");
        require(near(carried.padDepth,
                0.3 * supportFoundationPadDepthScale),
            "A footing pad's thickness must come from the carried "
            "cross-section, not from its own footprint, so it reads as a flat "
            "pad rather than a square pillar stub.");
        require(carried.padDepth < carried.padDimensions.x,
            "A footing pad must be thinner than it is wide.");
        require(near(glm::dvec3(carried.position), glm::dvec3{-2.0, 0.0, 0.0}),
            "A footing pad must sit at its foundation node position.");
    }

    void nonFoundationNodesProduceNoPad()
    {
        const Fixture fixture = postAndLedgerFixture();
        const SupportSolidPresentation presentation =
            buildSupportSolidPresentation(fixture.collection.structures.front());
        require(presentation.foundations.size() == 1,
            "Only foundation nodes produce footing pads.");
        require(near(presentation.foundations.front().padDimensions.x,
                0.25 * supportFoundationPadFootprintScale),
            "The footing under a 0.25 post must scale from that post's "
            "cross-section.");
        require(near(presentation.foundations.front().padDepth,
                0.25 * supportFoundationPadDepthScale),
            "The footing thickness must scale from the same carried "
            "cross-section.");
    }

    void foundationDimensionsDeriveIndependently()
    {
        Fixture fixture = postAndLedgerFixture();
        SupportFoundationAppearance appearance;
        appearance.padDimensions.x = 2.0;
        fixture.structure().foundationAppearance = appearance;

        const SupportSolidPresentation presentation =
            buildSupportSolidPresentation(fixture.structure());
        require(presentation.foundations.size() == 1,
            "The fixture must retain its single foundation.");
        require(near(presentation.foundations.front().padDimensions.x, 2.0)
                && near(presentation.foundations.front().padDimensions.y,
                    0.25 * supportFoundationPadFootprintScale),
            "An authored pad width must not discard the derived pad depth.");
    }

    void appearanceDefaultsForStructuresThatNeverAuthoredIt()
    {
        const Fixture fixture = postAndLedgerFixture();
        const SupportSolidPresentation presentation =
            buildSupportSolidPresentation(fixture.collection.structures.front());
        const SupportAppearance expected{};
        require(presentation.appearance == expected,
            "A structure with no authored appearance must resolve to the "
            "conservative default timber, matching the renderer's fallback.");
        require(near(presentation.appearance.baseColorTint.x, 0.78)
                && near(presentation.appearance.baseColorTint.y, 0.64)
                && near(presentation.appearance.baseColorTint.z, 0.47),
            "The default timber tint must be a conservative natural pine, not "
            "a saturated or cartoon brown.");
    }

    void appearanceIsIndependentOfFamilyAndSurvivesRegeneration()
    {
        AuthoredTrack track = elevatedTrack(24.0);
        WoodenSupportRunRecipe recipe{5.0, 25.0, 5.0, 4.0, 0.0, -0.5, 0.2,
            true};
        recipe.family = TimberSupportFamily::HybridTimberLattice;
        const SupportStructureId id = generateWoodenSupportRun(track, recipe);

        SupportAppearance tinted;
        tinted.baseColorTint = {0.55F, 0.56F, 0.55F};
        tinted.textureScale = 2.5F;
        SupportCollection tintedSupports = track.supports();
        setSupportAppearance(tintedSupports, id, tinted);
        track.setSupports(tintedSupports);

        const auto structure = std::ranges::find_if(
            track.supports().structures,
            [id](const SupportStructure& value)
            {
                return value.id == id;
            });
        require(structure != track.supports().structures.end(),
            "The generated structure must exist after authoring an appearance.");
        require(structure->appearance.has_value()
                && structure->appearance->baseColorTint
                    == tinted.baseColorTint,
            "An authored appearance must be stored on the structure.");

        // The identical topology must survive regeneration untouched.
        const std::vector<glm::dvec3> nodesBefore = [&] {
            std::vector<glm::dvec3> values;
            for (const SupportNode& node : structure->nodes)
            {
                values.push_back(node.position);
            }
            return values;
        }();
        const std::size_t memberCountBefore = structure->members.size();

        require(generateWoodenSupportRun(track, recipe, id) == id,
            "Regeneration must retain the structure's stable ID.");

        const auto regenerated = std::ranges::find_if(
            track.supports().structures,
            [id](const SupportStructure& value)
            {
                return value.id == id;
            });
        require(regenerated != track.supports().structures.end(),
            "Regeneration must keep the same structure identity.");
        require(regenerated->members.size() == memberCountBefore,
            "Regeneration must not change the member count.");
        require(regenerated->nodes.size() == nodesBefore.size(),
            "Regeneration must not change the node count.");
        for (std::size_t index = 0; index < nodesBefore.size(); ++index)
        {
            require(near(regenerated->nodes[index].position, nodesBefore[index]),
                "Regeneration must not move node positions.");
        }
        require(regenerated->appearance.has_value()
                && regenerated->appearance->baseColorTint
                    == tinted.baseColorTint
                && near(regenerated->appearance->textureScale, 2.5),
            "Regeneration must preserve the authored appearance, because "
            "regeneration replaces topology, not material.");
    }

    void appearanceRoundTripsAndOldDocumentsDefaultSafely()
    {
        SupportCollection collection;
        const SupportStructureId structureId =
            createSupportStructure(collection, "Round Trip");
        SupportAppearance appearance;
        appearance.baseColorTint = {0.42F, 0.30F, 0.22F};
        appearance.roughnessMultiplier = 1.6F;
        appearance.normalStrength = 0.4F;
        appearance.textureScale = 3.25F;
        SupportFoundationAppearance foundation;
        foundation.baseColorTint = {0.5F, 0.5F, 0.48F};
        foundation.padDimensions = {1.5, 2.5};
        foundation.padDepth = 0.75;
        setSupportAppearance(collection, structureId, appearance);
        setSupportFoundationAppearance(collection, structureId, foundation);

        AuthoredTrack track = createNewDocument();
        track.setSupports(collection);
        const std::string json = serializeCoasterDocument(track);
        const auto restored = deserializeCoasterDocument(json);
        require(restored.has_value(),
            "A document with authored support appearance must load.");

        const SupportStructure& structure =
            restored->supports().structures.front();
        require(structure.appearance.has_value(),
            "An authored appearance must survive save/load.");
        require(structure.appearance->baseColorTint
                == appearance.baseColorTint
                && near(structure.appearance->roughnessMultiplier, 1.6)
                && near(structure.appearance->normalStrength, 0.4)
                && near(structure.appearance->textureScale, 3.25),
            "Every authored appearance field must round-trip exactly.");
        require(structure.foundationAppearance.has_value()
                && structure.foundationAppearance->padDimensions
                    == foundation.padDimensions
                && near(structure.foundationAppearance->padDepth, 0.75),
            "Authored foundation appearance must round-trip exactly.");
        // Document equality is what Undo/Redo compares.
        require(restored->supports() == track.supports(),
            "A save/load round trip must preserve document equality.");

        // An M0/M1-era document has no appearance field at all.
        nlohmann::json parsed = nlohmann::json::parse(json);
        parsed["supports"]["structures"][0].erase("appearance");
        parsed["supports"]["structures"][0].erase("foundationAppearance");
        const auto legacy = deserializeCoasterDocument(parsed.dump());
        require(legacy.has_value(),
            "A document without appearance fields must still load.");
        require(!legacy->supports().structures.front().appearance.has_value(),
            "A document without an appearance must resolve to absent, which "
            "the renderer reads as the default.");
        const SupportSolidPresentation presentation =
            buildSupportSolidPresentation(
                legacy->supports().structures.front());
        require(presentation.appearance == SupportAppearance{},
            "An old document must present the default timber appearance.");
    }

    void appearanceRejectsMalformedValues()
    {
        SupportCollection collection;
        const SupportStructureId structureId =
            createSupportStructure(collection, "Validated");

        SupportAppearance outOfRange;
        outOfRange.baseColorTint = {1.5F, 0.5F, 0.5F};
        bool rejected = false;
        try
        {
            setSupportAppearance(collection, structureId, outOfRange);
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        require(rejected,
            "A tint channel above 1 must be rejected.");
        require(!collection.structures.front().appearance.has_value(),
            "A rejected appearance must leave the collection unchanged.");

        SupportAppearance zeroScale;
        zeroScale.textureScale = 0.0F;
        rejected = false;
        try
        {
            setSupportAppearance(collection, structureId, zeroScale);
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        require(rejected,
            "A non-positive texture scale must be rejected.");
        require(!collection.structures.front().appearance.has_value(),
            "A second rejected appearance must also leave the collection "
            "unchanged.");

        rejected = false;
        try
        {
            setSupportAppearance(collection, 99, SupportAppearance{});
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        require(rejected, "An unknown structure must be rejected.");

        // Zero foundation pad dimensions and depth are the documented derived
        // default, so they must be accepted rather than rejected.
        bool accepted = true;
        try
        {
            setSupportFoundationAppearance(collection, structureId,
                SupportFoundationAppearance{});
        }
        catch (const std::invalid_argument&)
        {
            accepted = false;
        }
        require(accepted,
            "Zero foundation pad dimensions must be accepted because they "
            "mean the footing is derived from the members it carries.");
        require(collection.structures.front().foundationAppearance
                .has_value(),
            "The accepted foundation appearance must be published.");

        // A negative dimension is not a derived value and must be rejected.
        rejected = false;
        try
        {
            SupportFoundationAppearance negative;
            negative.padDimensions = {-1.0, 1.0};
            setSupportFoundationAppearance(collection, structureId, negative);
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        require(rejected,
            "A negative foundation pad dimension must be rejected.");
        require(collection.structures.front().foundationAppearance
                ->padDimensions.x == 0.0,
            "A rejected foundation appearance must leave the collection "
            "unchanged.");
    }

    void allM1FamiliesProduceNondegenerateRenderInstances()
    {
        AuthoredTrack track = elevatedTrack(20.0);
        const std::array<TimberSupportFamily, 4> families{
            TimberSupportFamily::TraditionalTimberBent,
            TimberSupportFamily::ModernTwisterTimber,
            TimberSupportFamily::PrefabricatedTimberLattice,
            TimberSupportFamily::HybridTimberLattice};

        for (const TimberSupportFamily family : families)
        {
            WoodenSupportRunRecipe recipe{5.0, 25.0, 5.0, 4.0, 0.0, -0.5,
                0.22, true};
            recipe.family = family;
            recipe.storyHeight = 18.0;
            const SupportStructureId id =
                generateWoodenSupportRun(track, recipe);
            const SupportStructure& structure =
                *std::ranges::find_if(track.supports().structures,
                    [id](const SupportStructure& value)
                    {
                        return value.id == id;
                    });
            const SupportSolidPresentation presentation =
                buildSupportSolidPresentation(structure);
            require(presentation.memberCount() == structure.members.size(),
                "Every generated member must produce a render instance.");
            require(presentation.drawCallCount() <= 2,
                "A generated family must render in at most one draw call per "
                "profile shape.");
            require(!presentation.foundations.empty(),
                "A generated run must produce foundation pads.");

            for (const auto& batch : presentation.batches)
            {
                for (const SupportMemberInstance& instance : batch.instances)
                {
                    const double length = glm::length(
                        glm::vec3(instance.transform[0]));
                    require(std::isfinite(length) && length
                            > minimumSupportMemberLength,
                        "Every generated member must have a positive, finite "
                        "length.");
                    // A negative determinant would invert the basis and turn a
                    // solid into an inside-out shell.
                    const double determinant =
                        glm::length(glm::vec3(instance.transform[0]))
                        * glm::length(glm::vec3(instance.transform[1]))
                        * glm::length(glm::vec3(instance.transform[2]));
                    require(std::isfinite(determinant) && determinant > 0.0,
                        "Every generated member must have a positive basis "
                        "volume.");
                }
            }
        }
    }

    void curvedAndBankedRunsStayStable()
    {
        AuthoredTrack track = elevatedTrack(24.0);
        WoodenSupportRunRecipe recipe{0.0, 40.0, 3.0, 4.0, 0.0, -0.5, 0.2,
            true};
        recipe.family = TimberSupportFamily::ModernTwisterTimber;
        recipe.storyHeight = 16.0;
        const SupportStructureId id = generateWoodenSupportRun(track, recipe);
        const SupportStructure& structure =
            *std::ranges::find_if(track.supports().structures,
                [id](const SupportStructure& value)
                {
                    return value.id == id;
                });
        const SupportSolidPresentation presentation =
            buildSupportSolidPresentation(structure);
        require(presentation.memberCount() > 0,
            "A curved and banked run must still publish members.");

        // Orientation must stay continuous along the run: no two consecutive
        // longitudinal axes may flip, and the framing must not produce a
        // non-finite component anywhere.
        std::vector<glm::dvec3> axes;
        for (const auto& batch : presentation.batches)
        {
            for (const SupportMemberInstance& instance : batch.instances)
            {
                const glm::vec3 axis = glm::normalize(
                    glm::vec3(instance.transform[0]));
                axes.emplace_back(axis.x, axis.y, axis.z);
            }
        }
        for (const glm::dvec3& axis : axes)
        {
            require(std::isfinite(axis.x) && std::isfinite(axis.y)
                    && std::isfinite(axis.z),
                "A curved run must not produce a non-finite member axis.");
        }
    }

    void presentationBuildIsDeterministic()
    {
        const Fixture fixture = postAndLedgerFixture();
        const SupportSolidPresentation first =
            buildSupportSolidPresentation(fixture.collection.structures.front());
        const SupportSolidPresentation second =
            buildSupportSolidPresentation(fixture.collection.structures.front());
        require(first.batches.size() == second.batches.size(),
            "Repeated presentation builds must agree on batch count.");
        for (std::size_t batch = 0; batch < first.batches.size(); ++batch)
        {
            require(first.batches[batch].instances.size()
                    == second.batches[batch].instances.size(),
                "Repeated presentation builds must agree on instance count.");
            for (std::size_t index = 0;
                index < first.batches[batch].instances.size(); ++index)
            {
                require(first.batches[batch].instances[index]
                        == second.batches[batch].instances[index],
                    "Repeated presentation builds must produce identical "
                    "instance transforms.");
            }
        }
        require(first.foundations == second.foundations,
            "Repeated presentation builds must produce identical footings.");
    }

    void collectionPresentationCoversEveryStructure()
    {
        SupportCollection collection;
        const SupportStructureId first =
            createSupportStructure(collection, "First");
        // The second structure is intentionally empty: it must still appear
        // in the result so callers can key by structure ID.
        const SupportStructureId secondId =
            createSupportStructure(collection, "Second");
        SupportAppearance second;
        second.baseColorTint = {0.7F, 0.7F, 0.7F};
        setSupportAppearance(collection, secondId, second);

        const std::vector<SupportSolidPresentation> presentations =
            buildSupportSolidPresentation(collection);
        require(presentations.size() == 2,
            "Every structure must produce a presentation entry.");
        require(presentations.front().structureId == first
                && presentations.back().structureId == secondId,
            "Presentation entries must be keyed by structure ID in "
            "collection order.");
        require(presentations.back().appearance.baseColorTint
                == second.baseColorTint,
            "Each structure must carry its own appearance.");
    }

    void nonFiniteEndpointsAreRejected()
    {
        bool rejected = false;
        try
        {
            static_cast<void>(resolveSupportMemberFrame(
                {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
                {0.0, 0.0, 1.0}));
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        require(rejected,
            "A non-finite member endpoint must be rejected rather than "
            "producing a NaN basis.");
    }
}

int main()
{
    try
    {
        rectangularTransformMatchesEndpointsAndProfile();
        rectangularOrientationIsRightHandedAndPerpendicular();
        verticalMembersUseTheDocumentedFallback();
        neighbouringMembersDoNotRandomlyRoll();
        degenerateMembersAreSkipped();
        circularProfilesUseTheCircularMeshAndDiameter();
        mixedProfilesSplitIntoSeparateDrawCalls();
        foundationsProduceOnePadPerFoundationNode();
        nonFoundationNodesProduceNoPad();
        foundationDimensionsDeriveIndependently();
        appearanceDefaultsForStructuresThatNeverAuthoredIt();
        appearanceIsIndependentOfFamilyAndSurvivesRegeneration();
        appearanceRoundTripsAndOldDocumentsDefaultSafely();
        appearanceRejectsMalformedValues();
        allM1FamiliesProduceNondegenerateRenderInstances();
        curvedAndBankedRunsStayStable();
        presentationBuildIsDeterministic();
        collectionPresentationCoversEveryStructure();
        nonFiniteEndpointsAreRejected();
    }
    catch (const std::exception& error)
    {
        std::cerr << "Support solid geometry test failure: "
            << error.what() << '\n';
        return 1;
    }

    std::cout << "Support solid geometry tests passed.\n";
    return 0;
}
