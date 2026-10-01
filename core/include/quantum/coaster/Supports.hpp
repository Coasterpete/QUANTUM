#pragma once

#include <quantum/coaster/StaticMeshAsset.hpp>

#include <glm/gtc/quaternion.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace quantum::coaster
{
    // Structure identity is document-wide. Node and member identities share
    // one structure-local element namespace so every support element remains
    // unambiguous when paired with its owning structure ID.
    using SupportStructureId = std::uint32_t;
    using SupportElementId = std::uint32_t;

    inline constexpr SupportStructureId invalidSupportStructureId = 0;
    inline constexpr SupportElementId invalidSupportElementId = 0;

    enum class SupportMemberProfileShape : std::uint8_t
    {
        Circular,
        Rectangular
    };

    // The two directed ends of a member. Connections are authored per end and
    // remain independent of the node at that end.
    enum class SupportMemberEnd : std::uint8_t
    {
        Start,
        End
    };

    // The fixed set of member-end treatments. Serialization matches these
    // exact names (never numeric codes) so moving JSON between documents
    // cannot change treatment semantics.
    enum class SupportMemberEndTreatment : std::uint8_t
    {
        MiteredCut,
        EndCap,
        Plate,
        Flange,
        // M2B records an unpaired splice marker only; splice partners and
        // splice pairing are a later milestone.
        Splice,
        Saddle,
        Clamp,
        Base,
        Footing
    };

    // Optional logical placement of the member-end connector mesh relative to
    // the member's local frame at the affected end. When absent, the renderer
    // chooses the connector pose from support modeling conventions.
    struct SupportMemberEndPlacement
    {
        glm::dvec3 position{0.0};
        // Canonical unit quaternion (w, x, y, z), same convention as
        // AuthoredStartPose::orientation.
        glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
        glm::dvec3 scale{1.0};

        [[nodiscard]] friend bool operator==(
            const SupportMemberEndPlacement&,
            const SupportMemberEndPlacement&) = default;
    };

    enum class SupportMemberMountingMode : std::uint8_t
    {
        Face,
        TerminalSeat
    };

    // Signed axes of the supporting post frame: X follows the post chain away
    // from its foundation, Y follows its directed orientationReference, and Z
    // completes that basis. These are not the mounted member's local axes.
    enum class SupportMemberMountingFace : std::uint8_t
    {
        PositiveY,
        NegativeY,
        PositiveZ,
        NegativeZ,
        PositiveX,
        NegativeX
    };

    enum class SupportMemberMountingLayer : std::uint8_t
    {
        Direct,
        // Clear the actual outer surface of incident same-face ledgers/ties.
        OutsideLedger
    };

    enum class SupportMemberEndCoverage : std::uint8_t
    {
        Node,
        OutsideSupport
    };

    // Member-body placement intent, independent of connector localPlacement.
    // The host is the incident PrimaryPost chain (the lower segment at a story
    // junction). Distances are Core units; no world offset is stored. Section
    // edits therefore recompute both contact clearance and end coverage.
    struct SupportMemberMounting
    {
        SupportMemberMountingMode mode = SupportMemberMountingMode::Face;
        SupportMemberMountingFace face = SupportMemberMountingFace::PositiveZ;
        SupportMemberMountingLayer layer = SupportMemberMountingLayer::Direct;
        double separation = 0.0;
        SupportMemberEndCoverage coverage = SupportMemberEndCoverage::Node;
        double overhang = 0.0;

        [[nodiscard]] friend bool operator==(
            const SupportMemberMounting&, const SupportMemberMounting&) = default;
    };

    // Persistent authored connection metadata for one member end. No separate
    // connection ID exists: identity derives from (structureId, memberId,
    // SupportMemberEnd), so deleting a member removes both end connections.
    struct SupportMemberEndConnection
    {
        SupportMemberEndTreatment treatment =
            SupportMemberEndTreatment::MiteredCut;
        std::optional<StaticMeshAssetReference> asset;
        std::optional<SupportMemberEndPlacement> localPlacement;
        std::optional<SupportMemberMounting> mounting;

        [[nodiscard]] friend bool operator==(
            const SupportMemberEndConnection&,
            const SupportMemberEndConnection&) = default;
    };

    // Cross-section geometry only. A zero wall thickness denotes a solid
    // member; a positive thickness denotes a hollow tube or box section.
    struct SupportMemberProfile
    {
        SupportMemberProfileShape shape =
            SupportMemberProfileShape::Circular;
        glm::dvec2 outerDimensions{0.1, 0.1};
        double wallThickness = 0.0;

        [[nodiscard]] friend bool operator==(
            const SupportMemberProfile&,
            const SupportMemberProfile&) = default;
    };

    // Authored placement on the document's single ordered track path.
    // Station and offsets use Core coordinate units, not SI metres. The
    // resolved world position/frame are derived from current track geometry
    // and are deliberately not persisted here.
    struct TrackAttachment
    {
        double station = 0.0;
        double lateralOffset = 0.0;
        double verticalOffset = 0.0;

        [[nodiscard]] friend bool operator==(
            const TrackAttachment&,
            const TrackAttachment&) = default;
    };

    // Marks a node as an explicitly authored foundation/ground anchor. Its
    // SupportNode::position remains authoritative; terrain projection and
    // terrain-follow metadata are future additive concerns.
    struct Foundation
    {
        [[nodiscard]] friend bool operator==(
            const Foundation&,
            const Foundation&) = default;
    };

    struct SupportNode
    {
        SupportElementId id = invalidSupportElementId;
        glm::dvec3 position{0.0, 0.0, 0.0};
        std::optional<TrackAttachment> trackAttachment;
        std::optional<Foundation> foundation;

        [[nodiscard]] friend bool operator==(
            const SupportNode&, const SupportNode&) = default;
    };

    // Structural role of a timber member. This is authored semantics, not
    // inferred geometry: a shallow brace and a shallow ledger can share an
    // angle but never share a role. Roles exist so primary posts,
    // transverse ledgers/caps, longitudinal ties, diagonals, and demonstrated
    // track-support members can carry appropriate independent sections.
    //
    // The first accuracy target is the existing RMC/Hybrid reference material
    // (two-post bents on discrete foundations, a raised lower ledger, one
    // repeated diagonal per bent/story, upper caps, longitudinal ties, and
    // mostly open bays). Ledger and cap share one role for now because both
    // are transverse horizontals; a future split is additive if the reference
    // demands distinct cap sizing. TrackSupport requires an established
    // stringer/support function; row height alone is insufficient. Transverse
    // caps remain LedgerCap and upper post connections remain PrimaryPost.
    enum class SupportMemberRole : std::uint8_t
    {
        // Documents written before roles existed, plus manually authored
        // members that never picked a role. Serialized as an absent field.
        Unspecified,
        // Primary post segments, including upright inner posts and inclined
        // outer supports; inclination alone does not make a member a brace.
        PrimaryPost,
        // Transverse horizontals: intermediate story ledgers, the top-story
        // cap, and the Hybrid raised lower ledger.
        LedgerCap,
        // Longitudinal structural ties, including Hybrid upper/lower ties
        // and the other families' current ground-level foundation ties.
        LongitudinalTie,
        // Any diagonal: transverse bent braces and selected longitudinal bay
        // braces alike. Both are secondary bracing, not primary framing.
        Brace,
        // Members with a demonstrated track-support/stringer function.
        // The simple Hybrid generator currently creates upper ties instead.
        TrackSupport
    };

    // Intended rectangular-section alignment of a member inside its
    // structural frame. This is authored intent, not inferred geometry: a
    // transverse bent diagonal and a longitudinal bay diagonal can share the
    // Brace role yet lie in different planes, so the plane lives here rather
    // than in a proliferation of near-duplicate roles.
    //
    // Hybrid members name their structural context. The solid frame consumes
    // the optional geometry-derived reference; the enum chooses a fallback
    // when that reference is absent or parallel to an edited member axis.
    enum class SupportMemberOrientation : std::uint8_t
    {
        // Renderer fallback (world-Z projected, world-X for verticals).
        // Used by manual/legacy members and every non-Hybrid generator path.
        Generic,
        // Hybrid primary post lines: section faces aligned to the bent frame.
        BentPost,
        // Hybrid transverse horizontals (ledgers, caps, raised lower ledger).
        BentTransverse,
        // Hybrid longitudinal ties, including the upper attachment row.
        RunLongitudinal,
        // Hybrid diagonals lying in the transverse bent plane.
        BentDiagonal,
        // Hybrid diagonals lying in a longitudinal run-vertical plane.
        RunDiagonal
    };

    // Authored structural roll/frame evidence for one member: the directed
    // cross-section reference its bent was framed with, expressed in document
    // coordinates. This is evidence, not a re-derivation: moving a member's
    // endpoints preserves it, and only an explicit orientation edit replaces
    // it. The Hybrid generator constructs it deterministically from actual
    // rows, incident posts and member-specific triangles --
    // normalization keeps the direction, never flips the vector to enforce a
    // generic sign convention. Absent means "use the orientation-enum
    // fallback", which keeps legacy, manual, and non-Hybrid members compact
    // and byte-identical.
    using SupportMemberOrientationReference = std::optional<glm::dvec3>;

    struct SupportMember
    {
        SupportElementId id = invalidSupportElementId;
        SupportElementId startNodeId = invalidSupportElementId;
        SupportElementId endNodeId = invalidSupportElementId;
        SupportMemberProfile profile;
        SupportMemberRole role = SupportMemberRole::Unspecified;
        SupportMemberOrientation orientation = SupportMemberOrientation::Generic;
        SupportMemberOrientationReference orientationReference;
        std::optional<SupportMemberEndConnection> startConnection;
        std::optional<SupportMemberEndConnection> endConnection;

        [[nodiscard]] friend bool operator==(
            const SupportMember&, const SupportMember&) = default;
    };

    enum class TimberSupportFamily : std::uint8_t
    {
        TraditionalTimberBent,
        ModernTwisterTimber,
        PrefabricatedTimberLattice,
        HybridTimberLattice
    };

    // QUANTUM framing choices, not manufacturer engineering categories.
    enum class HybridFramingArchetype : std::uint8_t
    {
        Automatic,
        SimpleBent,
        ConnectedTowers
    };

    enum class HybridLongitudinalBracing : std::uint8_t
    {
        Open,
        SingleDiagonal
    };

    // First/last name the directed bent's post lanes, not screen or world axes.
    enum class HybridDiagonalDirection : std::uint8_t
    {
        LowerFirstToUpperLast,
        LowerLastToUpperFirst
    };

    struct HybridTransversePanelChoice
    {
        std::uint32_t towerIndex = 0;
        std::uint32_t panelIndex = 0;
        // +/- Z of the foundation-directed post frame are the bent faces.
        SupportMemberMountingFace face = SupportMemberMountingFace::PositiveZ;
        HybridDiagonalDirection direction = HybridDiagonalDirection::LowerFirstToUpperLast;

        [[nodiscard]] friend bool operator==(
            const HybridTransversePanelChoice&, const HybridTransversePanelChoice&) = default;
    };

    struct HybridLongitudinalPanelChoice
    {
        std::uint32_t bayIndex = 0;
        std::uint32_t panelIndex = 0;
        std::uint32_t laneIndex = 0;
        HybridLongitudinalBracing bracing = HybridLongitudinalBracing::Open;

        [[nodiscard]] friend bool operator==(
            const HybridLongitudinalPanelChoice&, const HybridLongitudinalPanelChoice&) = default;
    };

    // Local additions around the unchanged two-post core. Left/right follow
    // the negative/positive unbanked lateral direction in station order.
    enum class HybridOuterSupportSides : std::uint8_t
    {
        None,
        Left,
        Right,
        Both
    };

    struct HybridOuterSupportChoice
    {
        std::uint32_t towerIndex = 0;
        HybridOuterSupportSides sides = HybridOuterSupportSides::None;
        // Authored Core-unit distances outward from the corresponding inner
        // post, at the foundation and lower-frame shoulder. No width/height
        // rule chooses these values; the defaults are visual approximations.
        double foundationOutset = 4.0;
        double topOutset = 1.0;

        [[nodiscard]] friend bool operator==(
            const HybridOuterSupportChoice&, const HybridOuterSupportChoice&) = default;
    };

    // Authored presentation data for one support structure. It is
    // deliberately independent of TimberSupportFamily: generator topology
    // decides how a structure stands up, this decides how its members look.
    // Two structures generated from the same recipe can therefore carry
    // different appearances, and the same structure keeps its appearance
    // across regeneration.
    //
    // The timber base-color map is authored as neutral grayscale detail, so
    // baseColorTint is the dominant timber color and the texture supplies
    // grain, knots, and local brightness variation rather than hue.
    //
    // textureScale is the length in Core coordinate units that one repeat of
    // the timber maps spans. It is applied to object-space UVs, which keeps
    // grain physically sized across long posts, short braces, and ledgers
    // alike.
    struct SupportAppearance
    {
        // sRGB, matching TrackMaterial and GroundAppearance so every surface
        // in the scene is decoded through the same conversion path.
        glm::vec3 baseColorTint{0.78F, 0.64F, 0.47F};
        float roughnessMultiplier = 1.0F;
        float normalStrength = 1.0F;
        float textureScale = 1.0F;

        [[nodiscard]] friend bool operator==(
            const SupportAppearance&, const SupportAppearance&) = default;
    };

    // Foundation footings are rendered as neutral concrete rather than
    // timber. They are presentation only: the Foundation node position stays
    // authoritative and no footing dimension is derived from structural
    // loads.
    struct SupportFoundationAppearance
    {
        glm::vec3 baseColorTint{0.62F, 0.62F, 0.60F};
        float roughness = 0.92F;
        // Footing footprint and depth are expressed in Core coordinate units,
        // matching SupportMemberProfile::outerDimensions. Zero means the
        // renderer derives a pad from the member cross-section.
        glm::dvec2 padDimensions{0.0, 0.0};
        double padDepth = 0.0;

        [[nodiscard]] friend bool operator==(
            const SupportFoundationAppearance&,
            const SupportFoundationAppearance&) = default;
    };

    // Rejects a non-finite component, a tint channel outside [0, 1], a
    // non-positive or non-finite roughness/normal/texture scale, or a negative
    // or non-finite foundation dimension. Zero foundation pad dimensions and
    // depth are valid and mean "derive the footing from the members this
    // foundation carries". Callers validate before publication so a rejected
    // value can never half-apply.
    void validateSupportAppearance(const SupportAppearance& appearance);
    void validateSupportFoundationAppearance(
        const SupportFoundationAppearance& appearance);

    // A generated run owns one whole structure. Regeneration replaces only
    // that structure; manual structures have no recipe.
    struct WoodenSupportRunRecipe
    {
        double startStation = 0.0;
        double endStation = 40.0;
        double bentSpacing = 5.0;
        double bentWidth = 4.0;
        double foundationElevation = -10.0;
        double attachmentVerticalOffset = -0.5;
        double memberSize = 0.2;
        // Legacy run-wide policy for SimpleBent and the other families.
        // ConnectedTowers uses only its explicit local panel choices below.
        bool longitudinalBracing = true;
        TimberSupportFamily family = TimberSupportFamily::TraditionalTimberBent;
        // Maximum vertical distance between connected framing levels.
        double storyHeight = 24.0;
        HybridFramingArchetype hybridArchetype = HybridFramingArchetype::Automatic;
        // Sparse explicit overrides in station order, with zero-based panels
        // bottom-up. Transverse panels start at the raised lower ledger.
        // ConnectedTowers run panels lie between consecutive local tie rows;
        // laneIndex selects one post-side plane. Omitted run panels are Open.
        // Choices for absent panels remain dormant in the recipe, not remapped.
        std::vector<HybridTransversePanelChoice> hybridTransversePanels;
        std::vector<HybridLongitudinalPanelChoice> hybridLongitudinalPanels;
        // Omitted towers have no outer lines. Absent tower indices remain
        // dormant, like the local panel choices above.
        std::vector<HybridOuterSupportChoice> hybridOuterSupports;

        [[nodiscard]] friend bool operator==(
            const WoodenSupportRunRecipe&,
            const WoodenSupportRunRecipe&) = default;
    };

    struct SupportStructure
    {
        SupportStructureId id = invalidSupportStructureId;
        std::string name;
        std::vector<SupportNode> nodes;
        std::vector<SupportMember> members;
        SupportElementId nextElementId = 1;
        std::optional<WoodenSupportRunRecipe> generatedWoodenRun;
        // Absent in documents written before Supports M2A, which resolve to
        // the conservative default timber appearance.
        std::optional<SupportAppearance> appearance;
        std::optional<SupportFoundationAppearance> foundationAppearance;

        [[nodiscard]] friend bool operator==(
            const SupportStructure&, const SupportStructure&) = default;
    };

    struct SupportCollection
    {
        std::vector<SupportStructure> structures;
        SupportStructureId nextStructureId = 1;

        [[nodiscard]] bool empty() const noexcept
        {
            return structures.empty();
        }

        [[nodiscard]] friend bool operator==(
            const SupportCollection&, const SupportCollection&) = default;
    };

    // The profile created for manually connected members. A simple valid
    // solid round section; the M1A manual toolset authoring does not pick
    // materials or manufacturer-specific sections yet.
    [[nodiscard]] SupportMemberProfile defaultSupportMemberProfile() noexcept;

    // Allocation advances the persisted counter and never searches for or
    // reuses gaps. Malformed counters and exhausted ID spaces are rejected.
    [[nodiscard]] SupportStructureId allocateSupportStructureId(
        SupportCollection& collection);
    [[nodiscard]] SupportElementId allocateSupportElementId(
        SupportStructure& structure);

    // Invariant-preserving topology editing. Every mutation operates on a
    // live SupportCollection, owns allocator behavior (IDs are never
    // caller-supplied), validates its inputs before changing anything, and
    // finishes by validating the complete collection. On a rejected mutation
    // the collection is left exactly unchanged; deleted IDs are never reused
    // and allocator counters never roll back.
    //
    // Creates an empty structure, allocating its nextSupportStructureId and a
    // deterministic default name ("Support N") when name is empty.
    // Throws std::invalid_argument when the structure ID space is exhausted.
    [[nodiscard]] SupportStructureId createSupportStructure(
        SupportCollection& collection,
        std::string name = {});
    // Removes one structure and its entire owned node/member graph.
    // Throws std::invalid_argument for an unknown structure ID.
    void removeSupportStructure(
        SupportCollection& collection,
        SupportStructureId structureId);
    // Adds one node to the target structure, allocating its stable element ID.
    // Throws std::invalid_argument for an unknown structure or a non-finite
    // position.
    [[nodiscard]] SupportElementId createSupportNode(
        SupportCollection& collection,
        SupportStructureId structureId,
        const glm::dvec3& position);
    // Refuses to remove a node referenced by any member: the connected members
    // must be deleted first. Throws std::invalid_argument for an unknown
    // node or a node still referenced by a member.
    void removeSupportNode(
        SupportCollection& collection,
        SupportStructureId structureId,
        SupportElementId nodeId);
    // Connects two distinct nodes of the same structure. Rejects another
    // member across the same unordered endpoint pair, so A->B and B->A count
    // as the same connection and coincident duplicate members cannot be
    // authored. Uses defaultSupportMemberProfile() when no profile is given.
    // Throws std::invalid_argument for missing endpoints, a self connection,
    // or an existing member across the same pair.
    [[nodiscard]] SupportElementId createSupportMember(
        SupportCollection& collection,
        SupportStructureId structureId,
        SupportElementId startNodeId,
        SupportElementId endNodeId,
        const SupportMemberProfile& profile = defaultSupportMemberProfile(),
        SupportMemberRole role = SupportMemberRole::Unspecified,
        SupportMemberOrientation orientation = SupportMemberOrientation::Generic,
        SupportMemberOrientationReference orientationReference = std::nullopt);
    // Removes exactly one member, preserving every node. Throws
    // std::invalid_argument for an unknown member ID.
    void removeSupportMember(
        SupportCollection& collection,
        SupportStructureId structureId,
        SupportElementId memberId);

    // Node anchor metadata uses the node's existing stable identity. Setting
    // one anchor kind replaces no other state and rejects a node already
    // carrying the mutually exclusive anchor kind.
    void setSupportTrackAttachment(
        SupportCollection& collection,
        SupportStructureId structureId,
        SupportElementId nodeId,
        const TrackAttachment& attachment);
    void clearSupportTrackAttachment(
        SupportCollection& collection,
        SupportStructureId structureId,
        SupportElementId nodeId);
    void setSupportFoundation(
        SupportCollection& collection,
        SupportStructureId structureId,
        SupportElementId nodeId);
    void clearSupportFoundation(
        SupportCollection& collection,
        SupportStructureId structureId,
        SupportElementId nodeId);

    // Member-end connection metadata uses the member's existing stable
    // identity; no separate connection IDs are allocated. Deleting the member
    // deletes both end connections. The two ends are independent and never
    // merge with the node's anchor metadata. The supplied connection is
    // normalized (including its placement quaternion) before publication.
    // Throws std::invalid_argument for unknown members, malformed connection
    // values, or a Saddle/Clamp treatment at a node without a track
    // attachment / Base/Footing treatment at a node without a foundation.
    void setSupportMemberEndConnection(
        SupportCollection& collection,
        SupportStructureId structureId,
        SupportElementId memberId,
        SupportMemberEnd end,
        const SupportMemberEndConnection& connection);
    void clearSupportMemberEndConnection(
        SupportCollection& collection,
        SupportStructureId structureId,
        SupportElementId memberId,
        SupportMemberEnd end);
    // Explicit orientation edits. Setting normalizes (direction preserved)
    // and stores the reference; moving endpoints never touches it. Clearing
    // restores the enum fallback. Throws std::invalid_argument for unknown
    // members or malformed references.
    void setSupportMemberOrientationReference(
        SupportCollection& collection,
        SupportStructureId structureId,
        SupportElementId memberId,
        const glm::dvec3& reference);
    void clearSupportMemberOrientationReference(
        SupportCollection& collection,
        SupportStructureId structureId,
        SupportElementId memberId);

    // Appearance metadata uses the structure's existing stable identity and
    // never allocates an ID. Setting one replaces only that field, so timber
    // and foundation appearance stay independent of each other and of the
    // generator recipe. Each call validates its input and the finished
    // collection; on rejection the collection is left exactly unchanged.
    // Throws std::invalid_argument for an unknown structure or a malformed
    // appearance.
    void setSupportAppearance(
        SupportCollection& collection,
        SupportStructureId structureId,
        const SupportAppearance& appearance);
    void setSupportFoundationAppearance(
        SupportCollection& collection,
        SupportStructureId structureId,
        const SupportFoundationAppearance& appearance);

    // Returns a canonical package-relative identifier for a connector asset
    // below assets://support/. File-backed connectors must use the .glb
    // extension.
    [[nodiscard]] std::string normalizeSupportConnectorAssetIdentifier(
        std::string_view identifier);

    // Normalizes member-end connection state so that a stored placement
    // quaternion is finite, unit, and sign-canonical per Core conventions.
    // Throws std::invalid_argument for malformed values.
    [[nodiscard]] SupportMemberEndConnection normalizeSupportMemberEndConnection(
        const SupportMemberEndConnection& connection);
    void validateSupportMemberEndConnection(
        const SupportMemberEndConnection& connection);
    void validateSupportMemberMounting(const SupportMemberMounting& mounting);

    // Geometric/document consistency only. This does not perform loads,
    // stress, buckling, foundation, or code-compliance analysis.
    void validateSupportMemberProfile(const SupportMemberProfile& profile);
    void validateSupportMemberRole(SupportMemberRole role);
    void validateSupportMemberOrientation(SupportMemberOrientation orientation);

    // Normalizes an authored orientation reference to unit length, preserving
    // its direction exactly: no sign canonicalization is applied, so a
    // generator-built directed frame round-trips bit-identically. Throws
    // std::invalid_argument for a non-finite or zero-length vector.
    [[nodiscard]] glm::dvec3 normalizeSupportMemberOrientationReference(
        const glm::dvec3& reference);
    // Absent references are valid (enum fallback). A present reference must
    // be finite and unit-length within 1e-9; writers normalize, so stored
    // state that is not unit is rejected rather than repaired.
    void validateSupportMemberOrientationReference(
        const SupportMemberOrientationReference& reference);

    // Role-specific rectangular timber sections derived from the recipe's
    // nominal member size. These are visual presentation proportions informed
    // by the RMC/Hybrid reference language (posts read heaviest, braces
    // lightest), not structural sizing or load analysis. Posts keep the square
    // nominal section so foundation pads derived from the carried section stay
    // stable; every other role is a distinct non-square rectangle.
    [[nodiscard]] SupportMemberProfile timberProfileForRole(
        SupportMemberRole role,
        double memberSize);
    void validateWoodenSupportRunRecipe(const WoodenSupportRunRecipe& recipe);
    void validateSupportCollection(const SupportCollection& collection);
}
